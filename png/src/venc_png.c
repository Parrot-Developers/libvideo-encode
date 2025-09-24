/**
 * Copyright (c) 2023 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Drones SAS Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define ULOG_TAG venc_png
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "venc_png_priv.h"
#include <setjmp.h>

#define NB_SUPPORTED_ENCODINGS 1
#define NB_SUPPORTED_FORMATS 6

static struct vdef_raw_format supported_formats[NB_SUPPORTED_FORMATS];
static enum vdef_encoding supported_encodings[NB_SUPPORTED_ENCODINGS];
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;

struct mem_encode {
	png_bytep data;
	size_t capacity;
	size_t len;
};


static void initialize_supported_formats(void)
{
	supported_formats[0] = vdef_gray;
	supported_formats[1] = vdef_gray16;
	supported_formats[2] = vdef_raw8;
	supported_formats[3] = vdef_raw16;
	supported_formats[4] = vdef_raw16_be;
	supported_formats[5] = vdef_rgb;
	supported_encodings[0] = VDEF_ENCODING_PNG;
}


static void call_flush_done(void *userdata)
{
	struct venc_png *self = userdata;

	venc_call_flush_cb(self->base);
}


static void call_stop_done(void *userdata)
{
	struct venc_png *self = userdata;

	venc_call_stop_cb(self->base);
}


static void mbox_cb(int fd, uint32_t revents, void *userdata)
{
	struct venc_png *self = userdata;
	int ret, err;
	char message;

	do {
		/* Read from the mailbox */
		ret = mbox_peek(self->mbox, &message);
		if (ret < 0) {
			if (ret != -EAGAIN)
				VENC_LOG_ERRNO("mbox_peek", -ret);
			break;
		}

		switch (message) {
		case VENC_MSG_FLUSH:
			err = pomp_loop_idle_add_with_cookie(
				self->base->loop, call_flush_done, self, self);
			if (err < 0) {
				VENC_LOG_ERRNO("pomp_loop_idle_add_with_cookie",
					       -err);
			}
			break;
		case VENC_MSG_STOP:
			if ((venc_count_unreleased_frames(self->base) == 0)) {
				err = pomp_loop_idle_add_with_cookie(
					self->base->loop,
					call_stop_done,
					self,
					self);
				if (err < 0) {
					VENC_LOG_ERRNO(
						"pomp_loop_idle_add_"
						"with_cookie",
						-err);
				} else {
					atomic_store(&self->stopping, false);
				}
			} else {
				atomic_store(&self->stopping, true);
			}
			break;
		default:
			VENC_LOGE("unknown message: %c", message);
			break;
		}
	} while (ret == 0);
}


static void enc_out_queue_evt_cb(struct pomp_evt *evt, void *userdata)
{
	struct venc_png *self = userdata;
	struct mbuf_coded_video_frame *out_frame = NULL;
	int ret;
	do {
		ret = mbuf_coded_video_frame_queue_pop(self->enc_out_queue,
						       &out_frame);
		if (ret == -EAGAIN) {
			return;
		} else if (ret < 0) {
			VENC_LOG_ERRNO(
				"mbuf_coded_video_frame_queue_pop:enc_out",
				-ret);
			return;
		}
		venc_call_frame_output_cb(self->base, 0, out_frame);
		mbuf_coded_video_frame_unref(out_frame);
	} while (ret == 0);
}


static int fill_frame(struct venc_png *self,
		      struct mbuf_raw_video_frame *in_frame,
		      struct mbuf_coded_video_frame *out_frame,
		      unsigned char *out_picture,
		      unsigned int out_picture_size)
{
	int ret = 0;
	struct vmeta_frame *metadata = NULL;
	struct vdef_nalu nalu = {0};
	void *nalu_data;
	struct mbuf_mem *mem = NULL;

	ret = mbuf_raw_video_frame_foreach_ancillary_data(
		in_frame,
		mbuf_coded_video_frame_ancillary_data_copier,
		out_frame);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_foreach_ancillary_data",
			       -ret);
		goto out;
	}

	/* Frame metadata */
	ret = mbuf_raw_video_frame_get_metadata(in_frame, &metadata);
	if (ret == 0) {
		ret = mbuf_coded_video_frame_set_metadata(out_frame, metadata);
		if (ret < 0) {
			VENC_LOG_ERRNO("mbuf_coded_video_frame_set_metadata",
				       -ret);
			goto out;
		}
	} else if ((ret < 0) && (ret != -ENOENT)) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_get_metadata", -ret);
		goto out;
	}

	ret = mbuf_mem_generic_new(out_picture_size, &mem);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_mem_generic_new", -ret);
		goto out;
	}

	ret = mbuf_mem_get_data(mem, &nalu_data, &nalu.size);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_mem_get_data", -ret);
		goto out;
	}
	nalu.importance = 0;

	memcpy(nalu_data, out_picture, out_picture_size);
	ret = mbuf_coded_video_frame_add_nalu(out_frame, mem, 0, &nalu);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_mem_get_data", -ret);
		mem = NULL;
		goto out;
	}

out:
	if (mem) {
		ret = mbuf_mem_unref(mem);
		if (ret != 0)
			VENC_LOG_ERRNO("mbuf_mem_unref", -ret);
	}
	if (metadata)
		vmeta_frame_unref(metadata);
	return ret;
}


static void
png_write_data_cb(png_structp png_ptr, png_bytep data, png_size_t length)
{
	struct mem_encode *p = (struct mem_encode *)png_get_io_ptr(png_ptr);
	size_t nsize = p->len + length;

	/* Allocate or grow buffer */
	if ((p->data == NULL) || (nsize > p->capacity)) {
		p->data = (png_bytep)realloc(p->data, nsize);
		if (!p->data)
			png_error(png_ptr, "Write Error");
		else
			p->capacity = nsize;
	}

	/* Copy new bytes to end of buffer */
	if (p->data) {
		memcpy(p->data + p->len, data, length);
		p->len += length;
	}
}


/* Can be called from any thread */
static void frame_release(struct mbuf_coded_video_frame *frame, void *userdata)
{
	struct venc_png *self = userdata;

	venc_call_pre_release_cb(self->base, frame);

	if (atomic_load(&self->stopping) &&
	    (venc_count_unreleased_frames(self->base) == 0)) {
		int err = pomp_loop_idle_add_with_cookie(
			self->base->loop, call_stop_done, self, self);
		if (err < 0)
			VENC_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
		else
			atomic_store(&self->stopping, false);
	}
}


static int encode_frame(struct venc_png *self,
			struct mbuf_raw_video_frame *in_frame)
{
	int ret, err;
	size_t len;
	struct vdef_raw_frame in_info;
	const void *plane_data;
	struct mbuf_coded_video_frame *out_frame = NULL;
	struct vdef_coded_frame out_info = {};
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;
	png_structp png_handle = NULL;
	png_infop png_info = NULL;
	/* codecheck_ignore[VOLATILE] */
	png_bytepp volatile row_pointers = NULL;
	struct mem_encode mem_out = {0};
	size_t frame_size = 0;

	struct mbuf_coded_video_frame_cbs frame_cbs = {
		.pre_release = frame_release,
		.pre_release_userdata = (void *)self,
	};

	if (!in_frame)
		return 0;

	ret = mbuf_raw_video_frame_get_frame_info(in_frame, &in_info);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		return ret;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	if (!vdef_raw_format_intersect(
		    &in_info.format, supported_formats, NB_SUPPORTED_FORMATS)) {
		ret = -ENOSYS;
		VENC_LOG_ERRNO(
			"unsupported format:"
			" " VDEF_RAW_FORMAT_TO_STR_FMT,
			-ret,
			VDEF_RAW_FORMAT_TO_STR_ARG(&in_info.format));
		return ret;
	}

	if (self->in_picture.planes > SOFTPNG_MAX_PLANES) {
		VENC_LOGE("invalid number of planes: %d",
			  self->in_picture.planes);
		return -EINVAL;
	}

	for (unsigned int i = 0; i < self->in_picture.planes; i++) {
		ret = mbuf_raw_video_frame_get_plane(
			in_frame, i, &plane_data, &len);
		if (ret < 0) {
			VENC_LOG_ERRNO(
				"mbuf_raw_video_frame_get_plane(%u)", -ret, i);
			return ret;
		}

		self->in_picture.plane[i] = (uint8_t *)plane_data;
		self->in_picture.stride[i] = in_info.plane_stride[i];
		mbuf_raw_video_frame_release_plane(in_frame, i, plane_data);
		frame_size += len;
	}

	err = mbuf_raw_video_frame_add_ancillary_buffer(
		in_frame,
		VENC_ANCILLARY_KEY_DEQUEUE_TIME,
		&ts_us,
		sizeof(ts_us));
	if (err < 0)
		VENC_LOGW_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
				-err);

	/* allocate out buffer to be used by libpng to avoid multiple realloc
	 * in the libpng write data callback */
	mem_out.capacity = frame_size;
	mem_out.data = (png_bytep)malloc(mem_out.capacity);
	if (!mem_out.data) {
		VENC_LOGE("failed to allocate output data");
		return -ENOMEM;
	}

	png_handle = png_create_write_struct(
		PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (!png_handle) {
		VENC_LOGE("failed to create png write struct");
		ret = -ENOMEM;
		goto out;
	}

	self->base->counters.pushed++;

	png_info = png_create_info_struct(png_handle);
	if (!png_info) {
		VENC_LOGE("failed to create png info struct");
		ret = -ENOMEM;
		goto out;
	}

	png_set_IHDR(png_handle,
		     png_info,
		     self->in_picture.width,
		     self->in_picture.height,
		     self->in_picture.bit_depth,
		     self->in_picture.color_type,
		     PNG_INTERLACE_NONE,
		     PNG_COMPRESSION_TYPE_DEFAULT,
		     PNG_FILTER_TYPE_DEFAULT);

	png_set_write_fn(png_handle, &mem_out, png_write_data_cb, NULL);

	png_set_compression_level(png_handle,
				  (int)self->in_picture.compression_level);

	png_write_info(png_handle, png_info);

	if (self->in_picture.data_little_endian)
		png_set_swap(png_handle);

	row_pointers =
		(png_bytepp)malloc(sizeof(png_bytep) * self->in_picture.height);
	if (!row_pointers) {
		VENC_LOGE("failed to allocate row pointers");
		ret = -ENOMEM;
		goto out;
	}

	for (unsigned int idx = 0; idx < self->in_picture.height; idx++)
		row_pointers[idx] =
			(png_byte *)(self->in_picture.plane[0] +
				     (idx * self->in_picture.stride[0]));

	if (setjmp(png_jmpbuf(png_handle))) {
		VENC_LOGE("png jump error handling");
		ret = -EIO;
		goto out;
	}

	png_write_image(png_handle, row_pointers);
	png_write_end(png_handle, NULL);

	self->base->counters.pulled++;

	out_info.info = in_info.info;
	out_info.format = self->output_format;
	out_info.type = VDEF_CODED_FRAME_TYPE_CODED;
	out_info.layer = 0;

	/* Coded frame creation */
	ret = mbuf_coded_video_frame_new(&out_info, &out_frame);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_new", -ret);
		goto out;
	}
	ret = mbuf_coded_video_frame_set_callbacks(out_frame, &frame_cbs);
	if (ret < 0)
		VENC_LOG_ERRNO("mbuf_coded_video_frame_set_callbacks", -ret);

	ret = fill_frame(self, in_frame, out_frame, mem_out.data, mem_out.len);
	if (ret < 0)
		goto out;

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	err = mbuf_coded_video_frame_add_ancillary_buffer(
		out_frame,
		VENC_ANCILLARY_KEY_OUTPUT_TIME,
		&ts_us,
		sizeof(ts_us));
	if (err < 0) {
		VENC_LOGW_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-err);
	}

	/* Output the frame */
	ret = mbuf_coded_video_frame_finalize(out_frame);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_finalize", -ret);
		goto out;
	}

	ret = mbuf_coded_video_frame_queue_push(self->enc_out_queue, out_frame);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_queue_push", -ret);
		goto out;
	}

out:
	if (out_frame)
		mbuf_coded_video_frame_unref(out_frame);

	if (mem_out.data)
		free(mem_out.data);

	if (row_pointers)
		free(row_pointers);

	if (png_handle)
		png_destroy_write_struct(&png_handle, &png_info);

	return ret;
}


static int complete_flush(struct venc_png *self)
{
	int ret;

	if (atomic_load(&self->flush_discard)) {

		/* Flush the input queue */
		ret = mbuf_raw_video_frame_queue_flush(self->in_queue);
		if (ret < 0) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_flush:input",
				       -ret);
			return ret;
		}

		/* Flush the encoder output queue */
		ret = mbuf_coded_video_frame_queue_flush(self->enc_out_queue);
		if (ret < 0) {
			VENC_LOG_ERRNO(
				"mbuf_coded_video_frame_queue_flush:enc_out",
				-ret);
			return ret;
		}
	}

	atomic_store(&self->flushing, false);
	atomic_store(&self->flush_discard, false);

	/* Call the flush callback on the loop */
	char message = VENC_MSG_FLUSH;
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		VENC_LOG_ERRNO("mbox_push", -ret);

	return ret;
}


static void check_input_queue(struct venc_png *self)
{
	int ret, err = 0;
	struct mbuf_raw_video_frame *in_frame;

	ret = mbuf_raw_video_frame_queue_peek(self->in_queue, &in_frame);
	while (ret >= 0) {
		ret = encode_frame(self, in_frame);
		if (ret < 0) {
			if (ret != -EAGAIN)
				VENC_LOG_ERRNO("encode_frame", -ret);
			err = -ENOSPC;
		}
		if (in_frame) {
			mbuf_raw_video_frame_unref(in_frame);
			/* Pop the frame for real */
			ret = mbuf_raw_video_frame_queue_pop(self->in_queue,
							     &in_frame);
			if (ret < 0) {
				VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_pop",
					       -ret);
				break;
			}
			mbuf_raw_video_frame_unref(in_frame);
		}
		if (err)
			break;
		/* Peek the next frame */
		ret = mbuf_raw_video_frame_queue_peek(self->in_queue,
						      &in_frame);
		if (ret < 0 && ret != -EAGAIN && ret != -ENOSPC)
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_peek", -ret);
		if (self->flushing && ret == -EAGAIN) {
			in_frame = NULL;
			if ((atomic_load(&self->flushing)) &&
			    (!atomic_load(&self->flush_discard))) {
				ret = complete_flush(self);
				if (ret < 0)
					VENC_LOG_ERRNO("complete_flush", -ret);
				continue;

				/* Else we proceed to call encode_frame()
				 * without an input frame to flush the
				 * encoder */
			}
		}
	}

	if (self->flushing && ret == -EAGAIN) {
		if (((atomic_load(&self->flushing)) &&
		     (!atomic_load(&self->flush_discard)))) {
			ret = complete_flush(self);
			if (ret < 0)
				VENC_LOG_ERRNO("complete_flush", -ret);
		}
	}
}


static void input_event_cb(struct pomp_evt *evt, void *userdata)
{
	struct venc_png *self = userdata;
	check_input_queue(self);
}


static void *encoder_thread(void *ptr)
{
	int ret, timeout;
	struct venc_png *self = ptr;
	struct pomp_loop *loop = NULL;
	struct pomp_evt *in_queue_evt = NULL;
	char message;

#if defined(__APPLE__)
#	if !TARGET_OS_IPHONE
	ret = pthread_setname_np("venc_png");
	if (ret != 0)
		VENC_LOG_ERRNO("pthread_setname_np", ret);
#	endif
#else
	ret = pthread_setname_np(pthread_self(), "venc_png");
	if (ret != 0)
		VENC_LOG_ERRNO("pthread_setname_np", ret);
#endif

	loop = pomp_loop_new();
	if (!loop) {
		VENC_LOG_ERRNO("pomp_loop_new", ENOMEM);
		goto exit;
	}

	ret = mbuf_raw_video_frame_queue_get_event(self->in_queue,
						   &in_queue_evt);
	if (ret != 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_queue_get_event", -ret);
		goto exit;
	}

	ret = pomp_evt_attach_to_loop(in_queue_evt, loop, input_event_cb, self);
	if (ret != 0) {
		VENC_LOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto exit;
	}

	while ((!atomic_load(&self->should_stop)) ||
	       (atomic_load(&self->flushing))) {
		/* Start flush, discarding all frames */
		if ((atomic_load(&self->flushing)) &&
		    (atomic_load(&self->flush_discard))) {
			ret = complete_flush(self);
			if (ret < 0)
				VENC_LOG_ERRNO("complete_flush", -ret);
			continue;
		}

		/* Wait for an input buffer (without dequeueing it) */
		timeout = ((atomic_load(&self->flushing)) &&
			   (!atomic_load(&self->flush_discard)))
				  ? 0
				  : 5;
		ret = pomp_loop_wait_and_process(loop, timeout);
		if (ret < 0 && ret != -ETIMEDOUT) {
			VENC_LOG_ERRNO("pomp_loop_wait_and_process", -ret);
			if (!self->should_stop) {
				/* Avoid looping on errors */
				usleep(5000);
			}
			continue;
		} else if (ret == -ETIMEDOUT) {
			check_input_queue(self);
		}
	}

	/* Call the stop callback on the loop */
	message = VENC_MSG_STOP;
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		VENC_LOG_ERRNO("mbox_push", -ret);

exit:
	if (in_queue_evt != NULL) {
		ret = pomp_evt_detach_from_loop(in_queue_evt, loop);
		if (ret != 0)
			VENC_LOG_ERRNO("pomp_evt_detach_from_loop", -ret);
	}
	if (loop != NULL) {
		ret = pomp_loop_destroy(loop);
		if (ret != 0)
			VENC_LOG_ERRNO("pomp_loop_destroy", -ret);
	}

	return NULL;
}


static int get_supported_encodings(const enum vdef_encoding **encodings)
{
	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);
	*encodings = supported_encodings;
	return NB_SUPPORTED_ENCODINGS;
}


static int get_supported_input_formats(enum vdef_encoding encoding,
				       const struct vdef_raw_format **formats)
{
	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);
	*formats = supported_formats;
	return NB_SUPPORTED_FORMATS;
}


static int flush(struct venc_encoder *base, int discard_)
{
	struct venc_png *self;
	bool discard = discard_;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	atomic_store(&self->flushing, true);
	atomic_store(&self->flush_discard, discard);

	return 0;
}


static int stop(struct venc_encoder *base)
{
	struct venc_png *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	/* Stop the encoding thread */
	atomic_store(&self->should_stop, true);
	return 0;
}


static int destroy(struct venc_encoder *base)
{
	int err;
	struct venc_png *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;
	if (self == NULL)
		return 0;

	/* Stop and join the encoding thread */
	err = stop(base);
	if (err < 0)
		VENC_LOG_ERRNO("stop", -err);
	if (self->thread_launched) {
		err = pthread_join(self->thread, NULL);
		if (err != 0)
			VENC_LOG_ERRNO("pthread_join", err);
	}

	/* Free the resources */
	if (self->in_queue != NULL) {
		err = mbuf_raw_video_frame_queue_destroy(self->in_queue);
		if (err < 0)
			VENC_LOG_ERRNO(
				"mbuf_raw_video_frame_queue_destroy:input",
				-err);
	}
	if (self->enc_out_queue_evt != NULL &&
	    pomp_evt_is_attached(self->enc_out_queue_evt, base->loop)) {
		err = pomp_evt_detach_from_loop(self->enc_out_queue_evt,
						base->loop);
		if (err < 0)
			VENC_LOG_ERRNO("pomp_evt_detach_from_loop", -err);
	}
	if (self->enc_out_queue != NULL) {
		err = mbuf_coded_video_frame_queue_destroy(self->enc_out_queue);
		if (err < 0)
			VENC_LOG_ERRNO(
				"mbuf_coded_video_frame_queue_destroy:enc_out",
				-err);
	}
	if (self->mbox) {
		err = pomp_loop_remove(base->loop,
				       mbox_get_read_fd(self->mbox));
		if (err < 0)
			VENC_LOG_ERRNO("pomp_loop_remove", -err);
		mbox_destroy(self->mbox);
	}

	err = pomp_loop_idle_remove_by_cookie(base->loop, self);
	if (err < 0)
		VENC_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	for (unsigned int i = 0; i < self->in_picture.planes; i++)
		self->in_picture.plane[i] = NULL;

	xfree((void **)&self);
	return 0;
}


static bool input_filter(struct mbuf_raw_video_frame *frame, void *userdata)
{
	int ret;
	struct venc_png *self = userdata;
	struct vdef_raw_frame info;

	VENC_LOG_ERRNO_RETURN_ERR_IF(self == NULL, false);

	if (atomic_load(&self->flushing))
		return false;

	ret = mbuf_raw_video_frame_get_frame_info(frame, &info);
	if (ret < 0)
		return false;

	/* We do not need to use the default filter, but we still need to update
	 * the frame to set the input time */
	venc_default_input_filter_internal_confirm_frame(
		self->base, frame, &info);

	return true;
}


static int create(struct venc_encoder *base)
{
	int ret = 0;
	struct venc_png *self = NULL;
	struct vdef_raw_format *fmt;
	struct mbuf_raw_video_frame_queue_args queue_args = {
		.filter = input_filter,
	};

	VENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	/* Check the configuration */
	if (base->config.encoding != VDEF_ENCODING_PNG) {
		ret = -EINVAL;
		VENC_LOG_ERRNO("invalid encoding: %s",
			       -ret,
			       vdef_encoding_to_str(base->config.encoding));
		return ret;
	}
	if (base->config.output.preferred_format !=
	    VDEF_CODED_DATA_FORMAT_UNKNOWN) {
		ret = -ENOSYS;
		VENC_LOG_ERRNO("unsupported output format: %s",
			       -ret,
			       vdef_coded_data_format_to_str(
				       base->config.output.preferred_format));
		return ret;
	}
	if (base->cbs.frame_output == NULL) {
		ret = -EINVAL;
		VENC_LOG_ERRNO("invalid frame output callback", -ret);
		return ret;
	}

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	self->base = base;
	base->derived = self;
	self->output_format = (struct vdef_coded_format){
		.encoding = VDEF_ENCODING_PNG,
		.data_format = VDEF_CODED_DATA_FORMAT_UNKNOWN,
	};

	queue_args.filter_userdata = self;

	VENC_LOGI("png implementation");

	/* Initialize the mailbox for inter-thread messages  */
	self->mbox = mbox_new(1);
	if (self->mbox == NULL) {
		ret = -ENOMEM;
		VENC_LOG_ERRNO("mbox_new", -ret);
		goto error;
	}
	ret = pomp_loop_add(base->loop,
			    mbox_get_read_fd(self->mbox),
			    POMP_FD_EVENT_IN,
			    &mbox_cb,
			    self);
	if (ret < 0) {
		VENC_LOG_ERRNO("pomp_loop_add", -ret);
		goto error;
	}

	self->in_picture.width = base->config.input.info.resolution.width;
	self->in_picture.height = base->config.input.info.resolution.height;
	self->in_picture.bit_depth = base->config.input.info.bit_depth;

	fmt = &base->config.input.format;
	if (vdef_raw_format_cmp(fmt, &vdef_gray) ||
	    vdef_raw_format_cmp(fmt, &vdef_gray16) ||
	    vdef_raw_format_cmp(fmt, &vdef_raw8) ||
	    vdef_raw_format_cmp(fmt, &vdef_raw16) ||
	    vdef_raw_format_cmp(fmt, &vdef_raw16_be)) {
		self->in_picture.color_type = PNG_COLOR_TYPE_GRAY;
		self->in_picture.planes = 1;
	} else if (vdef_raw_format_cmp(fmt, &vdef_rgb)) {
		self->in_picture.color_type = PNG_COLOR_TYPE_RGB;
		self->in_picture.planes = 1;
	}

	self->in_picture.compression_level = base->config.png.compression_level;
	self->in_picture.data_little_endian =
		base->config.input.format.data_little_endian;

	/* Create the input buffers queue */
	ret = mbuf_raw_video_frame_queue_new_with_args(&queue_args,
						       &self->in_queue);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_new_with_args:input",
			       -ret);
		goto error;
	}

	/* Create the encoder output buffers queue */
	ret = mbuf_coded_video_frame_queue_new(&self->enc_out_queue);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_queue_new:enc_out",
			       -ret);
		goto error;
	}

	ret = mbuf_coded_video_frame_queue_get_event(self->enc_out_queue,
						     &self->enc_out_queue_evt);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_queue_get_event", -ret);
		goto error;
	}

	ret = pomp_evt_attach_to_loop(self->enc_out_queue_evt,
				      base->loop,
				      &enc_out_queue_evt_cb,
				      self);
	if (ret < 0) {
		VENC_LOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto error;
	}

	/* Create the encoding thread */
	ret = pthread_create(&self->thread, NULL, encoder_thread, (void *)self);
	if (ret != 0) {
		ret = -ret;
		VENC_LOG_ERRNO("pthread_create", -ret);
		goto error;
	}

	self->thread_launched = true;

	return 0;

error:
	/* Cleanup on error */
	destroy(base);
	base->derived = NULL;
	return ret;
}


static struct mbuf_pool *get_input_buffer_pool(struct venc_encoder *base)
{
	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	/* No input buffer pool allocated: use the application's */
	return NULL;
}


static struct mbuf_raw_video_frame_queue *
get_input_buffer_queue(struct venc_encoder *base)
{
	struct venc_png *self;

	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	self = (struct venc_png *)base->derived;

	return self->in_queue;
}


static int get_dyn_config(struct venc_encoder *base,
			  struct venc_dyn_config *config)
{
	return -ENOSYS;
}


static int set_dyn_config(struct venc_encoder *base,
			  const struct venc_dyn_config *config)
{
	return -ENOSYS;
}


const struct venc_ops venc_png_ops = {
	.get_supported_encodings = get_supported_encodings,
	.get_supported_input_formats = get_supported_input_formats,
	.create = create,
	.flush = flush,
	.stop = stop,
	.destroy = destroy,
	.get_input_buffer_pool = get_input_buffer_pool,
	.get_input_buffer_queue = get_input_buffer_queue,
	.get_dyn_config = get_dyn_config,
	.set_dyn_config = set_dyn_config,
};
