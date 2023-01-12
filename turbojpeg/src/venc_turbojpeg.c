/**
 * Copyright (c) 2017 Parrot Drones SAS
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

#define ULOG_TAG venc_turbojpeg
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "venc_turbojpeg_priv.h"


#define NB_SUPPORTED_ENCODINGS 1
#define NB_SUPPORTED_FORMATS 4

static struct vdef_raw_format supported_formats[NB_SUPPORTED_FORMATS];
static enum vdef_encoding supported_encodings[NB_SUPPORTED_ENCODINGS];
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;

static void initialize_supported_formats(void)
{
	supported_formats[0] = vdef_i420;
	supported_formats[1] = vdef_nv12;
	supported_formats[2] = vdef_nv21;
	supported_formats[3] = vdef_gray;
	supported_encodings[0] = VDEF_ENCODING_MJPEG;
}

static void call_flush_done(void *userdata)
{
	struct venc_turbojpeg *self = userdata;

	if (self->base->cbs.flush)
		self->base->cbs.flush(self->base, self->base->userdata);
}

static void call_stop_done(void *userdata)
{
	struct venc_turbojpeg *self = userdata;

	if (self->base->cbs.stop)
		self->base->cbs.stop(self->base, self->base->userdata);
}

static void mbox_cb(int fd, uint32_t revents, void *userdata)
{
	struct venc_turbojpeg *self = userdata;
	int ret, err;
	char message;

	do {
		/* Read from the mailbox */
		ret = mbox_peek(self->mbox, &message);
		if (ret < 0) {
			if (ret != -EAGAIN)
				ULOG_ERRNO("mbox_peek", -ret);
			break;
		}

		switch (message) {
		case VENC_MSG_FLUSH:
			err = pomp_loop_idle_add_with_cookie(
				self->base->loop, call_flush_done, self, self);
			if (err < 0) {
				ULOG_ERRNO("pomp_loop_idle_add_with_cookie",
					   -err);
			}
			break;
		case VENC_MSG_STOP:
			err = pomp_loop_idle_add_with_cookie(
				self->base->loop, call_stop_done, self, self);
			if (err < 0) {
				ULOG_ERRNO("pomp_loop_idle_add_with_cookie",
					   -err);
			}
			break;
		default:
			ULOGE("unknown message: %c", message);
			break;
		}
	} while (ret == 0);
}

static void enc_out_queue_evt_cb(struct pomp_evt *evt, void *userdata)
{
	struct venc_turbojpeg *self = userdata;
	struct mbuf_coded_video_frame *out_frame = NULL;
	int ret;
	do {
		ret = mbuf_coded_video_frame_queue_pop(self->enc_out_queue,
						       &out_frame);
		if (ret == -EAGAIN) {
			return;
		} else if (ret < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_queue_pop:enc_out",
				   -ret);
			return;
		}
		self->base->cbs.frame_output(
			self->base, 0, out_frame, self->base->userdata);
		mbuf_coded_video_frame_unref(out_frame);
	} while (ret == 0);
}

static int fill_frame(struct venc_turbojpeg *self,
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
		ULOG_ERRNO("mbuf_raw_video_frame_foreach_ancillary_data", -ret);
		goto out;
	}

	/* Frame metadata */
	ret = mbuf_raw_video_frame_get_metadata(in_frame, &metadata);
	if (ret == 0) {
		ret = mbuf_coded_video_frame_set_metadata(out_frame, metadata);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_set_metadata", -ret);
			goto out;
		}
	} else if ((ret < 0) && (ret != -ENOENT)) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_metadata", -ret);
		goto out;
	}

	ret = mbuf_mem_generic_new(out_picture_size, &mem);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_mem_generic_new", -ret);
		goto out;
	}

	ret = mbuf_mem_get_data(mem, &nalu_data, &nalu.size);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_mem_get_data", -ret);
		goto out;
	}
	nalu.importance = 0;

	memcpy(nalu_data, out_picture, out_picture_size);
	ret = mbuf_coded_video_frame_add_nalu(out_frame, mem, 0, &nalu);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_mem_get_data", -ret);
		mem = NULL;
		goto out;
	}

out:
	if (mem) {
		ret = mbuf_mem_unref(mem);
		if (ret != 0)
			ULOG_ERRNO("mbuf_mem_unref", -ret);
	}
	if (metadata)
		vmeta_frame_unref(metadata);
	return ret;
}

static int encode_frame(struct venc_turbojpeg *self,
			struct mbuf_raw_video_frame *in_frame)
{
	int ret;
	unsigned char *out_picture = NULL;
	long unsigned int out_picture_size = 0;
	size_t len;
	struct vdef_raw_frame in_info;
	const void *plane_data;
	const uint8_t *in_data;
	struct mbuf_coded_video_frame *out_frame = NULL;
	struct vdef_coded_frame out_info;
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;
	/* only needed when converting from NV12 to I420 */
	uint8_t *u_plane = NULL;
	uint8_t *v_plane = NULL;

	struct mbuf_coded_video_frame_cbs frame_cbs = {
		.pre_release = self->base->cbs.pre_release,
		.pre_release_userdata = self->base->userdata,
	};

	if (!in_frame)
		return 0;

	ret = mbuf_raw_video_frame_get_frame_info(in_frame, &in_info);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		return ret;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	if (!vdef_raw_format_intersect(
		    &in_info.format, supported_formats, NB_SUPPORTED_FORMATS)) {
		ret = -ENOSYS;
		ULOG_ERRNO(
			"unsupported format:"
			" " VDEF_RAW_FORMAT_TO_STR_FMT,
			-ret,
			VDEF_RAW_FORMAT_TO_STR_ARG(&in_info.format));
		return ret;
	}

	if (self->in_picture.planes > SOFTMPEG_MAX_PLANES) {
		ULOGE("invalid number of planes: %d", self->in_picture.planes);
		return -EINVAL;
	}

	for (unsigned int i = 0; i < self->in_picture.planes; i++) {
		ret = mbuf_raw_video_frame_get_plane(
			in_frame, i, &plane_data, &len);
		if (ret == 0) {
			in_data = plane_data;
			self->in_picture.plane[i] = (uint8_t *)in_data;
			self->in_picture.stride[i] = in_info.plane_stride[i];
			mbuf_raw_video_frame_release_plane(
				in_frame, i, plane_data);
		}
		if (ret < 0) {
			/* TODO: don't forget to drop the frame
			 * otherwise it remains in the queue. */
			ULOG_ERRNO(
				"mbuf_raw_video_frame_get_plane(%u)", -ret, i);
			return ret;
		}
	}

	ret = mbuf_raw_video_frame_add_ancillary_buffer(
		in_frame,
		VENC_ANCILLARY_KEY_DEQUEUE_TIME,
		&ts_us,
		sizeof(ts_us));
	if (ret < 0)
		ULOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer", -ret);

	if (vdef_raw_format_cmp(&in_info.format, &vdef_nv12) ||
	    vdef_raw_format_cmp(&in_info.format, &vdef_nv21)) {
		int i = 0;
		int width = self->in_picture.stride[1] / 2;
		int height = self->in_picture.height / 2;
		int up_offset = 0, vp_offset = 1;

		/* we get the plane containing UV */
		const unsigned char *plane = self->in_picture.plane[1];
		self->in_picture.stride[1] = width;
		self->in_picture.stride[2] = width;
		u_plane = (uint8_t *)calloc(width * height, sizeof(*u_plane));
		v_plane = (uint8_t *)calloc(width * height, sizeof(*v_plane));
		if (!u_plane || !v_plane)
			goto out;
		if (vdef_raw_format_cmp(&in_info.format, &vdef_nv21)) {
			up_offset = 1;
			vp_offset = 0;
		}

		for (i = 0; i < (width * height); i++) {
			u_plane[i] = plane[2 * i + up_offset];
			v_plane[i] = plane[2 * i + vp_offset];
		}
		self->in_picture.plane[1] = (const unsigned char *)u_plane;
		self->in_picture.plane[2] = (const unsigned char *)v_plane;
	}

	ret = tjCompressFromYUVPlanes(self->tj_handler,
				      self->in_picture.plane,
				      self->in_picture.width,
				      self->in_picture.stride,
				      self->in_picture.height,
				      self->in_picture.subsamp,
				      &out_picture,
				      &out_picture_size,
				      self->in_picture.quality,
				      self->in_picture.flags);
	if (ret < 0) {
		ULOGE("%s", tjGetErrorStr());
		return ret;
	}

	out_info.info = in_info.info;
	out_info.format = self->output_format;
	out_info.type = VDEF_CODED_FRAME_TYPE_CODED;

	/* Coded frame creation */
	ret = mbuf_coded_video_frame_new(&out_info, &out_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_new", -ret);
		goto out;
	}
	ret = mbuf_coded_video_frame_set_callbacks(out_frame, &frame_cbs);
	if (ret < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_set_callbacks", -ret);

	ret = fill_frame(
		self, in_frame, out_frame, out_picture, out_picture_size);
	if (ret < 0)
		goto out;

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		out_frame,
		VENC_ANCILLARY_KEY_OUTPUT_TIME,
		&ts_us,
		sizeof(ts_us));
	if (ret < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer", -ret);

	/* Output the frame */
	ret = mbuf_coded_video_frame_finalize(out_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_finalize", -ret);
		goto out;
	}

	ret = mbuf_coded_video_frame_queue_push(self->enc_out_queue, out_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_push", -ret);
		goto out;
	}

out:
	if (out_frame)
		mbuf_coded_video_frame_unref(out_frame);

	if (out_picture)
		tjFree(out_picture);

	if (u_plane)
		free(u_plane);

	if (v_plane)
		free(v_plane);

	return ret;
}

static int complete_flush(struct venc_turbojpeg *self)
{
	int ret;

	if (atomic_load(&self->flush_discard)) {

		/* Flush the input queue */
		ret = mbuf_raw_video_frame_queue_flush(self->in_queue);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_flush:input",
				   -ret);
			return ret;
		}

		/* Flush the encoder output queue */
		ret = mbuf_coded_video_frame_queue_flush(self->enc_out_queue);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_queue_flush:enc_out",
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
		ULOG_ERRNO("mbox_push", -ret);

	return ret;
}

static void check_input_queue(struct venc_turbojpeg *self)
{
	int ret, err = 0;
	struct mbuf_raw_video_frame *in_frame;

	ret = mbuf_raw_video_frame_queue_peek(self->in_queue, &in_frame);
	while (ret >= 0) {
		/* Push the input frame */
		/* Encode the frame */
		ret = encode_frame(self, in_frame);
		if (ret < 0) {
			if (ret != -EAGAIN)
				ULOG_ERRNO("encode_frame", -ret);
			err = -ENOSPC;
		}
		if (in_frame) {
			mbuf_raw_video_frame_unref(in_frame);
			/* Pop the frame for real */
			ret = mbuf_raw_video_frame_queue_pop(self->in_queue,
							     &in_frame);
			if (ret < 0) {
				ULOG_ERRNO("mbuf_raw_video_frame_queue_pop",
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
			ULOG_ERRNO("mbuf_raw_video_frame_queue_peek", -ret);
		if (self->flushing && ret == -EAGAIN) {
			in_frame = NULL;
			if ((atomic_load(&self->flushing)) &&
			    (!atomic_load(&self->flush_discard))) {
				ret = complete_flush(self);
				if (ret < 0)
					ULOG_ERRNO("complete_flush", -ret);
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
				ULOG_ERRNO("complete_flush", -ret);
		}
	}
}

static void input_event_cb(struct pomp_evt *evt, void *userdata)
{
	struct venc_turbojpeg *self = userdata;
	check_input_queue(self);
}


static void *encoder_thread(void *ptr)
{
	int ret, timeout;
	struct venc_turbojpeg *self = ptr;
	struct pomp_loop *loop = NULL;
	struct pomp_evt *in_queue_evt = NULL;
	char message;

	loop = pomp_loop_new();
	if (!loop) {
		ULOG_ERRNO("pomp_loop_new", ENOMEM);
		goto exit;
	}

	ret = mbuf_raw_video_frame_queue_get_event(self->in_queue,
						   &in_queue_evt);
	if (ret != 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_get_event", -ret);
		goto exit;
	}

	ret = pomp_evt_attach_to_loop(in_queue_evt, loop, input_event_cb, self);
	if (ret != 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto exit;
	}

	while ((!atomic_load(&self->should_stop)) ||
	       (atomic_load(&self->flushing))) {
		/* Start flush, discarding all frames */
		if ((atomic_load(&self->flushing)) &&
		    (atomic_load(&self->flush_discard))) {
			ret = complete_flush(self);
			if (ret < 0)
				ULOG_ERRNO("complete_flush", -ret);
			continue;
		}

		/* Wait for an input buffer (without dequeueing it) */
		timeout = ((atomic_load(&self->flushing)) &&
			   (!atomic_load(&self->flush_discard)))
				  ? 0
				  : 5;
		ret = pomp_loop_wait_and_process(loop, timeout);
		if (ret < 0 && ret != -ETIMEDOUT) {
			ULOG_ERRNO("pomp_loop_wait_and_process", -ret);
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
		ULOG_ERRNO("mbox_push", -ret);

exit:
	if (in_queue_evt != NULL) {
		ret = pomp_evt_detach_from_loop(in_queue_evt, loop);
		if (ret != 0)
			ULOG_ERRNO("pomp_evt_detach_from_loop", -ret);
	}
	if (loop != NULL) {
		ret = pomp_loop_destroy(loop);
		if (ret != 0)
			ULOG_ERRNO("pomp_loop_destroy", -ret);
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

static int get_supported_input_formats(const struct vdef_raw_format **formats)
{
	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);
	*formats = supported_formats;
	return NB_SUPPORTED_FORMATS;
}

static int flush(struct venc_encoder *base, int discard_)
{
	struct venc_turbojpeg *self;
	bool discard = discard_;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	atomic_store(&self->flushing, true);
	atomic_store(&self->flush_discard, discard);

	return 0;
}

static int stop(struct venc_encoder *base)
{
	struct venc_turbojpeg *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	/* Stop the encoding thread */
	atomic_store(&self->should_stop, true);
	return 0;
}

static int destroy(struct venc_encoder *base)
{
	int err;
	struct venc_turbojpeg *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;
	if (self == NULL)
		return 0;

	/* Stop and join the encoding thread */
	err = stop(base);
	if (err < 0)
		ULOG_ERRNO("stop", -err);
	if (self->thread_launched) {
		err = pthread_join(self->thread, NULL);
		if (err != 0)
			ULOG_ERRNO("pthread_join", err);
	}

	/* Free the resources */
	if (self->in_queue != NULL) {
		err = mbuf_raw_video_frame_queue_destroy(self->in_queue);
		if (err < 0)
			ULOG_ERRNO("mbuf_raw_video_frame_queue_destroy:input",
				   -err);
	}
	if (self->enc_out_queue_evt != NULL &&
	    pomp_evt_is_attached(self->enc_out_queue_evt, base->loop)) {
		err = pomp_evt_detach_from_loop(self->enc_out_queue_evt,
						base->loop);
		if (err < 0)
			ULOG_ERRNO("pomp_evt_detach_from_loop", -err);
	}
	if (self->enc_out_queue != NULL) {
		err = mbuf_coded_video_frame_queue_destroy(self->enc_out_queue);
		if (err < 0)
			ULOG_ERRNO(
				"mbuf_coded_video_frame_queue_destroy:enc_out",
				-err);
	}
	if (self->mbox) {
		err = pomp_loop_remove(base->loop,
				       mbox_get_read_fd(self->mbox));
		if (err < 0)
			ULOG_ERRNO("pomp_loop_remove", -err);
		mbox_destroy(self->mbox);
	}
	if (self->tj_handler) {
		err = tjDestroy(self->tj_handler);
		if (err < 0)
			ULOGE("%s", tjGetErrorStr());
	}

	err = pomp_loop_idle_remove_by_cookie(base->loop, self);
	if (err < 0)
		ULOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	self->in_picture.plane[0] = NULL;
	self->in_picture.plane[1] = NULL;
	self->in_picture.plane[2] = NULL;

	xfree((void **)&self);
	return 0;
}

static bool input_filter(struct mbuf_raw_video_frame *frame, void *userdata)
{
	int ret;
	struct venc_turbojpeg *self = userdata;
	struct vdef_raw_frame info;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, false);

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
	struct venc_turbojpeg *self = NULL;
	struct vdef_raw_format *fmt;
	struct mbuf_raw_video_frame_queue_args queue_args = {
		.filter = input_filter,
	};

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	/* Check the configuration */
	if (base->config.encoding != VDEF_ENCODING_MJPEG) {
		ret = -EINVAL;
		ULOG_ERRNO("invalid encoding: %s",
			   -ret,
			   vdef_encoding_to_str(base->config.encoding));
		return ret;
	}
	if ((base->config.output.preferred_format !=
	     VDEF_CODED_DATA_FORMAT_UNKNOWN) &&
	    (base->config.output.preferred_format !=
	     VDEF_CODED_DATA_FORMAT_JFIF)) {
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported output format: %s",
			   -ret,
			   vdef_coded_data_format_to_str(
				   base->config.output.preferred_format));
		return ret;
	}
	if (base->cbs.frame_output == NULL) {
		ret = -EINVAL;
		ULOG_ERRNO("invalid frame output callback", -ret);
		return ret;
	}

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	self->base = base;
	base->derived = self;
	self->output_format = (struct vdef_coded_format){
		.encoding = VDEF_ENCODING_MJPEG,
		.data_format = VDEF_CODED_DATA_FORMAT_JFIF,
	};

	queue_args.filter_userdata = self;

	ULOGI("turbojpeg implementation");

	self->tj_handler = tjInitCompress();
	if (!self->tj_handler) {
		ULOGE("%s", tjGetErrorStr());
		goto error;
	}

	/* Initialize the mailbox for inter-thread messages  */
	self->mbox = mbox_new(1);
	if (self->mbox == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("mbox_new", -ret);
		goto error;
	}
	ret = pomp_loop_add(base->loop,
			    mbox_get_read_fd(self->mbox),
			    POMP_FD_EVENT_IN,
			    &mbox_cb,
			    self);
	if (ret < 0) {
		ULOG_ERRNO("pomp_loop_add", -ret);
		goto error;
	}

	self->in_picture.width = base->config.input.info.resolution.width;
	self->in_picture.height = base->config.input.info.resolution.height;

	fmt = &base->config.input.format;
	if (vdef_raw_format_cmp(fmt, &vdef_i420)) {
		self->in_picture.subsamp = TJSAMP_420;
		self->in_picture.planes = 3;
	} else if (vdef_raw_format_cmp(fmt, &vdef_nv12) ||
		   vdef_raw_format_cmp(fmt, &vdef_nv21)) {
		/* Since libjpeg-turbo doesn't support NV12 and NV21,
		 * we need to convert to I420. */
		self->in_picture.subsamp = TJSAMP_420;
		self->in_picture.planes = 2;
	} else if (vdef_raw_format_cmp(fmt, &vdef_gray)) {
		self->in_picture.subsamp = TJSAMP_GRAY;
		self->in_picture.planes = 1;
	}

	self->in_picture.quality = base->config.mjpeg.quality;

	/* Create the input buffers queue */
	ret = mbuf_raw_video_frame_queue_new_with_args(&queue_args,
						       &self->in_queue);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_new_with_args:input",
			   -ret);
		goto error;
	}

	/* Create the encoder output buffers queue */
	ret = mbuf_coded_video_frame_queue_new(&self->enc_out_queue);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_new:enc_out", -ret);
		goto error;
	}

	ret = mbuf_coded_video_frame_queue_get_event(self->enc_out_queue,
						     &self->enc_out_queue_evt);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_get_event", -ret);
		goto error;
	}

	ret = pomp_evt_attach_to_loop(self->enc_out_queue_evt,
				      base->loop,
				      &enc_out_queue_evt_cb,
				      self);
	if (ret < 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto error;
	}

	/* Create the encoding thread */
	ret = pthread_create(&self->thread, NULL, encoder_thread, (void *)self);
	if (ret != 0) {
		ret = -ret;
		ULOG_ERRNO("pthread_create", -ret);
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
	struct venc_turbojpeg *self;

	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	self = (struct venc_turbojpeg *)base->derived;

	return self->in_queue;
}

static int get_dyn_config(struct venc_encoder *base,
			  struct venc_dyn_config *config)
{
	config->target_bitrate = base->config.mjpeg.target_bitrate;
	config->qp = base->config.mjpeg.quality;

	return 0;
}

static int set_dyn_config(struct venc_encoder *base,
			  const struct venc_dyn_config *config)
{
	struct venc_turbojpeg *self;
	self = base->derived;

	if (config->target_bitrate != 0 &&
	    base->config.mjpeg.target_bitrate != config->target_bitrate &&
	    base->config.mjpeg.rate_control != VENC_RATE_CONTROL_CQ) {
		base->config.mjpeg.target_bitrate = config->target_bitrate;
	}

	if (config->qp != base->config.mjpeg.quality)
		base->config.mjpeg.quality = config->qp;

	return 0;
}

const struct venc_ops venc_turbojpeg_ops = {
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
