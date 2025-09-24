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

#define ULOG_TAG venc_mediacodec
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "venc_mediacodec_priv.h"


#define INPUT_DEQUEUE_TIMEOUT_US 8000
#define OUTPUT_DEQUEUE_TIMEOUT_US 8000


#define MAX_SUPPORTED_ENCODINGS 2
#define NB_SUPPORTED_FORMATS 3
static struct vdef_raw_format supported_formats[NB_SUPPORTED_FORMATS];
static enum vdef_encoding supported_encodings[MAX_SUPPORTED_ENCODINGS];
static int nb_supported_encodings;
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;
static void initialize_supported_formats(void)
{
	supported_formats[0] = vdef_nv12;
	supported_formats[1] = vdef_i420;
	supported_formats[2] = vdef_opaque;

	enum vdef_encoding enc[] = {
		VDEF_ENCODING_H264,
		VDEF_ENCODING_H265,
		VDEF_ENCODING_UNKNOWN,
	};

	size_t j = 0;

	for (size_t i = 0; enc[i] != VDEF_ENCODING_UNKNOWN; i++) {
		const char *m = vdef_get_encoding_mime_type(enc[i]);
		AMediaCodec *mdec = AMediaCodec_createEncoderByType(m);
		if (mdec != NULL) {
			supported_encodings[j] = enc[i];
			j++;
			media_status_t status = AMediaCodec_delete(mdec);
			if (status != AMEDIA_OK) {
				ULOGE("AMediaCodec_delete (%s:%d)",
				      media_status_to_str(status),
				      status);
			}
		}
	}

	nb_supported_encodings = j;
}


static int get_supported_encodings(const enum vdef_encoding **encodings)
{
	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);
	*encodings = supported_encodings;
	return nb_supported_encodings;
}


static int get_supported_input_formats(enum vdef_encoding encoding,
				       const struct vdef_raw_format **formats)
{
	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);
	*formats = supported_formats;
	return NB_SUPPORTED_FORMATS;
}


static int copy_implem_cfg(const struct venc_config_impl *impl_cfg,
			   struct venc_config_impl **ret_obj)
{
	struct venc_config_mediacodec *specific =
		(struct venc_config_mediacodec *)impl_cfg;
	struct venc_config_mediacodec *copy = NULL;
	ULOG_ERRNO_RETURN_ERR_IF(specific == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	*ret_obj = NULL;

	copy = calloc(1, sizeof(*copy));
	ULOG_ERRNO_RETURN_ERR_IF(copy == NULL, ENOMEM);

	/* Deep copy */
	*copy = *specific;

	/* Note: nothing more to do */

	*ret_obj = (struct venc_config_impl *)copy;

	return 0;
}


static int free_implem_cfg(struct venc_config_impl *impl_cfg)
{
	struct venc_config_mediacodec *specific =
		(struct venc_config_mediacodec *)impl_cfg;
	ULOG_ERRNO_RETURN_ERR_IF(specific == NULL, EINVAL);

	/* Note: nothing more to do */

	free((void *)specific);

	return 0;
}


static void call_flush_done(void *userdata)
{
	struct venc_mediacodec *self = userdata;

	VENC_LOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	venc_call_flush_cb(self->base);
}


static void call_stop_done(void *userdata)
{
	struct venc_mediacodec *self = userdata;

	VENC_LOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	venc_call_stop_cb(self->base);
}


static int
copy_buf(uint8_t **dst, size_t *dst_size, const uint8_t *src, size_t src_size)
{
	uint8_t *t = realloc(*dst, src_size);
	if (t == NULL) {
		ULOG_ERRNO("realloc", -ENOMEM);
		return -ENOMEM;
	}
	*dst_size = src_size;
	*dst = t;
	memcpy(*dst, src, src_size);

	return 0;
}


static void copy_nalus(uint8_t ***dsts,
		       size_t **dst_sizes,
		       size_t dst_count,
		       const uint8_t *src,
		       size_t src_size)
{
	uint8_t start_code[] = {0, 0, 0, 1};

	for (size_t i = 0; i < dst_count; i++) {
		src += sizeof(start_code);
		src_size -= sizeof(start_code);

		size_t j = 0;
		while (j + sizeof(start_code) < src_size &&
		       (memcmp(src + j, start_code, sizeof(start_code)) != 0)) {
			j += 1;
		}

		if (j + sizeof(start_code) >= src_size)
			j = src_size;

		int r = copy_buf(dsts[i], dst_sizes[i], src, j);
		if (r < 0) {
			/* Unfortunate situation: we hit a memory allocation
			 * failure but all the user is going to get is
			 * `venc_get_h26x_ps` returning `-EAGAIN` forever. */
			for (size_t i = 0; i < dst_count; i++) {
				free(dsts[i]);
				dsts[i] = NULL;
			}
			return;
		}

		src += j;
		src_size -= j;
	}
}


static void store_ps(struct venc_mediacodec *self,
		     size_t buf_idx,
		     AMediaCodecBufferInfo info)
{
	int err;
	size_t size = 0;
	/* We’re not going to use the size returned by `getOutputBuffer`, but
	 * it might not support being passed `NULL`. */
	size_t discarded_size;
	const uint8_t *p =
		AMediaCodec_getOutputBuffer(self->mc, buf_idx, &discarded_size);
	if (p == NULL) {
		VENC_LOGE("AMediaCodec_getOutputBuffer");
		goto out;
	}

	p += info.offset;
	size = info.size;

	pthread_mutex_lock(&self->pull.mutex);
	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264: {
		/* Copy PS */
		uint8_t **a[] = {
			&self->base->h264.sps,
			&self->base->h264.pps,
		};
		size_t *lens[] = {
			&self->base->h264.sps_size,
			&self->base->h264.pps_size,
		};
		copy_nalus(a, lens, 2, p, size);
		/* Initialize the H.264 writer */
		err = venc_h264_writer_new(self->base->h264.sps,
					   self->base->h264.sps_size,
					   self->base->h264.pps,
					   self->base->h264.pps_size,
					   &self->base->h264.ctx);
		if (err < 0) {
			VENC_LOG_ERRNO("venc_h264_writer_new", -err);
			goto out;
		}
		err = venc_h264_patch_ps(self->base);
		if (err < 0) {
			VENC_LOG_ERRNO("venc_h264_patch_ps", -err);
			goto out;
		}
		break;
	}
	case VDEF_ENCODING_H265: {
		/* Copy PS */
		uint8_t **a[] = {
			&self->base->h265.vps,
			&self->base->h265.sps,
			&self->base->h265.pps,
		};
		size_t *lens[] = {
			&self->base->h265.vps_size,
			&self->base->h265.sps_size,
			&self->base->h265.pps_size,
		};
		copy_nalus(a, lens, 3, p, size);
		/* Initialize the H.265 writer */
		err = venc_h265_writer_new(self->base->h265.vps,
					   self->base->h265.vps_size,
					   self->base->h265.sps,
					   self->base->h265.sps_size,
					   self->base->h265.pps,
					   self->base->h265.pps_size,
					   &self->base->h265.ctx);
		if (err < 0) {
			VENC_LOG_ERRNO("venc_h265_writer_new", -err);
			goto out;
		}
		err = venc_h265_patch_ps(self->base);
		if (err < 0) {
			VENC_LOG_ERRNO("venc_h265_patch_ps", -err);
			goto out;
		}
		break;
	}
	default:
		break;
	}
	pthread_mutex_unlock(&self->pull.mutex);

out:
	err = pomp_evt_signal(self->out_evt);
	if (err < 0)
		VENC_LOG_ERRNO("pomp_evt_signal", -err);

	media_status_t status =
		AMediaCodec_releaseOutputBuffer(self->mc, buf_idx, false);
	if (status != AMEDIA_OK) {
		VENC_LOGE("AMediaCodec_releaseOutputBuffer (%s:%d)",
			  media_status_to_str(status),
			  status);
	}
}


static void on_output_event(struct pomp_evt *evt, void *userdata)
{
	int err;
	struct venc_mediacodec *self = userdata;

	switch (atomic_load(&self->state)) {
	case RUNNING:
		do {
			struct mbuf_coded_video_frame *out_frame;
			err = mbuf_coded_video_frame_queue_pop(self->out_queue,
							       &out_frame);
			if (err < 0) {
				if (err != -EAGAIN)
					VENC_LOG_ERRNO(
						"mbuf_coded_video_"
						"frame_queue_pop",
						-err);
				break;
			}
			struct vdef_coded_frame out_info = {};
			err = mbuf_coded_video_frame_get_frame_info(out_frame,
								    &out_info);
			if (err < 0)
				VENC_LOG_ERRNO(
					"mbuf_coded_video_frame_get_frame_info",
					-err);
			if (atomic_load(&self->state) != WAITING_FOR_FLUSH) {
				venc_call_frame_output_cb(
					self->base, 0, out_frame);
			} else {
				VENC_LOGD("discarding frame %d",
					  out_info.info.index);
			}
			err = mbuf_coded_video_frame_unref(out_frame);
			if (err < 0) {
				VENC_LOG_ERRNO("mbuf_coded_video_frame_unref",
					       -err);
			}
		} while (err == 0);

		pthread_mutex_lock(&self->pull.mutex);
		bool eos_flag = self->eos_flag;
		self->eos_flag = false;
		pthread_mutex_unlock(&self->pull.mutex);

		if (eos_flag) {
			media_status_t status = AMediaCodec_flush(self->mc);
			if (status != AMEDIA_OK) {
				VENC_LOGE("AMediaCodec_flush (%s:%d)",
					  media_status_to_str(status),
					  status);
			}
			err = pomp_loop_idle_add_with_cookie(
				self->base->loop, call_flush_done, self, self);
			if (err < 0) {
				VENC_LOG_ERRNO("pomp_loop_idle_add_with_cookie",
					       -err);
			}
		}
		break;
	case WAITING_FOR_FLUSH:
		pthread_mutex_lock(&self->push.mutex);
		pthread_mutex_lock(&self->pull.mutex);
		bool flush_complete =
			!self->push.flush_flag && !self->pull.flush_flag;
		pthread_mutex_unlock(&self->pull.mutex);
		pthread_mutex_unlock(&self->push.mutex);

		if (flush_complete) {
			err = mbuf_raw_video_frame_queue_flush(self->in_queue);
			if (err < 0) {
				VENC_LOG_ERRNO(
					"mbuf_raw_video_frame_queue_flush(in)",
					-err);
			}
			err = mbuf_raw_video_frame_queue_flush(
				self->meta_queue);
			if (err < 0) {
				VENC_LOG_ERRNO(
					"mbuf_raw_video_frame"
					"_queue_flush(meta)",
					-err);
			}
			err = mbuf_coded_video_frame_queue_flush(
				self->out_queue);
			if (err < 0) {
				VENC_LOG_ERRNO(
					"mbuf_coded_video_frame"
					"_queue_flush(out)",
					-err);
			}

			media_status_t status = AMediaCodec_flush(self->mc);
			if (status != AMEDIA_OK) {
				VENC_LOGE("AMediaCodec_flush (%s:%d)",
					  media_status_to_str(status),
					  status);
			}

			atomic_store(&self->state, RUNNING);

			err = pomp_loop_idle_add_with_cookie(
				self->base->loop, call_flush_done, self, self);
			if (err < 0) {
				VENC_LOG_ERRNO("pomp_loop_idle_add_with_cookie",
					       -err);
			}
		}
		break;
	case WAITING_FOR_STOP:
		pthread_mutex_lock(&self->push.mutex);
		pthread_mutex_lock(&self->pull.mutex);
		bool stop_complete =
			!self->push.stop_flag && !self->pull.stop_flag;
		pthread_mutex_unlock(&self->pull.mutex);
		pthread_mutex_unlock(&self->push.mutex);

		if (stop_complete) {
			media_status_t status = AMediaCodec_stop(self->mc);
			if (status != AMEDIA_OK) {
				VENC_LOGE("AMediaCodec_stop (%s:%d)",
					  media_status_to_str(status),
					  status);
			}

			err = pomp_loop_idle_add_with_cookie(
				self->base->loop, call_stop_done, self, self);
			if (err < 0) {
				VENC_LOG_ERRNO("pomp_loop_idle_add_with_cookie",
					       -err);
			}
		}
		break;
	default:
		break;
	}
}


static int flush(struct venc_encoder *base, int discard)
{
	struct venc_mediacodec *self = base->derived;

	if (discard) {
		int state = atomic_exchange(&self->state, WAITING_FOR_FLUSH);
		if (state == WAITING_FOR_FLUSH)
			return 0;

		pthread_mutex_lock(&self->push.mutex);
		self->push.flush_flag = true;
		self->push.cond_signalled = true;
		pthread_cond_signal(&self->push.cond);
		pthread_mutex_unlock(&self->push.mutex);

		pthread_mutex_lock(&self->pull.mutex);
		self->pull.flush_flag = true;
		self->pull.cond_signalled = true;
		pthread_cond_signal(&self->pull.cond);
		pthread_mutex_unlock(&self->pull.mutex);
	} else {
		pthread_mutex_lock(&self->push.mutex);
		self->push.eos_flag = true;
		self->push.cond_signalled = true;
		pthread_cond_signal(&self->push.cond);
		pthread_mutex_unlock(&self->push.mutex);
	}

	return 0;
}


static int stop(struct venc_encoder *base)
{
	struct venc_mediacodec *self = base->derived;

	int state = atomic_exchange(&self->state, WAITING_FOR_STOP);
	if (state == WAITING_FOR_STOP)
		return 0;

	pthread_mutex_lock(&self->push.mutex);
	self->push.stop_flag = true;
	self->push.cond_signalled = true;
	pthread_cond_signal(&self->push.cond);
	pthread_mutex_unlock(&self->push.mutex);

	pthread_mutex_lock(&self->pull.mutex);
	self->pull.stop_flag = true;
	self->pull.cond_signalled = true;
	pthread_cond_signal(&self->pull.cond);
	pthread_mutex_unlock(&self->pull.mutex);

	return 0;
}


static int destroy(struct venc_encoder *base)
{
	struct venc_mediacodec *self = base->derived;
	int err;

	if (self == NULL)
		return 0;

	if (self->libmediandk_handle != NULL)
		dlclose(self->libmediandk_handle);

	if (self->mc != NULL) {
		media_status_t status = AMediaCodec_stop(self->mc);
		if (status != AMEDIA_OK) {
			VENC_LOGE("AMediaCodec_stop (%s:%d)",
				  media_status_to_str(status),
				  status);
		}
	}

	if (self->mem != NULL) {
		err = mbuf_mem_unref(self->mem);
		if (err != 0)
			VENC_LOG_ERRNO("mbuf_mem_unref", -err);
	}

	if (self->format != NULL)
		AMediaFormat_delete(self->format);

	if (self->pull.thread_created) {
		pthread_mutex_lock(&self->pull.mutex);
		self->pull.stop_flag = true;
		self->pull.cond_signalled = true;
		pthread_cond_signal(&self->pull.cond);
		pthread_mutex_unlock(&self->pull.mutex);

		pthread_join(self->pull.thread, NULL);
	}

	if (self->push.thread_created) {
		pthread_mutex_lock(&self->push.mutex);
		self->push.stop_flag = true;
		self->push.cond_signalled = true;
		pthread_cond_signal(&self->push.cond);
		pthread_mutex_unlock(&self->push.mutex);

		pthread_join(self->push.thread, NULL);
	}

	/* Remove any leftover idle callbacks */
	err = pomp_loop_idle_remove_by_cookie(self->base->loop, self);
	if (err < 0)
		VENC_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	if (self->out_evt != NULL) {
		if (pomp_evt_is_attached(self->out_evt, base->loop)) {
			err = pomp_evt_detach_from_loop(self->out_evt,
							base->loop);
			if (err < 0)
				VENC_LOG_ERRNO("pomp_evt_detach_from_loop",
					       -err);
		}
		err = pomp_evt_destroy(self->out_evt);
		if (err < 0)
			VENC_LOG_ERRNO("pomp_evt_destroy", -err);
	}

	if (self->out_queue != NULL) {
		err = mbuf_coded_video_frame_queue_destroy(self->out_queue);
		if (err < 0)
			VENC_LOG_ERRNO(
				"mbuf_coded_video_frame_queue_destroy(out)",
				-err);
	}

	if (self->meta_queue != NULL) {
		err = mbuf_raw_video_frame_queue_destroy(self->meta_queue);
		if (err < 0)
			VENC_LOG_ERRNO(
				"mbuf_raw_video_frame_queue_destroy(meta)",
				-err);
	}

	if (self->in_queue != NULL) {
		struct pomp_evt *evt;
		err = mbuf_raw_video_frame_queue_get_event(self->in_queue,
							   &evt);
		if (err < 0) {
			VENC_LOG_ERRNO(
				"mbuf_raw_video_frame_queue_get_event(in)",
				-err);
		}
		if (pomp_evt_is_attached(evt, base->loop)) {
			err = pomp_evt_detach_from_loop(evt, base->loop);
			if (err < 0)
				VENC_LOG_ERRNO("pomp_evt_detach_from_loop",
					       -err);
		}
		err = mbuf_raw_video_frame_queue_destroy(self->in_queue);
		if (err < 0) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_destroy(in)",
				       -err);
		}
	}

	if (self->enc_name != NULL) {
		AMediaCodec_releaseName(self->mc, self->enc_name);
		self->enc_name = NULL;
	}

	if (self->mc != NULL) {
		media_status_t status = AMediaCodec_delete(self->mc);
		if (status != AMEDIA_OK) {
			VENC_LOGE("AMediaCodec_delete (%s:%d)",
				  media_status_to_str(status),
				  status);
		}
	}

	if (self->surface != NULL)
		ANativeWindow_release(self->surface);

	pthread_mutex_destroy(&self->push.mutex);
	pthread_cond_destroy(&self->push.cond);

	pthread_mutex_destroy(&self->pull.mutex);
	pthread_cond_destroy(&self->pull.cond);

	free(self);
	base->derived = NULL;

	return 0;
}


static int push_eos(struct venc_mediacodec *self)
{
	ssize_t buf_idx;
	ssize_t status;
	media_status_t _status;

	if (self->surface != NULL) {
		_status = AMediaCodec_signalEndOfInputStream(self->mc);
		if (_status != AMEDIA_OK) {
			VENC_LOGE("AMediaCodec_signalEndOfInputStream (%s:%d)",
				  media_status_to_str(_status),
				  _status);
			return -EPROTO;
		}
		goto out;
	}

	status = AMediaCodec_dequeueInputBuffer(self->mc,
						INPUT_DEQUEUE_TIMEOUT_US);
	if (status == AMEDIACODEC_INFO_TRY_AGAIN_LATER) {
		return -EAGAIN;
	} else if (status < 0) {
		VENC_LOGE(
			"AMediaCodec"
			"_dequeueInputBuffer (%s:%d)",
			media_status_to_str((media_status_t)status),
			(media_status_t)status);
		return -EPROTO;
	}
	buf_idx = status;

	_status = AMediaCodec_queueInputBuffer(
		self->mc,
		buf_idx,
		0,
		0,
		0,
		AMEDIACODEC_BUFFER_FLAG_END_OF_STREAM);
	if (_status != AMEDIA_OK) {
		VENC_LOGE("AMediaCodec_queueInputBuffer (%s:%d)",
			  media_status_to_str(_status),
			  _status);
		return -EPROTO;
	}

out:
	pthread_mutex_lock(&self->pull.mutex);
	self->pull.eos_flag = true;
	self->pull.cond_signalled = true;
	pthread_cond_signal(&self->pull.cond);
	pthread_mutex_unlock(&self->pull.mutex);

	return 0;
}


static int push_frame(struct venc_mediacodec *self,
		      struct mbuf_raw_video_frame *frame)
{
	int err;
	int res;
	struct vdef_raw_frame frame_info;
	media_status_t status;
	ssize_t _status;
	ssize_t buf_idx = -1;
	size_t buf_size;
	size_t required_len = 0;
	uint8_t *buf_data;
	size_t nplanes = 0;
	const void *planes[VDEF_RAW_MAX_PLANE_COUNT] = {};
	size_t plane_lens[VDEF_RAW_MAX_PLANE_COUNT] = {};
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;
	struct mbuf_raw_video_frame *meta_frame = NULL;

	res = mbuf_raw_video_frame_get_frame_info(frame, &frame_info);
	if (res < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -res);
		goto end;
	}

	ts_us = VDEF_ROUND(frame_info.info.timestamp * 1000000,
			   frame_info.info.timescale);

	if (self->surface != NULL) {
		if (!vdef_raw_format_cmp(&frame_info.format, &vdef_opaque)) {
			res = -EPROTO;
			ULOGE("%s: expecting opaque format "
			      "for encoding from a surface",
			      __func__);
			goto end;
		}
		goto no_buffer;
	}

	nplanes = vdef_get_raw_frame_plane_count(&frame_info.format);
	if (nplanes == 0) {
		res = -EPROTO;
		VENC_LOGE("vdef_get_raw_frame_plane_count");
		goto end;
	}

	for (size_t i = 0; i < nplanes; i++) {
		int res = mbuf_raw_video_frame_get_plane(
			frame, i, planes + i, plane_lens + i);
		if (res < 0) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_get_plane", -res);
			goto end;
		}
	}

	_status = AMediaCodec_dequeueInputBuffer(self->mc,
						 INPUT_DEQUEUE_TIMEOUT_US);
	if (_status == AMEDIACODEC_INFO_TRY_AGAIN_LATER) {
		res = -EAGAIN;
		goto end;
	} else if (_status < 0) {
		VENC_LOGE(
			"AMediaCodec"
			"_dequeueInputBuffer (%s:%d)",
			media_status_to_str((media_status_t)_status),
			(media_status_t)_status);
		res = -EPROTO;
		goto end;
	}
	buf_idx = _status;

	buf_data = AMediaCodec_getInputBuffer(self->mc, buf_idx, &buf_size);
	if (buf_data == NULL) {
		VENC_LOGE("AMediaCodec_getInputBuffer");
		res = -EPROTO;
		goto end;
	}

	for (size_t i = 0; i < nplanes; i++)
		required_len += plane_lens[i];

	if (buf_size < required_len) {
		VENC_LOGE("buf_size (%zu) < required_len (%zu)",
			  buf_size,
			  required_len);
		res = -ENOBUFS;
		goto end;
	}

	/**
	 * TODO: provide AMediaCodec buffers in our own input pool to
	 * avoid copies
	 */
	for (size_t i = 0; i < nplanes; i++) {
		memcpy(buf_data, planes[i], plane_lens[i]);
		buf_data += plane_lens[i];
	}

	status = AMediaCodec_queueInputBuffer(
		self->mc, buf_idx, 0, required_len, ts_us, 0);
	if (status != AMEDIA_OK) {
		VENC_LOGE("AMediaCodec_queueInputBuffer (%s:%d)",
			  media_status_to_str(status),
			  status);
		res = -EPROTO;
		goto end;
	}
	buf_idx = -1;

no_buffer:
	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	err = mbuf_raw_video_frame_add_ancillary_buffer(
		frame, VENC_ANCILLARY_KEY_DEQUEUE_TIME, &ts_us, sizeof(ts_us));
	if (err < 0)
		VENC_LOGW_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
				-err);

	res = venc_copy_raw_frame_as_metadata(frame, self->mem, &meta_frame);
	if (res < 0) {
		VENC_LOG_ERRNO("venc_copy_raw_frame_as_metadata", -res);
		goto end;
	}

	err = mbuf_raw_video_frame_queue_push(self->meta_queue, meta_frame);
	if (err != 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_push", -err);
		goto end;
	}

	/* Don't wake-up pull routine when flushing or stopping */
	if (atomic_load(&self->state) == RUNNING) {
		pthread_mutex_lock(&self->pull.mutex);
		self->pull.cond_signalled = true;
		pthread_cond_signal(&self->pull.cond);
		pthread_mutex_unlock(&self->pull.mutex);
	}

	res = 0;

end:
	if (buf_idx >= 0) {
		status = AMediaCodec_queueInputBuffer(
			self->mc, buf_idx, 0, 0, 0, 0);
		if (status != AMEDIA_OK) {
			VENC_LOGE("AMediaCodec_queueInputBuffer (%s:%d)",
				  media_status_to_str(status),
				  status);
		}
	}
	if (meta_frame) {
		err = mbuf_raw_video_frame_unref(meta_frame);
		if (err < 0)
			VENC_LOG_ERRNO("mbuf_raw_video_frame_unref", -err);
	}
	if (frame) {
		for (size_t i = 0; i < nplanes; i++) {
			if (planes[i] != NULL) {
				err = mbuf_raw_video_frame_release_plane(
					frame, i, planes[i]);
				if (err < 0)
					VENC_LOG_ERRNO(
						"mbuf_raw_video_frame"
						"_release_plane",
						-err);
			}
		}
		err = mbuf_raw_video_frame_unref(frame);
		if (err < 0)
			VENC_LOG_ERRNO("mbuf_raw_video_frame_unref", -err);
	}
	return res;
}


static void on_input_event(struct pomp_evt *evt, void *userdata)
{
	struct venc_mediacodec *self = userdata;

	/* Don't wake-up push routine when flushing or stopping */
	if (atomic_load(&self->state) != RUNNING)
		return;

	pthread_mutex_lock(&self->push.mutex);
	self->push.cond_signalled = true;
	pthread_cond_signal(&self->push.cond);
	pthread_mutex_unlock(&self->push.mutex);
}


static void *push_routine(void *ptr)
{
	int err;
	struct venc_mediacodec *self = ptr;
	unsigned int decimation_counter = 0;

	err = pthread_setname_np(pthread_self(), "venc_mdcdc_push");
	if (err != 0)
		ULOG_ERRNO("pthread_setname_np", err);

	pthread_mutex_lock(&self->push.mutex);
	while (true) {
		self->push.cond_signalled = false;
		if (self->push.stop_flag) {
			self->push.stop_flag = false;
			err = pomp_evt_signal(self->out_evt);
			if (err < 0)
				VENC_LOG_ERRNO("pomp_evt_signal", -err);
			pthread_mutex_unlock(&self->push.mutex);
			break;
		}

		if (self->push.flush_flag) {
			self->push.flush_flag = false;
			err = pomp_evt_signal(self->out_evt);
			if (err < 0)
				VENC_LOG_ERRNO("pomp_evt_signal", -err);
			goto wait;
		}

		struct mbuf_raw_video_frame *frame;
		int res =
			mbuf_raw_video_frame_queue_peek(self->in_queue, &frame);
		if (res == 0) {
			pthread_mutex_unlock(&self->push.mutex);
			if (decimation_counter == 0) {
				/* Note: push_frame unrefs the frame */
				err = push_frame(self, frame);
				if (err == -EAGAIN) {
					/* Retry later */
					pthread_mutex_lock(&self->push.mutex);
					goto wait;
				} else if (err != 0) {
					ULOG_ERRNO("push_frame", -err);
				}
			} else {
				/* Skip frame */
				err = mbuf_raw_video_frame_unref(frame);
				if (err < 0) {
					VENC_LOG_ERRNO(
						"mbuf_raw_video_frame_unref",
						-err);
				}
				err = 0;
			}
			if (err == 0)
				self->base->counters.pushed++;
			/* Pop the frame for real */
			res = mbuf_raw_video_frame_queue_pop(self->in_queue,
							     &frame);
			if (res < 0) {
				VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_pop",
					       -res);
				pthread_mutex_lock(&self->push.mutex);
				continue;
			}
			err = mbuf_raw_video_frame_unref(frame);
			if (err < 0) {
				VENC_LOG_ERRNO("mbuf_raw_video_frame_unref",
					       -err);
			}
			decimation_counter += 1;
			if (decimation_counter >= self->dynconf.decimation)
				decimation_counter = 0;
			pthread_mutex_lock(&self->push.mutex);
			continue;
		} else if (res == -EAGAIN && self->push.eos_flag) {
			pthread_mutex_unlock(&self->push.mutex);
			int err = push_eos(self);
			if (err == 0)
				self->push.eos_flag = false;
			else if (err < 0 && err != -EAGAIN)
				VENC_LOG_ERRNO("push_eos", -err);
			pthread_mutex_lock(&self->push.mutex);
			continue;
		} else if (res != -EAGAIN) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_peek", -res);
		}

		/* clang-format off */
wait:
		/* clang-format on */
		while (!self->push.cond_signalled)
			pthread_cond_wait(&self->push.cond, &self->push.mutex);
	}

	return NULL;
}


static void release_mc_mem(void *data, size_t len, void *userdata)
{
	size_t idx = len;
	AMediaCodec *mc = userdata;

	media_status_t status = AMediaCodec_releaseOutputBuffer(mc, idx, false);
	if (status != AMEDIA_OK) {
		ULOGE("AMediaCodec_releaseOutputBuffer (%s:%d)",
		      media_status_to_str(status),
		      status);
	}
}


static void frame_release(struct mbuf_coded_video_frame *frame, void *userdata)
{
	struct venc_mediacodec *self = userdata;

	venc_call_pre_release_cb(self->base, frame);
}


static int pull_frame(struct venc_mediacodec *self,
		      struct mbuf_raw_video_frame *meta_frame)
{
	int res, err;
	AMediaCodecBufferInfo info;
	size_t buffer_index = SIZE_MAX;
	struct mbuf_mem *out_mem = NULL;
	struct mbuf_coded_video_frame *frame = NULL;
	struct mbuf_coded_video_frame *out_frame = NULL;
	uint8_t *out_data = NULL;
	size_t out_size;
	struct mbuf_mem *mem = NULL;
	struct vdef_raw_frame meta_info = {};
	struct vmeta_frame *metadata = NULL;

	struct mbuf_coded_video_frame_cbs frame_cbs = {
		.pre_release = frame_release,
		.pre_release_userdata = (void *)self,
	};

	while (true) {
		ssize_t status = AMediaCodec_dequeueOutputBuffer(
			self->mc, &info, OUTPUT_DEQUEUE_TIMEOUT_US);
		switch (status) {
		case AMEDIACODEC_INFO_TRY_AGAIN_LATER:
			res = -EAGAIN;
			goto end;
		case AMEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED:
			/* MediaCodec sends both `FORMAT_CHANGED` events and
			 * codec-config buffers. We use only the latter. */
			continue;
		default:
			if (status < 0) {
				VENC_LOGE(
					"AMediaCodec"
					"_dequeueOutputBuffer (%s:%d)",
					media_status_to_str(
						(media_status_t)status),
					(media_status_t)status);
				res = -EPROTO;
				goto end;
			}
			buffer_index = status;
			/* Buffers marked with
			 * `AMEDIACODEC_BUFFER_FLAG_CODEC_CONFIG` contain only
			 * parameter sets. */
			if (info.flags & AMEDIACODEC_BUFFER_FLAG_CODEC_CONFIG) {
				/* Note: store_ps releases the buffer */
				store_ps(self, buffer_index, info);
				buffer_index = SIZE_MAX;
				continue;
			}
			goto end_loop;
		}
	}

end_loop:
	out_data =
		AMediaCodec_getOutputBuffer(self->mc, buffer_index, &out_size) +
		info.offset;
	if (out_data == NULL) {
		res = -EPROTO;
		VENC_LOGE("AMediaCodec_getOutputBuffer");
		goto end;
	}

	self->base->counters.pulled++;

	/* buffer index is passed as argument instead of out_data buffer's
	 * length */
	res = mbuf_mem_generic_wrap(
		out_data, buffer_index, release_mc_mem, self->mc, &mem);
	if (res < 0) {
		VENC_LOG_ERRNO("mbuf_mem_generic_wrap", -res);
		goto end;
	}

	err = mbuf_raw_video_frame_get_frame_info(meta_frame, &meta_info);
	if (err < 0)
		VENC_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -err);

	struct vdef_coded_frame out_info = {
		.info = meta_info.info,
		.type = VDEF_CODED_FRAME_TYPE_UNKNOWN,
		.layer = 0,
	};
	switch (self->base->config.encoding) {
	default:
	case VDEF_ENCODING_H264:
		switch (self->base->config.output.preferred_format) {
		case VDEF_CODED_DATA_FORMAT_BYTE_STREAM:
		default:
			out_info.format = vdef_h264_byte_stream;
			break;
		case VDEF_CODED_DATA_FORMAT_AVCC:
			out_info.format = vdef_h264_avcc;
			break;
		case VDEF_CODED_DATA_FORMAT_RAW_NALU:
			out_info.format = vdef_h264_raw_nalu;
			break;
		}
		break;
	case VDEF_ENCODING_H265:
		switch (self->base->config.output.preferred_format) {
		case VDEF_CODED_DATA_FORMAT_BYTE_STREAM:
		default:
			out_info.format = vdef_h265_byte_stream;
			break;
		case VDEF_CODED_DATA_FORMAT_HVCC:
			out_info.format = vdef_h265_hvcc;
			break;
		case VDEF_CODED_DATA_FORMAT_RAW_NALU:
			out_info.format = vdef_h265_raw_nalu;
			break;
		}
		break;
	}

	uint8_t start_code[] = {0, 0, 0, 1};
	uint8_t *p = out_data;
	size_t n = info.size;
	while (true) {
		uint8_t *q = p + 1;
		size_t m = n - 1;

		bool end;
		while (true) {
			if (m < sizeof(start_code)) {
				q += m;
				m = 0;
				end = true;
				break;
			}

			if (memcmp(q, start_code, sizeof(start_code)) == 0) {
				end = false;
				break;
			}

			q += 1;
			m -= 1;
		}

		switch (self->base->config.encoding) {
		case VDEF_ENCODING_H264:
			if ((*(p + 4) & 0x1F) == H264_NALU_TYPE_SLICE_IDR) {
				out_info.type = VDEF_CODED_FRAME_TYPE_IDR;
				break;
			}
			break;
		case VDEF_ENCODING_H265:
			if ((((*(p + 4) & 0x7E) >> 1) ==
			     H265_NALU_TYPE_IDR_W_RADL) ||
			    (((*(p + 4) & 0x7E) >> 1) ==
			     H265_NALU_TYPE_IDR_N_LP)) {
				out_info.type = VDEF_CODED_FRAME_TYPE_IDR;
				break;
			}
			break;
		default:
			break;
		}

		if (end)
			break;

		p = q;
		n = m;
	}

	res = mbuf_coded_video_frame_new(&out_info, &frame);
	if (res < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_few", -res);
		goto end;
	}
	res = mbuf_coded_video_frame_set_callbacks(frame, &frame_cbs);
	if (res < 0)
		VENC_LOG_ERRNO("mbuf_coded_video_frame_set_callbacks", -res);

	res = mbuf_raw_video_frame_foreach_ancillary_data(
		meta_frame,
		mbuf_coded_video_frame_ancillary_data_copier,
		frame);
	if (res < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_foreach_ancillary_data",
			       -res);
		goto end;
	}

	res = mbuf_raw_video_frame_get_metadata(meta_frame, &metadata);
	if (res == 0) {
		res = mbuf_coded_video_frame_set_metadata(frame, metadata);
		if (res < 0) {
			VENC_LOG_ERRNO("mbuf_coded_video_frame_set_metadata",
				       -res);
			goto end;
		}
	} else if (res == -ENOENT) {
		res = 0;
	} else {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_get_metadata", -res);
		goto end;
	}

	if (self->base->config.encoding == VDEF_ENCODING_H264) {
		/* Add generated NAL units */
		res = venc_h264_generate_nalus(self->base, frame, &out_info);
		if (res < 0) {
			VENC_LOG_ERRNO("venc_h264_generate_nalus", -res);
			goto end;
		}
	} else if (self->base->config.encoding == VDEF_ENCODING_H265) {
		/* Add generated NAL units */
		res = venc_h265_generate_nalus(self->base, frame, &out_info);
		if (res < 0) {
			VENC_LOG_ERRNO("venc_h265_generate_nalus", -res);
			goto end;
		}
	}

	p = out_data;
	n = info.size;
	while (true) {
		uint8_t *q = p + 1;
		size_t m = n - 1;
		uint32_t nalu_size;

		bool end;
		while (true) {
			if (m < sizeof(start_code)) {
				q += m;
				m = 0;
				end = true;
				break;
			}

			if (memcmp(q, start_code, sizeof(start_code)) == 0) {
				end = false;
				break;
			}

			q += 1;
			m -= 1;
		}

		switch (self->base->config.output.preferred_format) {
		case VDEF_CODED_DATA_FORMAT_BYTE_STREAM:
		default:
			break;
		case VDEF_CODED_DATA_FORMAT_AVCC:
			/* Note: VDEF_CODED_DATA_FORMAT_HVCC is handled here */
			nalu_size = htonl((q - p) - 4);
			memcpy(p, &nalu_size, 4);
			break;
		case VDEF_CODED_DATA_FORMAT_RAW_NALU:
			p += 4;
			break;
		}

		struct vdef_nalu nalu = {
			.size = q - p,
			.importance = 0,
		};
		switch (self->base->config.encoding) {
		case VDEF_ENCODING_H264:
			nalu.h264.type = (*(p + 4) & 0x1F);
			if ((nalu.h264.type == H264_NALU_TYPE_AUD) ||
			    (nalu.h264.type == H264_NALU_TYPE_SPS) ||
			    (nalu.h264.type == H264_NALU_TYPE_PPS) ||
			    (nalu.h264.type == H264_NALU_TYPE_SEI)) {
				p = q;
				n = m;
				continue;
			}
			break;
		case VDEF_ENCODING_H265:
			nalu.h265.type = ((*(p + 4) & 0x7E) >> 1);
			if ((nalu.h265.type == H265_NALU_TYPE_AUD_NUT) ||
			    (nalu.h265.type == H265_NALU_TYPE_VPS_NUT) ||
			    (nalu.h265.type == H265_NALU_TYPE_SPS_NUT) ||
			    (nalu.h265.type == H265_NALU_TYPE_PPS_NUT) ||
			    (nalu.h265.type == H265_NALU_TYPE_PREFIX_SEI_NUT)) {
				p = q;
				n = m;
				continue;
			}
			break;
		default:
			break;
		}

		res = mbuf_coded_video_frame_add_nalu(
			frame, mem, p - out_data, &nalu);
		if (res < 0) {
			VENC_LOG_ERRNO("mbuf_coded_video_frame_add_nalu", -res);
			goto end;
		}

		if (end)
			break;

		p = q;
		n = m;
	}

	struct timespec cur_ts;
	time_get_monotonic(&cur_ts);
	uint64_t ts_us;
	time_timespec_to_us(&cur_ts, &ts_us);

	err = mbuf_coded_video_frame_add_ancillary_buffer(
		frame, VENC_ANCILLARY_KEY_OUTPUT_TIME, &ts_us, sizeof(ts_us));
	if (err < 0) {
		VENC_LOGW_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-err);
	}

	res = mbuf_coded_video_frame_finalize(frame);
	if (res < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_finalize", -res);
		goto end;
	}

	ssize_t capacity = 0;
	capacity = mbuf_coded_video_frame_get_packed_size(frame);
	if (capacity < 0) {
		res = -EPROTO;
		VENC_LOG_ERRNO("mbuf_coded_video_frame_get_packed_size", -res);
		goto end;
	}

	res = mbuf_mem_generic_new(capacity, &out_mem);
	if (res < 0) {
		VENC_LOG_ERRNO("mbuf_mem_generic_new", -res);
		goto end;
	}

	res = mbuf_coded_video_frame_copy(frame, out_mem, &out_frame);
	if (res < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_copy", -res);
		goto end;
	}
	res = mbuf_coded_video_frame_finalize(out_frame);
	if (res < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_finalize", -res);
		goto end;
	}
	res = mbuf_coded_video_frame_queue_push(self->out_queue, out_frame);
	if (res < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_queue_push", -res);
		goto end;
	}

	err = pomp_evt_signal(self->out_evt);
	if (err < 0)
		VENC_LOG_ERRNO("pomp_evt_signal", -err);

	res = 0;

end:
	if (metadata) {
		err = vmeta_frame_unref(metadata);
		if (err < 0)
			VENC_LOG_ERRNO("vmeta_frame_unref", err);
	}
	if (mem) {
		err = mbuf_mem_unref(mem);
		if (err < 0)
			VENC_LOG_ERRNO("mbuf_mem_unref", err);
	} else if (buffer_index != SIZE_MAX) {
		/* Buffer wasn't wrapped yet */
		media_status_t status = AMediaCodec_releaseOutputBuffer(
			self->mc, buffer_index, false);
		if (status != AMEDIA_OK) {
			VENC_LOGE("AMediaCodec_releaseOutputBuffer (%s:%d)",
				  media_status_to_str(status),
				  status);
		}
	}
	if (frame) {
		err = mbuf_coded_video_frame_unref(frame);
		if (err < 0)
			VENC_LOG_ERRNO("mbuf_coded_video_frame_unref", err);
	}
	if (out_mem) {
		err = mbuf_mem_unref(out_mem);
		if (err < 0)
			VENC_LOG_ERRNO("mbuf_mem_unref", err);
	}
	if (out_frame) {
		err = mbuf_coded_video_frame_unref(out_frame);
		if (err < 0)
			VENC_LOG_ERRNO("mbuf_coded_video_frame_unref", err);
	}
	if (meta_frame) {
		err = mbuf_raw_video_frame_unref(meta_frame);
		if (err < 0)
			VENC_LOG_ERRNO("mbuf_raw_video_frame_unref", err);
	}

	return res;
}


static void *pull_routine(void *ptr)
{
	int err;
	struct venc_mediacodec *self = ptr;

	err = pthread_setname_np(pthread_self(), "venc_mdcdc_pull");
	if (err != 0)
		ULOG_ERRNO("pthread_setname_np", err);

	pthread_mutex_lock(&self->pull.mutex);
	while (true) {
		self->pull.cond_signalled = false;
		if (self->pull.stop_flag) {
			self->pull.stop_flag = false;
			err = pomp_evt_signal(self->out_evt);
			if (err < 0)
				VENC_LOG_ERRNO("pomp_evt_signal", -err);
			pthread_mutex_unlock(&self->pull.mutex);
			break;
		}

		if (self->pull.flush_flag) {
			self->pull.flush_flag = false;
			err = pomp_evt_signal(self->out_evt);
			if (err < 0)
				VENC_LOG_ERRNO("pomp_evt_signal", -err);
			goto wait;
		}

		struct mbuf_raw_video_frame *frame;
		int res = mbuf_raw_video_frame_queue_peek(self->meta_queue,
							  &frame);
		if (res == 0) {
			pthread_mutex_unlock(&self->pull.mutex);
			/* Note: pull_frame unrefs the frame */
			int err = pull_frame(self, frame);
			if (err == -EAGAIN) {
				/* Retry later */
				pthread_mutex_lock(&self->pull.mutex);
				goto wait;
			} else if (err < 0) {
				VENC_LOG_ERRNO("pull_frame", -err);
			}
			/* Pop the frame for real */
			res = mbuf_raw_video_frame_queue_pop(self->meta_queue,
							     &frame);
			if (res < 0) {
				VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_pop",
					       -res);
				pthread_mutex_lock(&self->pull.mutex);
				continue;
			}
			err = mbuf_raw_video_frame_unref(frame);
			if (err < 0) {
				VENC_LOG_ERRNO("mbuf_raw_video_frame_unref",
					       -err);
			}
			pthread_mutex_lock(&self->pull.mutex);
			continue;
		} else if (res == -EAGAIN && self->pull.eos_flag) {
			self->pull.eos_flag = false;
			self->eos_flag = true;
			err = pomp_evt_signal(self->out_evt);
			if (err < 0)
				VENC_LOG_ERRNO("pomp_evt_signal", -err);
		} else if (res != -EAGAIN) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_peek", -res);
		}

		/* clang-format off */
wait:
		/* clang-format on */
		while (!self->pull.cond_signalled)
			pthread_cond_wait(&self->pull.cond, &self->pull.mutex);
	}

	return NULL;
}


static int h264_fill_format(AMediaFormat *format, const struct venc_config *c)
{
	enum mc_h264_profile p;
	if (!convert_h264_profile(c->h264.profile, &p)) {
		int e = EINVAL;
		ULOG_ERRNO("%u is not a supported H.264 profile",
			   e,
			   c->h264.profile);
		return -e;
	}
	AMediaFormat_setInt32(format, "profile", p);

	enum mc_h264_level l;
	if (!convert_h264_level(c->h264.level, &l)) {
		int e = EINVAL;
		ULOG_ERRNO(
			"%u is not a supported H.264 level", e, c->h264.level);
		return -e;
	}
	AMediaFormat_setInt32(format, "level", l);

	int mdec_bitrate_mode;
	switch (c->h264.rate_control) {
	case VENC_RATE_CONTROL_CBR:
		mdec_bitrate_mode = 2;
		break;

	case VENC_RATE_CONTROL_VBR:
		mdec_bitrate_mode = 1;
		break;

	case VENC_RATE_CONTROL_CQ:
		AMediaFormat_setInt32(format, "quality", c->h264.qp);
		mdec_bitrate_mode = 0;
		break;
	}
	AMediaFormat_setInt32(format, "bitrate-mode", mdec_bitrate_mode);

	AMediaFormat_setInt32(format, "bitrate", c->h264.target_bitrate);

	AMediaFormat_setFloat(
		format, "i-frame-interval", c->h264.gop_length_sec);

	if (c->h264.intra_refresh != VENC_INTRA_REFRESH_NONE)
		AMediaFormat_setFloat(format,
				      "intra-refresh-period",
				      c->h264.intra_refresh_period);

	return 0;
}


static int h265_fill_format(AMediaFormat *format, const struct venc_config *c)
{
	enum mc_h265_profile p;
	if (!convert_h265_profile(c->h265.profile, &p)) {
		int e = EINVAL;
		ULOG_ERRNO("%u is not a supported H.265 profile",
			   e,
			   c->h265.profile);
		return -e;
	}
	AMediaFormat_setInt32(format, "profile", p);

	enum mc_h265_level l;
	if (!convert_h265_level(c->h265.level, &l)) {
		int e = EINVAL;
		ULOG_ERRNO(
			"%u is not a supported H.265 level", e, c->h265.level);
		return -e;
	}
	AMediaFormat_setInt32(format, "level", l);

	int mdec_bitrate_mode;
	switch (c->h265.rate_control) {
	case VENC_RATE_CONTROL_CBR:
		mdec_bitrate_mode = 2;
		break;

	case VENC_RATE_CONTROL_VBR:
		mdec_bitrate_mode = 1;
		break;

	case VENC_RATE_CONTROL_CQ:
		AMediaFormat_setInt32(format, "quality", c->h265.qp);
		mdec_bitrate_mode = 0;
		break;
	}
	AMediaFormat_setInt32(format, "bitrate-mode", mdec_bitrate_mode);

	AMediaFormat_setInt32(format, "bitrate", c->h265.target_bitrate);
	AMediaFormat_setInt32(format, "max-bitrate", c->h265.max_bitrate);

	AMediaFormat_setFloat(
		format, "i-frame-interval", c->h265.gop_length_sec);

	return 0;
}


static bool input_filter(struct mbuf_raw_video_frame *frame, void *userdata)
{
	int ret;

	struct vdef_raw_frame info;
	struct venc_mediacodec *self = userdata;

	VENC_LOG_ERRNO_RETURN_ERR_IF(self == NULL, false);

	if ((atomic_load(&self->state) != RUNNING) || self->push.eos_flag ||
	    self->pull.eos_flag)
		return false;

	ret = mbuf_raw_video_frame_get_frame_info(frame, &info);
	if (ret != 0)
		return false;

	/* Pass default filters first */
	if (!venc_default_input_filter_internal(self->base,
						frame,
						&info,
						supported_formats,
						NB_SUPPORTED_FORMATS))
		return false;

	const void *tmp;
	size_t tmplen;
	/* Input frame must be packed */
	ret = mbuf_raw_video_frame_get_packed_buffer(frame, &tmp, &tmplen);
	if (ret != 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_get_packed_buffer", -ret);
		return false;
	}
	mbuf_raw_video_frame_release_packed_buffer(frame, tmp);

	venc_default_input_filter_internal_confirm_frame(
		self->base, frame, &info);

	return true;
}


static int create(struct venc_encoder *base)
{
	int ret;
	struct venc_mediacodec *self = NULL;
	struct venc_config_mediacodec *specific;
	struct pomp_evt *evt;
	media_status_t status;

	const struct venc_config *c = &base->config;

	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	const char *mime_type = vdef_get_encoding_mime_type(c->encoding);

	/* Enforce the configuration */
	if (base->config.output.preferred_format ==
	    VDEF_CODED_DATA_FORMAT_UNKNOWN) {
		base->config.output.preferred_format =
			VDEF_CODED_DATA_FORMAT_BYTE_STREAM;
	}
	if ((base->config.output.preferred_format !=
	     VDEF_CODED_DATA_FORMAT_BYTE_STREAM) &&
	    (base->config.output.preferred_format !=
	     VDEF_CODED_DATA_FORMAT_AVCC) &&
	    (base->config.output.preferred_format !=
	     VDEF_CODED_DATA_FORMAT_RAW_NALU)) {
		ULOGE("unsupported output format: %s",
		      vdef_coded_data_format_to_str(
			      base->config.output.preferred_format));
		return -EINVAL;
	}

	specific = (struct venc_config_mediacodec *)venc_config_get_specific(
		&base->config, VENC_ENCODER_IMPLEM_MEDIACODEC);

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	base->derived = self;
	self->base = base;
	atomic_init(&self->state, RUNNING);

	pthread_mutex_init(&self->push.mutex, NULL);
	pthread_cond_init(&self->push.cond, NULL);

	pthread_mutex_init(&self->pull.mutex, NULL);
	pthread_cond_init(&self->pull.cond, NULL);

	/* Create a new mbuf_mem for meta_frame */
	ret = mbuf_mem_generic_new(sizeof(uint32_t), &self->mem);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_mem_generic_new", -ret);
		goto error;
	}

	self->mc = AMediaCodec_createEncoderByType(mime_type);
	if (self->mc == NULL) {
		ret = -ENOSYS;
		VENC_LOG_ERRNO("AMediaCodec_createEncoderByType", -ret);
		goto error;
	}

	status = AMediaCodec_getName(self->mc, &self->enc_name);
	if (status < 0) {
		VENC_LOGE(
			"AMediaCodec"
			"_getName (%s:%d)",
			media_status_to_str((media_status_t)status),
			(media_status_t)status);
		ret = -EPROTO;
		goto error;
	}

	VENC_LOGI("mediacodec implementation (%s)", self->enc_name);

	self->format = AMediaFormat_new();

	/* The keys `AMediaFormat` accepts are detailed at
	 * https://developer.android.com/reference/android/media/MediaFormat.html
	 */

	AMediaFormat_setString(self->format, "mime", mime_type);

	AMediaFormat_setInt32(
		self->format, "width", c->input.info.resolution.width);
	AMediaFormat_setInt32(
		self->format, "height", c->input.info.resolution.height);

	enum color_format color_format;
	if (vdef_raw_format_cmp(&c->input.format, &vdef_opaque) &&
	    specific != NULL && specific->surface != NULL) {
		color_format = COLOR_FORMAT_SURFACE;
	} else if (vdef_raw_format_cmp(&c->input.format, &vdef_i420) &&
		   (specific == NULL || specific->surface == NULL)) {
		color_format = YUV420_PLANAR;
	} else if (vdef_raw_format_cmp(&c->input.format, &vdef_nv12) &&
		   (specific == NULL || specific->surface == NULL)) {
		color_format = YUV420_SEMIPLANAR;
	} else {
		ret = -ENOSYS;
		goto error;
	}
	AMediaFormat_setInt32(self->format, "color-format", color_format);

	float framerate = ((float)c->input.info.framerate.num) /
			  ((float)c->input.info.framerate.den);
	AMediaFormat_setFloat(self->format, "frame-rate", framerate);

	switch (c->encoding) {
	case VDEF_ENCODING_H264:
		self->dynconf = (struct venc_dyn_config){
			.qp = c->h264.qp,
			.target_bitrate = c->h264.target_bitrate,
			.decimation = c->h264.decimation,
		};
		ret = h264_fill_format(self->format, c);
		if (ret < 0)
			goto error;
		break;

	case VDEF_ENCODING_H265:
		self->dynconf = (struct venc_dyn_config){
			.qp = c->h265.qp,
			.target_bitrate = c->h265.target_bitrate,
			.decimation = c->h265.decimation,
		};
		ret = h265_fill_format(self->format, c);
		if (ret < 0)
			goto error;
		break;

	default:
		ret = -ENOSYS;
		goto error;
	}

	status = AMediaCodec_configure(self->mc,
				       self->format,
				       NULL,
				       NULL,
				       AMEDIACODEC_CONFIGURE_FLAG_ENCODE);
	if (status != AMEDIA_OK) {
		ret = -EPROTO;
		VENC_LOGE("AMediaCodec_configure (%s:%d)",
			  media_status_to_str(status),
			  status);
		goto error;
	}

	if (specific != NULL && specific->surface != NULL) {
		self->surface = (ANativeWindow *)specific->surface;
		VENC_LOGI("using input from surface (%p)", self->surface);
		ANativeWindow_acquire(self->surface);
		status = AMediaCodec_setInputSurface(self->mc, self->surface);
		if (status != AMEDIA_OK) {
			ret = -EPROTO;
			VENC_LOGE("AMediaCodec_setInputSurface (%s:%d)",
				  media_status_to_str(status),
				  status);
			goto error;
		}
	}

	struct mbuf_raw_video_frame_queue_args queue_args = {
		.filter = input_filter,
		.filter_userdata = self,
	};
	ret = mbuf_raw_video_frame_queue_new_with_args(&queue_args,
						       &self->in_queue);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_new:input", -ret);
		goto error;
	}

	ret = mbuf_raw_video_frame_queue_new(&self->meta_queue);
	if (ret < 0) {
		VENC_LOG_ERRNO("vbuf_queue_new:meta", -ret);
		goto error;
	}

	ret = mbuf_raw_video_frame_queue_get_event(self->in_queue, &evt);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_get_event(in)",
			       -ret);
		goto error;
	}

	ret = pomp_evt_attach_to_loop(evt, base->loop, on_input_event, self);
	if (ret < 0) {
		VENC_LOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto error;
	}

	ret = mbuf_coded_video_frame_queue_new(&self->out_queue);
	if (ret < 0) {
		VENC_LOG_ERRNO("vbuf_queue_new:output", -ret);
		goto error;
	}

	self->out_evt = pomp_evt_new();
	if (self->out_evt == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	ret = pomp_evt_attach_to_loop(
		self->out_evt, base->loop, on_output_event, self);
	if (ret < 0) {
		VENC_LOG_ERRNO("pomp_event_attach_to_loop", -ret);
		goto error;
	}

	status = AMediaCodec_start(self->mc);
	if (status != AMEDIA_OK) {
		ret = -EPROTO;
		VENC_LOGE("AMediaCodec_start (%s:%d)",
			  media_status_to_str(status),
			  status);
		goto error;
	}

	ret = pthread_create(&self->push.thread, NULL, push_routine, self);
	if (ret != 0) {
		ret = -ret;
		VENC_LOG_ERRNO("pthread_create", -ret);
		goto error;
	}
	self->push.thread_created = true;

	ret = pthread_create(&self->pull.thread, NULL, pull_routine, self);
	if (ret != 0) {
		ret = -ret;
		VENC_LOG_ERRNO("pthread_create", -ret);
		goto error;
	}
	self->pull.thread_created = true;

	/* Failing to load `AMediaCodec_setParameters` is not fatal, but it will
	 * disable dynamic configuration support. */
	self->libmediandk_handle = dlopen("libmediandk.so", RTLD_LAZY);
	if (self->libmediandk_handle != NULL) {
		dlerror();
		self->set_parameters = dlsym(self->libmediandk_handle,
					     "AMediaCodec_setParameters");
		char *error = dlerror();
		if (error != NULL)
			self->set_parameters = NULL;
	}

	return 0;
error:
	destroy(base);
	return ret;
}


static struct mbuf_pool *get_input_buffer_pool(struct venc_encoder *base)
{
	return NULL;
}


static struct mbuf_raw_video_frame_queue *
get_input_buffer_queue(struct venc_encoder *base)
{
	struct venc_mediacodec *self = base->derived;
	return self->in_queue;
}


static int get_dyn_config(struct venc_encoder *base,
			  struct venc_dyn_config *config)
{
	struct venc_mediacodec *self = base->derived;

	*config = self->dynconf;

	return 0;
}


static int set_dyn_config(struct venc_encoder *base,
			  const struct venc_dyn_config *config)
{
	struct venc_mediacodec *self = base->derived;

	if (self->set_parameters == NULL)
		return -ENOSYS;

	if (config->qp != 0)
		return -ENOSYS;

	unsigned int target_bitrate = config->target_bitrate == 0
					      ? self->dynconf.target_bitrate
					      : config->target_bitrate;
	unsigned int decimation = config->decimation == 0
					  ? self->dynconf.decimation
					  : config->decimation;

	if (target_bitrate == self->dynconf.target_bitrate &&
	    decimation == self->dynconf.decimation)
		return 0;

	int32_t bitrate = config->target_bitrate * decimation;
	AMediaFormat_setInt32(self->format, "video-bitrate", bitrate);

	media_status_t status = self->set_parameters(self->mc, self->format);
	if (status != AMEDIA_OK) {
		VENC_LOGE("AMediaCodec_setParameters (%s:%d)",
			  media_status_to_str(status),
			  status);
		return -ENOSYS;
	}
	self->dynconf.target_bitrate = config->target_bitrate;
	self->dynconf.decimation = decimation;

	return 0;
}


const struct venc_ops venc_mediacodec_ops = {
	.get_supported_encodings = get_supported_encodings,
	.get_supported_input_formats = get_supported_input_formats,
	.copy_implem_cfg = copy_implem_cfg,
	.free_implem_cfg = free_implem_cfg,
	.create = create,
	.flush = flush,
	.stop = stop,
	.destroy = destroy,
	.get_input_buffer_pool = get_input_buffer_pool,
	.get_input_buffer_queue = get_input_buffer_queue,
	.get_dyn_config = get_dyn_config,
	.set_dyn_config = set_dyn_config,
};
