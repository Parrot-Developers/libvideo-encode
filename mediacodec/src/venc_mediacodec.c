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

#include <dlfcn.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <futils/timetools.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media/NdkMediaCodec.h>
#include <media/NdkMediaFormat.h>
#include <video-encode/venc_core.h>
#include <video-encode/venc_h264.h>
#include <video-encode/venc_h265.h>
#include <video-encode/venc_internal.h>
#include <video-encode/venc_mediacodec.h>

enum state {
	RUNNING,
	WAITING_FOR_FLUSH,
	WAITING_FOR_STOP,
};


struct venc_mediacodec {
	struct venc_encoder *base;

	AMediaCodec *mc;

	struct mbuf_raw_video_frame_queue *in_queue;
	struct mbuf_raw_video_frame_queue *meta_queue;
	struct mbuf_coded_video_frame_queue *out_queue;
	struct pomp_evt *out_evt;

	struct {
		pthread_t thread;
		bool thread_created;
		pthread_mutex_t mutex;
		pthread_cond_t cond;
		bool stop_flag;
		bool flush_flag;
		bool eos_flag;
	} push;

	struct {
		pthread_t thread;
		bool thread_created;
		pthread_mutex_t mutex;
		pthread_cond_t cond;
		bool stop_flag;
		bool flush_flag;
		bool eos_flag;
	} pull;

	struct {
		uint8_t *vps;
		size_t vps_size;
		uint8_t *sps;
		size_t sps_size;
		uint8_t *pps;
		size_t pps_size;
	} pending;

	bool eos_flag;

	struct venc_dyn_config dynconf;

	void *libmediandk_handle;
	media_status_t (*set_parameters)(AMediaCodec *codec,
					 const AMediaFormat *params);

	enum state state;
};


#define MAX_SUPPORTED_ENCODINGS 2
#define NB_SUPPORTED_FORMATS 2
static struct vdef_raw_format supported_formats[NB_SUPPORTED_FORMATS];
static enum vdef_encoding supported_encodings[MAX_SUPPORTED_ENCODINGS];
static int nb_supported_encodings;
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;
static void initialize_supported_formats(void)
{
	supported_formats[0] = vdef_i420;
	supported_formats[1] = vdef_nv12;

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
			AMediaCodec_delete(mdec);
		}
	}

	nb_supported_encodings = j;
}


/* See
 * https://developer.android.com/reference/android/media/MediaCodecInfo.CodecCapabilities.html
 * for reference. */
enum color_format {
	YUV420_PLANAR = 0x00000013,
	YUV420_PACKED_PLANAR = 0x00000014,
	YUV420_SEMIPLANAR = 0x00000015,
	YUV420_PACKED_SEMIPLANAR = 0x00000027,
	TI_YUV420_PACKED_SEMIPLANAR = 0x7F000100,
	QCOM_YUV420_SEMIPLANAR = 0x7FA30C00,
	QCOM_YUV420_PACKED_SEMIPLANAR64X32_TILE2_M8KA = 0x7FA30C03,
	QCOM_YUV420_SEMIPLANAR32_M = 0x7FA30C04,
};


/* See
 * https://developer.android.com/reference/android/media/MediaCodecInfo.CodecProfileLevel
 * for reference. */
enum mc_h264_profile {
	MC_H264_PROFILE_BASELINE = 0x1,
	MC_H264_PROFILE_CONSTRAINED_BASELINE = 0x10000,
	MC_H264_PROFILE_CONSTRAINED_HIGH = 0x80000,
	MC_H264_PROFILE_EXTENDED = 0x4,
	MC_H264_PROFILE_HIGH = 0x8,
	MC_H264_PROFILE_HIGH10 = 0x10,
	MC_H264_PROFILE_HIGH422 = 0x20,
	MC_H264_PROFILE_HIGH444 = 0x40,
	MC_H264_PROFILE_MAIN = 0x2,
};


static bool convert_h264_profile(unsigned int p, enum mc_h264_profile *o)
{
	switch (p) {
	case H264_PROFILE_MAIN:
		*o = MC_H264_PROFILE_MAIN;
		return true;
	default:
		return false;
	}
}


/* See
 * https://developer.android.com/reference/android/media/MediaCodecInfo.CodecProfileLevel
 * for reference. */
enum mc_h264_level {
	MC_H264_LEVEL_1 = 0x1,
	MC_H264_LEVEL_11 = 0x4,
	MC_H264_LEVEL_12 = 0x8,
	MC_H264_LEVEL_13 = 0x10,
	MC_H264_LEVEL_1b = 0x2,
	MC_H264_LEVEL_2 = 0x20,
	MC_H264_LEVEL_21 = 0x40,
	MC_H264_LEVEL_22 = 0x80,
	MC_H264_LEVEL_3 = 0x100,
	MC_H264_LEVEL_31 = 0x200,
	MC_H264_LEVEL_32 = 0x400,
	MC_H264_LEVEL_4 = 0x800,
	MC_H264_LEVEL_41 = 0x1000,
	MC_H264_LEVEL_42 = 0x2000,
	MC_H264_LEVEL_5 = 0x4000,
	MC_H264_LEVEL_51 = 0x8000,
	MC_H264_LEVEL_52 = 0x10000,
	MC_H264_LEVEL_6 = 0x20000,
	MC_H264_LEVEL_61 = 0x40000,
	MC_H264_LEVEL_62 = 0x80000,
};


static bool convert_h264_level(unsigned int l, enum mc_h264_level *o)
{
	switch (l) {
	case 40:
		*o = MC_H264_LEVEL_4;
		return true;
	default:
		return false;
	}
}


/* See
 * https://developer.android.com/reference/android/media/MediaCodecInfo.CodecProfileLevel
 * for reference. */
enum mc_h265_profile {
	MC_H265_PROFILE_MAIN = 1,
	MC_H265_PROFILE_MAIN10 = 2,
	MC_H265_PROFILE_MAIN10_HDR10 = 4096,
	MC_H265_PROFILE_MAIN10_HDR10PLUS = 8192,
	MC_H265_PROFILE_MAIN_STILL = 4,
};


static bool convert_h265_profile(unsigned int p, enum mc_h265_profile *o)
{
	switch (p) {
	case 1:
		*o = MC_H265_PROFILE_MAIN;
		return true;
	default:
		return false;
	}
}


/* See
 * https://developer.android.com/reference/android/media/MediaCodecInfo.CodecProfileLevel
 * for reference. */
enum mc_h265_level {
	MC_H265_HIGH_TIER_LEVEL1 = 0x2,
	MC_H265_HIGH_TIER_LEVEL2 = 0x8,
	MC_H265_HIGH_TIER_LEVEL21 = 0x20,
	MC_H265_HIGH_TIER_LEVEL3 = 0x80,
	MC_H265_HIGH_TIER_LEVEL31 = 0x200,
	MC_H265_HIGH_TIER_LEVEL4 = 0x800,
	MC_H265_HIGH_TIER_LEVEL41 = 0x2000,
	MC_H265_HIGH_TIER_LEVEL5 = 0x8000,
	MC_H265_HIGH_TIER_LEVEL51 = 0x20000,
	MC_H265_HIGH_TIER_LEVEL52 = 0x80000,
	MC_H265_HIGH_TIER_LEVEL6 = 0x200000,
	MC_H265_HIGH_TIER_LEVEL61 = 0x800000,
	MC_H265_HIGH_TIER_LEVEL62 = 0x2000000,
	MC_H265_MAIN_TIER_LEVEL1 = 0x1,
	MC_H265_MAIN_TIER_LEVEL2 = 0x4,
	MC_H265_MAIN_TIER_LEVEL21 = 0x10,
	MC_H265_MAIN_TIER_LEVEL3 = 0x40,
	MC_H265_MAIN_TIER_LEVEL31 = 0x100,
	MC_H265_MAIN_TIER_LEVEL4 = 0x400,
	MC_H265_MAIN_TIER_LEVEL41 = 0x1000,
	MC_H265_MAIN_TIER_LEVEL5 = 0x4000,
	MC_H265_MAIN_TIER_LEVEL51 = 0x10000,
	MC_H265_MAIN_TIER_LEVEL52 = 0x40000,
	MC_H265_MAIN_TIER_LEVEL6 = 0x100000,
	MC_H265_MAIN_TIER_LEVEL61 = 0x400000,
	MC_H265_MAIN_TIER_LEVEL62 = 0x1000000,
};


static bool convert_h265_level(unsigned int l, enum mc_h265_level *o)
{
	switch (l) {
	case 40:
		*o = MC_H265_MAIN_TIER_LEVEL4;
		return true;

	default:
		return false;
	}
}


static int get_supported_encodings(const enum vdef_encoding **encodings)
{
	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);
	*encodings = supported_encodings;
	return nb_supported_encodings;
}


static int get_supported_input_formats(const struct vdef_raw_format **formats)
{
	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);
	*formats = supported_formats;
	return NB_SUPPORTED_FORMATS;
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
	/* We’re not going to use the size returned by `getOutputBuffer`, but
	 * it might not support being passed `NULL`. */
	size_t discarded_size;
	const uint8_t *p =
		AMediaCodec_getOutputBuffer(self->mc, buf_idx, &discarded_size);

	p += info.offset;
	size_t size = info.size;

	pthread_mutex_lock(&self->pull.mutex);
	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264: {
		uint8_t **a[] = {
			&self->pending.sps,
			&self->pending.pps,
		};
		size_t *lens[] = {
			&self->pending.sps_size,
			&self->pending.pps_size,
		};
		copy_nalus(a, lens, 2, p, size);
		/* Initialize the H.264 writer */
		venc_h264_writer_new(self->pending.sps,
				     self->pending.sps_size,
				     self->pending.pps,
				     self->pending.pps_size,
				     &self->base->h264.ctx);
		break;
	}

	case VDEF_ENCODING_H265: {
		uint8_t **a[] = {
			&self->pending.vps,
			&self->pending.sps,
			&self->pending.pps,
		};
		size_t *lens[] = {
			&self->pending.vps_size,
			&self->pending.sps_size,
			&self->pending.pps_size,
		};
		copy_nalus(a, lens, 3, p, size);
		/* Initialize the H.265 writer */
		venc_h265_writer_new(self->pending.vps,
				     self->pending.vps_size,
				     self->pending.sps,
				     self->pending.sps_size,
				     self->pending.pps,
				     self->pending.pps_size,
				     &self->base->h265.ctx);
		break;
	}

	default:
		break;
	}
	pthread_mutex_unlock(&self->pull.mutex);

	pomp_evt_signal(self->out_evt);

	media_status_t status =
		AMediaCodec_releaseOutputBuffer(self->mc, buf_idx, false);
	if (status != AMEDIA_OK)
		ULOGE("AMediaCodec_releaseOutputBuffer");
}


static void on_output_event(struct pomp_evt *evt, void *userdata)
{
	struct venc_mediacodec *self = userdata;

	switch (self->state) {
	case RUNNING:
		pthread_mutex_lock(&self->pull.mutex);
		switch (self->base->config.encoding) {
		case VDEF_ENCODING_H264:
			if (self->pending.sps != NULL) {
				self->base->h264.sps = self->pending.sps;
				self->base->h264.sps_size =
					self->pending.sps_size;
				self->pending.sps = NULL;
			}
			if (self->pending.pps != NULL) {
				self->base->h264.pps = self->pending.pps;
				self->base->h264.pps_size =
					self->pending.pps_size;
				self->pending.pps = NULL;
			}
			break;
		case VDEF_ENCODING_H265:
			if (self->pending.vps != NULL) {
				self->base->h265.vps = self->pending.vps;
				self->base->h265.vps_size =
					self->pending.vps_size;
				self->pending.vps = NULL;
			}
			if (self->pending.sps != NULL) {
				self->base->h265.sps = self->pending.sps;
				self->base->h265.sps_size =
					self->pending.sps_size;
				self->pending.sps = NULL;
			}
			if (self->pending.pps != NULL) {
				self->base->h265.pps = self->pending.pps;
				self->base->h265.pps_size =
					self->pending.pps_size;
				self->pending.pps = NULL;
			}
			break;
		default:
			break;
		}
		pthread_mutex_unlock(&self->pull.mutex);

		while (true) {
			struct mbuf_coded_video_frame *frame;
			int res = mbuf_coded_video_frame_queue_pop(
				self->out_queue, &frame);
			if (res < 0) {
				if (res != -EAGAIN)
					ULOG_ERRNO(
						"mbuf_coded_video_frame_queue_pop",
						-res);
				break;
			}
			self->base->cbs.frame_output(
				self->base, 0, frame, self->base->userdata);
			mbuf_coded_video_frame_unref(frame);
		}

		pthread_mutex_lock(&self->pull.mutex);
		bool eos_flag = self->eos_flag;
		self->eos_flag = false;
		pthread_mutex_unlock(&self->pull.mutex);

		if (eos_flag) {
			AMediaCodec_flush(self->mc);
			if (self->base->cbs.flush)
				self->base->cbs.flush(self->base,
						      self->base->userdata);
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
			self->state = RUNNING;

			mbuf_raw_video_frame_queue_flush(self->in_queue);
			mbuf_raw_video_frame_queue_flush(self->meta_queue);
			mbuf_coded_video_frame_queue_flush(self->out_queue);
			AMediaCodec_flush(self->mc);

			if (self->base->cbs.flush != NULL)
				self->base->cbs.flush(self->base,
						      self->base->userdata);
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
			if (self->base->cbs.stop != NULL)
				self->base->cbs.stop(self->base,
						     self->base->userdata);
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
		pthread_mutex_lock(&self->push.mutex);
		self->push.flush_flag = true;
		pthread_cond_signal(&self->push.cond);
		pthread_mutex_unlock(&self->push.mutex);

		pthread_mutex_lock(&self->pull.mutex);
		self->pull.flush_flag = true;
		pthread_cond_signal(&self->pull.cond);
		pthread_mutex_unlock(&self->pull.mutex);

		AMediaCodec_flush(self->mc);

		self->state = WAITING_FOR_FLUSH;
	} else {
		pthread_mutex_lock(&self->push.mutex);
		self->push.eos_flag = true;
		pthread_cond_signal(&self->push.cond);
		pthread_mutex_unlock(&self->push.mutex);
	}

	return 0;
}


static int stop(struct venc_encoder *base)
{
	struct venc_mediacodec *self = base->derived;

	AMediaCodec_stop(self->mc);

	pthread_mutex_lock(&self->push.mutex);
	self->push.stop_flag = true;
	pthread_cond_signal(&self->push.cond);
	pthread_mutex_unlock(&self->push.mutex);

	pthread_mutex_lock(&self->pull.mutex);
	self->pull.stop_flag = true;
	pthread_cond_signal(&self->pull.cond);
	pthread_mutex_unlock(&self->pull.mutex);

	self->state = WAITING_FOR_STOP;

	return 0;
}


static int destroy(struct venc_encoder *base)
{
	struct venc_mediacodec *self = base->derived;

	if (self == NULL)
		return 0;

	free(self->pending.vps);
	free(self->pending.sps);
	free(self->pending.pps);

	if (self->libmediandk_handle != NULL)
		dlclose(self->libmediandk_handle);

	if (self->mc != NULL)
		AMediaCodec_stop(self->mc);

	if (self->pull.thread_created) {
		pthread_mutex_lock(&self->pull.mutex);
		self->pull.stop_flag = true;
		pthread_cond_signal(&self->pull.cond);
		pthread_mutex_unlock(&self->pull.mutex);

		pthread_join(self->pull.thread, NULL);
	}

	if (self->push.thread_created) {
		pthread_mutex_lock(&self->push.mutex);
		self->push.stop_flag = true;
		pthread_cond_signal(&self->push.cond);
		pthread_mutex_unlock(&self->push.mutex);

		pthread_join(self->push.thread, NULL);
	}

	if (self->out_evt != NULL) {
		if (pomp_evt_is_attached(self->out_evt, base->loop))
			pomp_evt_detach_from_loop(self->out_evt, base->loop);
		pomp_evt_destroy(self->out_evt);
	}

	if (self->out_queue != NULL)
		mbuf_coded_video_frame_queue_destroy(self->out_queue);

	if (self->meta_queue != NULL)
		mbuf_raw_video_frame_queue_destroy(self->meta_queue);

	if (self->in_queue != NULL) {
		struct pomp_evt *evt;
		mbuf_raw_video_frame_queue_get_event(self->in_queue, &evt);
		if (pomp_evt_is_attached(evt, base->loop))
			pomp_evt_detach_from_loop(evt, base->loop);
		mbuf_raw_video_frame_queue_destroy(self->in_queue);
	}

	if (self->mc != NULL)
		AMediaCodec_delete(self->mc);

	pthread_mutex_destroy(&self->push.mutex);
	pthread_cond_destroy(&self->push.cond);

	pthread_mutex_destroy(&self->pull.mutex);
	pthread_cond_destroy(&self->pull.cond);

	free(self);

	return 0;
}


static void push_eos(struct venc_mediacodec *self)
{
	ssize_t dec_buf_idx = AMediaCodec_dequeueInputBuffer(self->mc, -1);
	if (dec_buf_idx < 0) {
		ULOGE("AMediaCodec_dequeueInputBuffer");
		return;
	}

	media_status_t status = AMediaCodec_queueInputBuffer(
		self->mc,
		dec_buf_idx,
		0,
		0,
		0,
		AMEDIACODEC_BUFFER_FLAG_END_OF_STREAM);
	if (status != AMEDIA_OK)
		ULOGE("AMediaCodec_queueInputBuffer");

	pthread_mutex_lock(&self->pull.mutex);
	self->pull.eos_flag = true;
	pthread_cond_signal(&self->pull.cond);
	pthread_mutex_unlock(&self->pull.mutex);
}


static void push_frame(struct venc_mediacodec *self,
		       struct mbuf_raw_video_frame *frame)
{
	struct vdef_raw_frame frame_info;
	mbuf_raw_video_frame_get_frame_info(frame, &frame_info);

	size_t nplanes = vdef_get_raw_frame_plane_count(&frame_info.format);
	const void *planes[VDEF_RAW_MAX_PLANE_COUNT];
	size_t plane_lens[VDEF_RAW_MAX_PLANE_COUNT];

	for (size_t i = 0; i < nplanes; i++) {
		int res = mbuf_raw_video_frame_get_plane(
			frame, i, planes + i, plane_lens + i);
		if (res < 0) {
			for (size_t j = 0; j < i; j++)
				mbuf_raw_video_frame_release_plane(
					frame, j, planes[j]);
			mbuf_raw_video_frame_unref(frame);
			return;
		}
	}

	ssize_t buf_idx = AMediaCodec_dequeueInputBuffer(self->mc, -1);
	if (buf_idx < 0) {
		ULOGE("AMediaCodec_dequeueInputBuffer");
		for (size_t i = 0; i < nplanes; i++)
			mbuf_raw_video_frame_release_plane(frame, i, planes[i]);
		mbuf_raw_video_frame_unref(frame);
		return;
	}

	size_t buf_size;
	uint8_t *buf_data =
		AMediaCodec_getInputBuffer(self->mc, buf_idx, &buf_size);

	size_t required_len = 0;
	for (size_t i = 0; i < nplanes; i++)
		required_len += plane_lens[i];

	if (buf_size < required_len) {
		ULOGE("buf_size < required_len");
		for (size_t i = 0; i < nplanes; i++)
			mbuf_raw_video_frame_release_plane(frame, i, planes[i]);
		mbuf_raw_video_frame_unref(frame);
		AMediaCodec_queueInputBuffer(self->mc, buf_idx, 0, 0, 0, 0);
		return;
	}

	/**
	 * TODO: provide AMediaCodec buffers in our own input pool to avoid
	 * copies
	 */
	for (size_t i = 0; i < nplanes; i++) {
		memcpy(buf_data, planes[i], plane_lens[i]);
		buf_data += plane_lens[i];
	}

	for (size_t i = 0; i < nplanes; i++)
		mbuf_raw_video_frame_release_plane(frame, i, planes[i]);

	uint64_t ts_us = VDEF_ROUND(frame_info.info.timestamp * 1000000,
				    frame_info.info.timescale);

	media_status_t status = AMediaCodec_queueInputBuffer(
		self->mc, buf_idx, 0, required_len, ts_us, 0);
	if (status != AMEDIA_OK)
		ULOGE("AMediaCodec_queueInputBuffer");

	mbuf_raw_video_frame_queue_push(self->meta_queue, frame);
	mbuf_raw_video_frame_unref(frame);

	pthread_mutex_lock(&self->pull.mutex);
	pthread_cond_signal(&self->pull.cond);
	pthread_mutex_unlock(&self->pull.mutex);
}


static void *push_routine(void *ptr)
{
	struct venc_mediacodec *self = ptr;
	unsigned int decimation_counter = 0;

	pthread_mutex_lock(&self->push.mutex);
	while (true) {
		if (self->push.stop_flag) {
			self->push.stop_flag = false;
			pomp_evt_signal(self->out_evt);
			pthread_mutex_unlock(&self->push.mutex);
			break;
		}

		if (self->push.flush_flag) {
			self->push.flush_flag = false;
			pomp_evt_signal(self->out_evt);
			pthread_cond_wait(&self->push.cond, &self->push.mutex);
			continue;
		}

		struct mbuf_raw_video_frame *frame;
		int res =
			mbuf_raw_video_frame_queue_pop(self->in_queue, &frame);
		if (res == 0) {
			if (decimation_counter == 0) {
				pthread_mutex_unlock(&self->push.mutex);
				push_frame(self, frame);
				pthread_mutex_lock(&self->push.mutex);
			}
			decimation_counter += 1;
			if (decimation_counter >= self->dynconf.decimation)
				decimation_counter = 0;
			continue;
		} else if (res == -EAGAIN && self->push.eos_flag) {
			self->push.eos_flag = false;
			pthread_mutex_unlock(&self->push.mutex);
			push_eos(self);
			pthread_mutex_lock(&self->push.mutex);
			continue;
		} else if (res != -EAGAIN) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_pop", -res);
		}

		pthread_cond_wait(&self->push.cond, &self->push.mutex);
	}

	return NULL;
}


static void release_mc_mem(void *data, size_t len, void *userdata)
{
	size_t idx = len;
	AMediaCodec *mc = userdata;

	AMediaCodec_releaseOutputBuffer(mc, idx, false);
}


static void pull_frame(struct venc_mediacodec *self,
		       struct mbuf_raw_video_frame *meta_frame)
{
	AMediaCodecBufferInfo info;
	ssize_t buffer_index;

	struct mbuf_coded_video_frame_cbs frame_cbs = {
		.pre_release = self->base->cbs.pre_release,
		.pre_release_userdata = self->base->userdata,
	};

	while (true) {
		buffer_index =
			AMediaCodec_dequeueOutputBuffer(self->mc, &info, -1);

		if (buffer_index == AMEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED) {
			/* MediaCodec sends both `FORMAT_CHANGED` events and
			 * codec-config buffers. We use only the latter. */
			continue;
		}

		if (buffer_index < 0) {
			/* We assume that we got woken up by the input routine
			 * calling `stop` or `flush` */
			mbuf_raw_video_frame_unref(meta_frame);
			return;
		}

		/* Buffers marked with `AMEDIACODEC_BUFFER_FLAG_CODEC_CONFIG`
		 * contain only parameter sets. */
		if (info.flags & AMEDIACODEC_BUFFER_FLAG_CODEC_CONFIG)
			store_ps(self, buffer_index, info);
		else
			break;
	}

	size_t out_size;
	uint8_t *out_data =
		AMediaCodec_getOutputBuffer(self->mc, buffer_index, &out_size) +
		info.offset;

	struct mbuf_mem *mem;
	/* buffer index is passed as argument instead of out_data buffer's
	 * length */
	int res = mbuf_mem_generic_wrap(
		out_data, buffer_index, release_mc_mem, self->mc, &mem);
	if (res < 0) {
		AMediaCodec_releaseOutputBuffer(self->mc, buffer_index, false);
		mbuf_raw_video_frame_unref(meta_frame);
		return;
	}

	struct vdef_raw_frame meta_info;
	mbuf_raw_video_frame_get_frame_info(meta_frame, &meta_info);

	struct vdef_coded_frame out_info = {
		.info = meta_info.info,
		.type = VDEF_CODED_FRAME_TYPE_UNKNOWN,
		.format = (self->base->config.encoding == VDEF_ENCODING_H264)
				  ? vdef_h264_byte_stream
				  : vdef_h265_byte_stream,
	};

	uint8_t start_code[] = {0, 0, 0, 1};
	const uint8_t *p = out_data;
	size_t n = info.size;
	while (true) {
		const uint8_t *q = p + 1;
		size_t m = n - 1;

		bool end;
		while (true) {
			if (m < sizeof(start_code)) {
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

	struct mbuf_coded_video_frame *out_frame;
	res = mbuf_coded_video_frame_new(&out_info, &out_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_few", -res);
		mbuf_raw_video_frame_unref(meta_frame);
		mbuf_mem_unref(mem);
		return;
	}
	res = mbuf_coded_video_frame_set_callbacks(out_frame, &frame_cbs);
	if (res < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_set_callbacks", -res);

	res = mbuf_raw_video_frame_foreach_ancillary_data(
		meta_frame,
		mbuf_coded_video_frame_ancillary_data_copier,
		out_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_foreach_ancillary_data", -res);
		mbuf_raw_video_frame_unref(meta_frame);
		mbuf_mem_unref(mem);
		mbuf_coded_video_frame_unref(out_frame);
		return;
	}

	struct vmeta_frame *metadata;
	res = mbuf_raw_video_frame_get_metadata(meta_frame, &metadata);
	mbuf_raw_video_frame_unref(meta_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_metadata", -res);
		mbuf_mem_unref(mem);
		mbuf_coded_video_frame_unref(out_frame);
		return;
	}

	if (metadata != NULL) {
		res = mbuf_coded_video_frame_set_metadata(out_frame, metadata);
		vmeta_frame_unref(metadata);
		if (res < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_set_metadata", -res);
			mbuf_mem_unref(mem);
			mbuf_coded_video_frame_unref(out_frame);
			return;
		}
	}

	if (self->base->config.encoding == VDEF_ENCODING_H264) {
		/* Add generated NAL units */
		/* TODO: Set SPS and PPS in base before first frame */
		res = venc_h264_generate_nalus(
			self->base, out_frame, &out_info);
		if (res < 0) {
			ULOG_ERRNO("venc_h264_generate_nalus", -res);
			mbuf_mem_unref(mem);
			mbuf_coded_video_frame_unref(out_frame);
			return;
		}
	} else if (self->base->config.encoding == VDEF_ENCODING_H265) {
		/* Add generated NAL units */
		/* TODO: Set VPS, SPS and PPS in base before first frame */
		res = venc_h265_generate_nalus(
			self->base, out_frame, &out_info);
		if (res < 0) {
			ULOG_ERRNO("venc_h265_generate_nalus", -res);
			mbuf_mem_unref(mem);
			mbuf_coded_video_frame_unref(out_frame);
			return;
		}
	}

	p = out_data;
	n = info.size;
	while (true) {
		const uint8_t *q = p + 1;
		size_t m = n - 1;

		bool end;
		while (true) {
			if (m < sizeof(start_code)) {
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

		struct vdef_nalu nalu = {
			.size = q - p,
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
			out_frame, mem, p - out_data, &nalu);
		if (res < 0) {
			mbuf_mem_unref(mem);
			mbuf_coded_video_frame_unref(out_frame);
			return;
		}

		if (end)
			break;

		p = q;
		n = m;
	}

	mbuf_mem_unref(mem);

	struct timespec cur_ts;
	time_get_monotonic(&cur_ts);
	uint64_t ts_us;
	time_timespec_to_us(&cur_ts, &ts_us);

	res = mbuf_coded_video_frame_add_ancillary_buffer(
		out_frame,
		VENC_ANCILLARY_KEY_OUTPUT_TIME,
		&ts_us,
		sizeof(ts_us));
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer", -res);
		mbuf_coded_video_frame_unref(out_frame);
		return;
	}

	res = mbuf_coded_video_frame_finalize(out_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_finalize", -res);
		mbuf_coded_video_frame_unref(out_frame);
		return;
	}

	res = mbuf_coded_video_frame_queue_push(self->out_queue, out_frame);
	mbuf_coded_video_frame_unref(out_frame);
	if (res < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_queue_push", -res);

	pomp_evt_signal(self->out_evt);
}


static void *pull_routine(void *ptr)
{
	struct venc_mediacodec *self = ptr;

	pthread_mutex_lock(&self->pull.mutex);
	while (true) {
		if (self->pull.stop_flag) {
			self->pull.stop_flag = false;
			pomp_evt_signal(self->out_evt);
			pthread_mutex_unlock(&self->pull.mutex);
			break;
		}

		if (self->pull.flush_flag) {
			self->pull.flush_flag = false;
			pomp_evt_signal(self->out_evt);
			pthread_cond_wait(&self->pull.cond, &self->pull.mutex);
			continue;
		}

		struct mbuf_raw_video_frame *frame;
		int res = mbuf_raw_video_frame_queue_pop(self->meta_queue,
							 &frame);
		if (res == 0) {
			pthread_mutex_unlock(&self->pull.mutex);
			pull_frame(self, frame);
			pthread_mutex_lock(&self->pull.mutex);
			continue;
		} else if (res == -EAGAIN && self->pull.eos_flag) {
			self->pull.eos_flag = false;
			self->eos_flag = true;
			pomp_evt_signal(self->out_evt);
		} else if (res != -EAGAIN) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_pop", -res);
		}

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

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, false);

	if (self->state == WAITING_FOR_FLUSH ||
	    self->state == WAITING_FOR_STOP || self->push.eos_flag == true ||
	    self->pull.eos_flag == true)
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
		ULOG_ERRNO("mbuf_raw_video_frame_get_packed_buffer", -ret);
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
	struct venc_mediacodec *self;

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

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	base->derived = self;
	self->base = base;
	self->state = RUNNING;

	self->mc = AMediaCodec_createEncoderByType(mime_type);
	if (self->mc == NULL) {
		ret = -ENOSYS;
		ULOG_ERRNO("AMediaCodec_createEncoderByType", -ret);
		goto error;
	}

	AMediaFormat *format = AMediaFormat_new();

	/* The keys `AMediaFormat` accepts are detailed at
	 * https://developer.android.com/reference/android/media/MediaFormat.html
	 */

	AMediaFormat_setString(format, "mime", mime_type);

	AMediaFormat_setInt32(format, "width", c->input.info.resolution.width);
	AMediaFormat_setInt32(
		format, "height", c->input.info.resolution.height);

	enum color_format color_format;
	if (vdef_raw_format_cmp(&c->input.format, &vdef_i420)) {
		color_format = YUV420_PLANAR;
	} else if (vdef_raw_format_cmp(&c->input.format, &vdef_nv12)) {
		color_format = YUV420_SEMIPLANAR;
	} else {
		AMediaFormat_delete(format);
		ret = -ENOSYS;
		goto error;
	}
	AMediaFormat_setInt32(format, "color-format", color_format);

	float framerate = ((float)c->input.info.framerate.num) /
			  ((float)c->input.info.framerate.den);
	AMediaFormat_setFloat(format, "frame-rate", framerate);

	switch (c->encoding) {
	case VDEF_ENCODING_H264:
		self->dynconf = (struct venc_dyn_config){
			.qp = c->h264.qp,
			.target_bitrate = c->h264.target_bitrate,
			.decimation = c->h264.decimation,
		};
		ret = h264_fill_format(format, c);
		if (ret < 0) {
			AMediaFormat_delete(format);
			goto error;
		}
		break;

	case VDEF_ENCODING_H265:
		self->dynconf = (struct venc_dyn_config){
			.qp = c->h265.qp,
			.target_bitrate = c->h265.target_bitrate,
			.decimation = c->h265.decimation,
		};
		ret = h265_fill_format(format, c);
		if (ret < 0) {
			AMediaFormat_delete(format);
			goto error;
		}
		break;

	default:
		AMediaFormat_delete(format);
		ret = -ENOSYS;
		goto error;
	}

	media_status_t status =
		AMediaCodec_configure(self->mc,
				      format,
				      NULL,
				      NULL,
				      AMEDIACODEC_CONFIGURE_FLAG_ENCODE);

	AMediaFormat_delete(format);

	if (status != AMEDIA_OK) {
		ret = -EPROTO;
		ULOG_ERRNO("mediacodec.configure status=%d", -ret, status);
		goto error;
	}

	struct mbuf_raw_video_frame_queue_args queue_args = {
		.filter = input_filter,
		.filter_userdata = self,
	};
	ret = mbuf_raw_video_frame_queue_new_with_args(&queue_args,
						       &self->in_queue);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_new:input", -ret);
		goto error;
	}

	ret = mbuf_raw_video_frame_queue_new(&self->meta_queue);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_queue_new:meta", -ret);
		goto error;
	}

	ret = mbuf_coded_video_frame_queue_new(&self->out_queue);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_queue_new:output", -ret);
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
		ULOG_ERRNO("pomp_event_attach_to_loop", -ret);
		goto error;
	}

	status = AMediaCodec_start(self->mc);
	if (status != AMEDIA_OK) {
		ret = -EPROTO;
		ULOG_ERRNO("mediacodec.start status=%d", -ret, status);
		goto error;
	}

	ret = pthread_create(&self->push.thread, NULL, push_routine, self);
	if (ret != 0) {
		ret = -ret;
		ULOG_ERRNO("pthread_create", -ret);
		goto error;
	}
	self->push.thread_created = true;

	ret = pthread_create(&self->pull.thread, NULL, pull_routine, self);
	if (ret != 0) {
		ret = -ret;
		ULOG_ERRNO("pthread_create", -ret);
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

	AMediaFormat *format = AMediaFormat_new();
	if (format == NULL) {
		ULOG_ERRNO("AMediaFormat_new", ENOMEM);
		return -ENOMEM;
	}
	int32_t bitrate = config->target_bitrate * decimation;
	AMediaFormat_setInt32(format, "video-bitrate", bitrate);

	media_status_t status = self->set_parameters(self->mc, format);
	AMediaFormat_delete(format);

	if (status != AMEDIA_OK) {
		ULOG_ERRNO("AMediaCodec_setParameters", ENOSYS);
		return -ENOSYS;
	}
	self->dynconf.target_bitrate = config->target_bitrate;
	self->dynconf.decimation = decimation;

	return 0;
}


const struct venc_ops venc_mediacodec_ops = {
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
