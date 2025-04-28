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

#define ULOG_TAG venc_x264
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "venc_x264_priv.h"

#include <futils/futils.h>


#define VENC_X264_MIN_QP 10
#define VENC_X264_MAX_QP 51


#define NB_SUPPORTED_ENCODINGS 1
#define NB_SUPPORTED_FORMATS 5
static struct vdef_raw_format supported_formats[NB_SUPPORTED_FORMATS];
static enum vdef_encoding supported_encodings[NB_SUPPORTED_ENCODINGS];
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;
static void initialize_supported_formats(void)
{
	supported_formats[0] = vdef_i420;
	supported_formats[1] = vdef_yv12;
	supported_formats[2] = vdef_nv12;
	supported_formats[3] = vdef_nv21;
	supported_formats[4] = vdef_gray;

	supported_encodings[0] = VDEF_ENCODING_H264;
}


static void call_flush_done(void *userdata)
{
	struct venc_x264 *self = userdata;

	if (self->base->cbs.flush)
		self->base->cbs.flush(self->base, self->base->userdata);
}


static void call_stop_done(void *userdata)
{
	struct venc_x264 *self = userdata;

	if (self->base->cbs.stop)
		self->base->cbs.stop(self->base, self->base->userdata);
}


static int x264_to_ulog_level(int level)
{
	switch (level) {
	case X264_LOG_ERROR:
		return ULOG_ERR;
	case X264_LOG_WARNING:
		return ULOG_WARN;
	case X264_LOG_INFO:
		return ULOG_INFO;
	case X264_LOG_DEBUG:
		return ULOG_DEBUG;
	default:
		return 0;
	}
}


static void
x264_log_cb(void *p_unused, int i_level, const char *psz_fmt, va_list arg)
{
	char *str = NULL;
	int l = x264_to_ulog_level(i_level);
	if (l == 0)
		return;
	int ret = asprintf(&str, "x264: %s", psz_fmt);
#ifdef __clang__
#	pragma clang diagnostic push
#	pragma clang diagnostic ignored "-Wformat-nonliteral"
#endif
	if (ret > 0 && str != NULL)
		ULOG_PRI_VA(l, str, arg);
#ifdef __clang__
#	pragma clang diagnostic pop
#endif
	free(str);
}


static int reopen_encoder(struct venc_x264 *self)
{
	int ret;
	x264_param_t x264_params = {};

	if (!atomic_load(&self->reopen_needed))
		return -EALREADY;

	x264_encoder_parameters(self->x264, &x264_params);
	/* Deep copy parameters that are freed when closing encoder */
	x264_params.rc.psz_stat_in = strdup(x264_params.rc.psz_stat_in);
	x264_params.rc.psz_stat_out = strdup(x264_params.rc.psz_stat_out);
	x264_encoder_close(self->x264);
	self->x264 = NULL;

	VENC_LOGI("reopening encoder");

	/* Initialize the encoder */
	self->x264 = x264_encoder_open(&x264_params);
	if (self->x264 == NULL) {
		ret = -EPROTO;
		VENC_LOGE("x264_encoder_open failed");
		goto out;
	}
	ret = 0;

	atomic_store(&self->reopen_needed, false);

out:
	free(x264_params.rc.psz_stat_in);
	free(x264_params.rc.psz_stat_out);
	return ret;
}


static void mbox_cb(int fd, uint32_t revents, void *userdata)
{
	struct venc_x264 *self = userdata;
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
			if (err < 0)
				VENC_LOG_ERRNO("pomp_loop_idle_add_with_cookie",
					       -err);
			break;
		case VENC_MSG_STOP:
			err = pomp_loop_idle_add_with_cookie(
				self->base->loop, call_stop_done, self, self);
			if (err < 0)
				VENC_LOG_ERRNO("pomp_loop_idle_add_with_cookie",
					       -err);
			break;
		default:
			VENC_LOGE("unknown message: %c", message);
			break;
		}
	} while (ret == 0);
}


static void enc_out_queue_evt_cb(struct pomp_evt *evt, void *userdata)
{
	struct venc_x264 *self = userdata;
	struct mbuf_coded_video_frame *out_frame = NULL;
	int err;

	do {
		err = mbuf_coded_video_frame_queue_pop(self->enc_out_queue,
						       &out_frame);
		if (err == -EAGAIN) {
			return;
		} else if (err < 0) {
			VENC_LOG_ERRNO(
				"mbuf_coded_video_frame_queue_pop:enc_out",
				-err);
			return;
		}
		struct vdef_coded_frame out_info = {};
		err = mbuf_coded_video_frame_get_frame_info(out_frame,
							    &out_info);
		if (err < 0)
			VENC_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info",
				       -err);

		if (!atomic_load(&self->flush_discard)) {
			self->base->cbs.frame_output(
				self->base, 0, out_frame, self->base->userdata);
			self->base->counters.out++;
		} else {
			VENC_LOGD("discarding frame %d", out_info.info.index);
		}

		mbuf_coded_video_frame_unref(out_frame);
	} while (err == 0);
}


static int generate_sps_pps(struct venc_x264 *self)
{
	int ret, i, nalu_count = 0;
	x264_nal_t *nalu = NULL;
	size_t len;

	ret = x264_encoder_headers(self->x264, &nalu, &nalu_count);
	if (ret < 0) {
		VENC_LOG_ERRNO("x264_encoder_headers", -ret);
		return ret;
	}
	VENC_LOG_ERRNO_RETURN_ERR_IF(nalu_count < 2, ENODATA);

	xfree((void **)&self->base->h264.sps);
	self->base->h264.sps_size = 0;
	xfree((void **)&self->base->h264.pps);
	self->base->h264.pps_size = 0;

	for (i = 0; i < nalu_count; i++) {
		if ((nalu[i].i_type == NAL_SPS) && (!self->base->h264.sps)) {
			len = nalu[i].i_payload - nalu[i].i_padding;
			VENC_LOG_ERRNO_RETURN_ERR_IF(len <= 4, ENODATA);
			self->base->h264.sps = malloc(len - 4);
			VENC_LOG_ERRNO_RETURN_ERR_IF(
				self->base->h264.sps == NULL, ENOMEM);
			memcpy(self->base->h264.sps,
			       nalu[i].p_payload + 4,
			       len - 4);
			self->base->h264.sps_size = len - 4;
		} else if ((nalu[i].i_type == NAL_PPS) &&
			   (!self->base->h264.pps)) {
			len = nalu[i].i_payload - nalu[i].i_padding;
			VENC_LOG_ERRNO_RETURN_ERR_IF(len <= 4, ENODATA);
			self->base->h264.pps = malloc(len - 4);
			VENC_LOG_ERRNO_RETURN_ERR_IF(
				self->base->h264.pps == NULL, ENOMEM);
			memcpy(self->base->h264.pps,
			       nalu[i].p_payload + 4,
			       len - 4);
			self->base->h264.pps_size = len - 4;
		}
	}

	VENC_LOG_ERRNO_RETURN_ERR_IF(self->base->h264.sps == NULL, ENODATA);
	VENC_LOG_ERRNO_RETURN_ERR_IF(self->base->h264.sps_size == 0, ENODATA);
	VENC_LOG_ERRNO_RETURN_ERR_IF(self->base->h264.pps == NULL, ENODATA);
	VENC_LOG_ERRNO_RETURN_ERR_IF(self->base->h264.pps_size == 0, ENODATA);

	ret = h264_reader_parse_nalu(self->h264_reader,
				     0,
				     self->base->h264.sps,
				     self->base->h264.sps_size);
	if (ret < 0) {
		VENC_LOG_ERRNO("h264_reader_parse", -ret);
		return ret;
	}

	ret = h264_reader_parse_nalu(self->h264_reader,
				     0,
				     self->base->h264.pps,
				     self->base->h264.pps_size);
	if (ret < 0) {
		VENC_LOG_ERRNO("h264_reader_parse", -ret);
		return ret;
	}

	return 0;
}


static enum vdef_coded_frame_type x264_to_vdef_frame_type(int frame_type)
{
	if (frame_type == X264_TYPE_IDR)
		return VDEF_CODED_FRAME_TYPE_IDR;
	else if (frame_type == X264_TYPE_I)
		return VDEF_CODED_FRAME_TYPE_I;
	else if (frame_type == X264_TYPE_P)
		return VDEF_CODED_FRAME_TYPE_P;
	else
		return VDEF_CODED_FRAME_TYPE_NOT_CODED;
}


static int add_x264_nalus(struct venc_x264 *self,
			  struct mbuf_coded_video_frame *out_frame,
			  struct vdef_coded_frame *out_info,
			  x264_nal_t *nalu,
			  size_t nalu_count)
{
	int res, err;
	size_t i;
	struct mbuf_mem *nalus_mem;
	void *nalu_data;
	uint8_t *data;
	size_t mem_size = 0;
	size_t nalu_offset = 4;
	size_t nalu_len = 0;
	size_t offset = 0, base_offset;
	struct vdef_nalu out_nalu = {0};

	for (i = 0; i < nalu_count; i++)
		mem_size += nalu[i].i_payload - nalu[i].i_padding;

	res = mbuf_mem_generic_new(mem_size, &nalus_mem);
	if (res < 0) {
		VENC_LOG_ERRNO("mbuf_mem_generic_new", -res);
		return res;
	}
	res = mbuf_mem_get_data(nalus_mem, &nalu_data, &mem_size);
	if (res < 0) {
		VENC_LOG_ERRNO("mbuf_mem_get_data", -res);
		goto out;
	}
	data = nalu_data;

	for (i = 0; i < nalu_count; i++) {
		/* Discard AUD, SPS, PPS and SEI NAL units (we have already
		 * inserted these NAL units generated ourselves) */
		if ((nalu[i].i_type == H264_NALU_TYPE_AUD) ||
		    (nalu[i].i_type == H264_NALU_TYPE_SPS) ||
		    (nalu[i].i_type == H264_NALU_TYPE_PPS) ||
		    (nalu[i].i_type == H264_NALU_TYPE_SEI))
			continue;

		base_offset = offset;
		out_nalu.size = 0;
		out_nalu.importance = 0;
		nalu_len = nalu[i].i_payload - nalu[i].i_padding - nalu_offset;
		if (out_info->format.data_format ==
		    VDEF_CODED_DATA_FORMAT_BYTE_STREAM) {
			if (mem_size < 4) {
				VENC_LOG_ERRNO("", ENOBUFS);
				res = -ENOBUFS;
				goto out;
			}
			uint32_t start_code = htonl(0x00000001);
			memcpy(data + offset, &start_code, 4);
			offset += 4;
			out_nalu.size = 4;
			mem_size -= 4;
		} else if (out_info->format.data_format ==
			   VDEF_CODED_DATA_FORMAT_AVCC) {
			if (mem_size < 4) {
				VENC_LOG_ERRNO("", ENOBUFS);
				res = -ENOBUFS;
			}
			uint32_t nalu_size = htonl(nalu_len);
			memcpy(data + offset, &nalu_size, 4);
			offset += 4;
			out_nalu.size = 4;
			mem_size -= 4;
		}
		if (nalu_len > mem_size) {
			VENC_LOG_ERRNO("", ENOBUFS);
			res = -ENOBUFS;
			goto out;
		}
		memcpy(data + offset,
		       nalu[i].p_payload + nalu_offset,
		       nalu_len);
		out_nalu.size += nalu_len;
		out_nalu.h264.type = (enum h264_nalu_type)nalu[i].i_type;
		out_nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN; /* TODO */
		out_nalu.h264.slice_mb_count =
			(out_nalu.h264.type == H264_NALU_TYPE_SLICE ||
			 out_nalu.h264.type == H264_NALU_TYPE_SLICE_IDR)
				? nalu[i].i_last_mb - nalu[i].i_first_mb + 1
				: 0;
		res = mbuf_coded_video_frame_add_nalu(
			out_frame, nalus_mem, base_offset, &out_nalu);
		if (res < 0) {
			VENC_LOG_ERRNO("mbuf_coded_video_frame_add_nalu", -res);
			goto out;
		}
		offset += nalu_len;
	}
out:
	err = mbuf_mem_unref(nalus_mem);
	if (err != 0)
		VENC_LOG_ERRNO("mbuf_mem_unref", -err);

	return res;
}


static int fill_frame(struct venc_x264 *self,
		      struct mbuf_raw_video_frame *in_frame,
		      struct mbuf_coded_video_frame *out_frame,
		      struct vdef_coded_frame *out_info,
		      x264_picture_t *out_picture,
		      x264_nal_t *nalu,
		      int nalu_count,
		      int frame_size)
{
	int ret = 0;
	struct vmeta_frame *metadata = NULL;

	if (frame_size == 0)
		goto out;

	/* Ancillary data */
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

	/* Add generated NAL units */
	ret = venc_h264_generate_nalus(self->base, out_frame, out_info);
	if (ret < 0) {
		VENC_LOG_ERRNO("venc_h264_generate_nalus", -ret);
		goto out;
	}

	/* Add x264 NAL units */
	ret = add_x264_nalus(self, out_frame, out_info, nalu, nalu_count);

out:
	if (metadata)
		vmeta_frame_unref(metadata);
	return ret;
}


static int encode_frame(struct venc_x264 *self,
			struct mbuf_raw_video_frame *in_frame)
{
	int ret, err, nalu_count = 0, frame_size;
	x264_nal_t *nalu = NULL;
	x264_picture_t out_picture;
	struct vdef_raw_frame info;
	const void *plane_data;
	const uint8_t *in_data;
	size_t len, nalu_len = 0, nalu_offset = 4;
	ssize_t i;
	struct mbuf_raw_video_frame *enc_frame;
	struct mbuf_coded_video_frame *out_frame = NULL;
	struct vdef_coded_frame out_info = {};
	struct vdef_raw_frame in_info;
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;
	unsigned int plane_count = 0;

	struct mbuf_coded_video_frame_cbs frame_cbs = {
		.pre_release = self->base->cbs.pre_release,
		.pre_release_userdata = self->base->userdata,
	};

	if (in_frame != NULL) {
		ret = mbuf_raw_video_frame_get_frame_info(in_frame, &info);
		if (ret < 0) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info",
				       -ret);
			return ret;
		}

		/* Frame skipping in case of decimation */
		if (self->input_frame_cnt %
			    self->base->config.h264.decimation !=
		    0) {
			self->x264_pts++;
			self->input_frame_cnt++;
			return 0;
		}

		time_get_monotonic(&cur_ts);
		time_timespec_to_us(&cur_ts, &ts_us);

		if (!vdef_raw_format_intersect(&info.format,
					       supported_formats,
					       NB_SUPPORTED_FORMATS)) {
			ret = -ENOSYS;
			VENC_LOG_ERRNO(
				"unsupported format:"
				" " VDEF_RAW_FORMAT_TO_STR_FMT,
				-ret,
				VDEF_RAW_FORMAT_TO_STR_ARG(&info.format));
			return ret;
		}

		if (vdef_raw_format_cmp(&info.format, &vdef_gray)) {
			plane_count = 1;
			/* Add the dummy plane */
			self->in_picture.img.plane[1] = self->dummy_uv_plane;
			self->in_picture.img.i_stride[1] =
				self->dummy_uv_plane_stride;
		} else if (vdef_raw_format_cmp(&info.format, &vdef_nv12) ||
			   vdef_raw_format_cmp(&info.format, &vdef_nv21)) {
			plane_count = 2;
		} else if (vdef_raw_format_cmp(&info.format, &vdef_i420) ||
			   vdef_raw_format_cmp(&info.format, &vdef_yv12)) {
			plane_count = 3;
		}

		for (unsigned int i = 0; i < plane_count; i++) {
			ret = mbuf_raw_video_frame_get_plane(
				in_frame, i, &plane_data, &len);
			if (ret == 0) {
				in_data = plane_data;
				self->in_picture.img.plane[i] =
					(uint8_t *)in_data;
				self->in_picture.img.i_stride[i] =
					info.plane_stride[i];
				mbuf_raw_video_frame_release_plane(
					in_frame, i, plane_data);
			}
			if (ret < 0) {
				/* TODO: don't forget to drop the frame
				 * otherwise it remains in the queue. */
				VENC_LOG_ERRNO(
					"mbuf_raw_video_frame_get_plane(%u)",
					-ret,
					i);
				return ret;
			}
		}

		err = mbuf_raw_video_frame_add_ancillary_buffer(
			in_frame,
			VENC_ANCILLARY_KEY_DEQUEUE_TIME,
			&ts_us,
			sizeof(ts_us));
		if (err < 0) {
			VENC_LOGW_ERRNO(
				"mbuf_raw_video_frame_add_ancillary_buffer",
				-err);
		}

		self->in_picture.i_pts = self->x264_pts;
		self->in_picture.opaque = in_frame;
		self->in_picture.i_type = X264_TYPE_AUTO;
		if (atomic_load(&self->insert_idr)) {
			self->in_picture.i_type = X264_TYPE_IDR;
			atomic_store(&self->insert_idr, false);
		}

		self->input_frame_cnt++;
		self->x264_pts++;
	}

	memset(&out_picture, 0, sizeof(out_picture));
	if (in_frame != NULL)
		self->base->counters.pushed++;
	else
		atomic_store(&self->reopen_needed, true);
	ret = x264_encoder_encode(self->x264,
				  &nalu,
				  &nalu_count,
				  in_frame ? &self->in_picture : NULL,
				  &out_picture);
	if (ret < 0) {
		VENC_LOG_ERRNO("x264_encoder_encode", -ret);
		return ret;
	}
	frame_size = ret;

	/* Push the frame in the encoder input queue if provided*/
	if (in_frame) {
		ret = mbuf_raw_video_frame_queue_push(self->enc_in_queue,
						      in_frame);
		if (ret < 0) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_push:enc_in",
				       -ret);
			return ret;
		}
	}

	/* Return if no frame was output */
	if (out_picture.opaque == NULL)
		return 0;

	self->base->counters.pulled++;

	/* Find the frame (non-blocking);
	 * drop frames until the frame corresponding to the
	 * x264 output is found */
	enc_frame = NULL;
	do {
		if (enc_frame != NULL) {
			mbuf_raw_video_frame_unref(enc_frame);
			enc_frame = NULL;
		}
		ret = mbuf_raw_video_frame_queue_pop(self->enc_in_queue,
						     &enc_frame);
		if (ret == -EAGAIN) {
			return 0;
		} else if (ret < 0) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_pop:enc_in",
				       -ret);
			return ret;
		}
	} while (enc_frame != out_picture.opaque);

	ret = mbuf_raw_video_frame_get_frame_info(enc_frame, &in_info);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		goto out;
	}

	out_info.info = in_info.info;
	out_info.format = self->output_format;
	out_info.type = x264_to_vdef_frame_type(out_picture.i_type);
	out_info.layer = 0;

	if (frame_size == 0)
		out_info.type = VDEF_CODED_FRAME_TYPE_NOT_CODED;

	self->recovery_point = false;
	for (i = 0; i < nalu_count; i++) {
		if ((*(nalu[i].p_payload + nalu_offset) & 0x1F) !=
		    H264_NALU_TYPE_SEI) {
			continue;
		}
		nalu_len = nalu[i].i_payload - nalu[i].i_padding - nalu_offset;

		ret = h264_reader_parse_nalu(self->h264_reader,
					     0,
					     nalu[i].p_payload + nalu_offset,
					     nalu_len);
		if (ret < 0) {
			VENC_LOG_ERRNO("h264_reader_parse_nalu", -ret);
			goto out;
		}

		if (self->recovery_point &&
		    (self->base->config.h264.intra_refresh !=
		     VENC_INTRA_REFRESH_NONE))
			out_info.type = VDEF_CODED_FRAME_TYPE_P_IR_START;
	}

	ret = mbuf_coded_video_frame_new(&out_info, &out_frame);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_new", -ret);
		goto out;
	}
	ret = mbuf_coded_video_frame_set_callbacks(out_frame, &frame_cbs);
	if (ret < 0)
		VENC_LOG_ERRNO("mbuf_coded_video_frame_set_callbacks", -ret);

	ret = fill_frame(self,
			 enc_frame,
			 out_frame,
			 &out_info,
			 &out_picture,
			 nalu,
			 nalu_count,
			 frame_size);
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
	if (enc_frame)
		mbuf_raw_video_frame_unref(enc_frame);
	if (out_frame)
		mbuf_coded_video_frame_unref(out_frame);

	return ret;
}


static int complete_flush(struct venc_x264 *self)
{
	int ret;

	if (atomic_load(&self->flush_discard)) {

		while ((ret = x264_encoder_delayed_frames(self->x264)) > 0) {
			ret = encode_frame(self, NULL);
			if (ret < 0)
				return ret;
		}
		/* Flush the input queue */
		ret = mbuf_raw_video_frame_queue_flush(self->in_queue);
		if (ret < 0) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_flush:input",
				       -ret);
			return ret;
		}
		/* Flush the encoder input queue */
		ret = mbuf_raw_video_frame_queue_flush(self->enc_in_queue);
		if (ret < 0) {
			VENC_LOG_ERRNO(
				"mbuf_raw_video_frame_queue_flush:enc_in",
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


static void check_input_queue(struct venc_x264 *self)
{
	int ret, err = 0;
	struct mbuf_raw_video_frame *in_frame;

	ret = mbuf_raw_video_frame_queue_peek(self->in_queue, &in_frame);
	while (ret >= 0) {
		/* Once the encoder is flushed, it does not accept frames
		 * anymore and must be re-opened before encoding frames again */
		if (in_frame && atomic_load(&self->reopen_needed)) {
			err = reopen_encoder(self);
			if (err < 0)
				VENC_LOG_ERRNO("reopen_encoder", -err);
		}
		/* Push the input frame */
		/* Encode the frame */
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
		if (atomic_load(&self->flushing) && ret == -EAGAIN) {
			in_frame = NULL;
			if (!atomic_load(&self->flush_discard)) {
				ret = x264_encoder_delayed_frames(self->x264);
				if (ret == 0) {
					ret = complete_flush(self);
					if (ret < 0)
						VENC_LOG_ERRNO("complete_flush",
							       -ret);
					continue;
				}
				/* Else we proceed to call encode_frame()
				 * without an input frame to flush the
				 * encoder */
			}
		}
	}

	if ((ret == -EAGAIN) && atomic_load(&self->flushing) &&
	    (!atomic_load(&self->flush_discard))) {
		ret = x264_encoder_delayed_frames(self->x264);
		if (ret == 0) {
			ret = complete_flush(self);
			if (ret < 0)
				VENC_LOG_ERRNO("complete_flush", -ret);
		} else {
			/* Else we proceed to call encode_frame()
			 * without an input frame to flush the
			 * encoder */
			while ((ret = x264_encoder_delayed_frames(self->x264)) >
			       0) {
				ret = encode_frame(self, NULL);
				if (ret < 0) {
					if (ret != -EAGAIN)
						VENC_LOG_ERRNO("encode_frame",
							       -ret);
					break;
				}
			}
		}
	}
}


static void input_event_cb(struct pomp_evt *evt, void *userdata)
{
	struct venc_x264 *self = userdata;
	check_input_queue(self);
}


static void *encoder_thread(void *ptr)
{
	int ret, timeout;
	struct venc_x264 *self = ptr;
	struct pomp_loop *loop = NULL;
	struct pomp_evt *in_queue_evt = NULL;
	char message;

	ret = pthread_setname_np(pthread_self(), "venc_x264");
	if (ret != 0)
		VENC_LOG_ERRNO("pthread_setname_np", ret);

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


static int get_supported_input_formats(const struct vdef_raw_format **formats)
{
	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);
	*formats = supported_formats;
	return NB_SUPPORTED_FORMATS;
}


static int copy_implem_cfg(const struct venc_config_impl *impl_cfg,
			   struct venc_config_impl **ret_obj)
{
	struct venc_config_x264 *specific = (struct venc_config_x264 *)impl_cfg;
	struct venc_config_x264 *copy = NULL;
	ULOG_ERRNO_RETURN_ERR_IF(specific == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	*ret_obj = NULL;

	copy = calloc(1, sizeof(*copy));
	ULOG_ERRNO_RETURN_ERR_IF(copy == NULL, ENOMEM);

	/* Deep copy */
	*copy = *specific;
	if (copy->preset)
		copy->preset = strdup(specific->preset);
	if (copy->tune)
		copy->tune = strdup(specific->tune);

	*ret_obj = (struct venc_config_impl *)copy;

	return 0;
}


static int free_implem_cfg(struct venc_config_impl *impl_cfg)
{
	struct venc_config_x264 *specific = (struct venc_config_x264 *)impl_cfg;
	ULOG_ERRNO_RETURN_ERR_IF(specific == NULL, EINVAL);

	free((void *)specific->preset);
	free((void *)specific->tune);
	free((void *)specific);

	return 0;
}


static int flush(struct venc_encoder *base, int discard_)
{
	struct venc_x264 *self;
	bool discard = discard_;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	atomic_store(&self->flushing, true);
	atomic_store(&self->flush_discard, discard);

	return 0;
}


static int stop(struct venc_encoder *base)
{
	struct venc_x264 *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	/* Stop the encoding thread */
	atomic_store(&self->should_stop, true);

	return 0;
}


static int destroy(struct venc_encoder *base)
{
	int err;
	struct venc_x264 *self;

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
	if (self->enc_in_queue != NULL) {
		err = mbuf_raw_video_frame_queue_destroy(self->enc_in_queue);
		if (err < 0)
			VENC_LOG_ERRNO(
				"mbuf_raw_video_frame_queue_destroy:enc_in",
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
	if (self->mbox != NULL) {
		err = pomp_loop_remove(base->loop,
				       mbox_get_read_fd(self->mbox));
		if (err < 0)
			VENC_LOG_ERRNO("pomp_loop_remove", -err);
		mbox_destroy(self->mbox);
	}
	if (self->h264_reader) {
		err = h264_reader_destroy(self->h264_reader);
		if (err < 0)
			VENC_LOG_ERRNO("h264_reader_destroy", -err);
	}
	if (self->x264 != NULL)
		x264_encoder_close(self->x264);
	free(self->dummy_uv_plane);

	err = pomp_loop_idle_remove_by_cookie(base->loop, self);
	if (err < 0)
		VENC_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	free(self);
	base->derived = NULL;

	return 0;
}


static bool input_filter(struct mbuf_raw_video_frame *frame, void *userdata)
{
	int ret;
	const void *tmp;
	size_t tmplen;
	struct vdef_raw_frame info;
	struct venc_x264 *self = userdata;

	VENC_LOG_ERRNO_RETURN_ERR_IF(self == NULL, false);

	if (atomic_load(&self->flushing) || atomic_load(&self->should_stop))
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

static void sei_recovery_point_cb(struct h264_ctx *ctx,
				  const uint8_t *buf,
				  size_t len,
				  const struct h264_sei_recovery_point *sei,
				  void *userdata)
{
	struct venc_x264 *self = userdata;

	self->base->recovery_frame_cnt = sei->recovery_frame_cnt;
	self->recovery_point = true;
}


static const struct h264_ctx_cbs h264_cbs = {
	.sei_recovery_point = &sei_recovery_point_cb,
};


static int create(struct venc_encoder *base)
{
	int ret = 0;
	struct venc_x264 *self = NULL;
	x264_param_t x264_params;
	struct venc_config_x264 *specific;
	const char *preset;
	const char *tune;
	struct vdef_raw_format *fmt;
	struct mbuf_raw_video_frame_queue_args queue_args = {
		.filter = input_filter,
	};
	unsigned int min_qp = FUTILS_MAX(VENC_X264_MIN_QP, 1);
	unsigned int max_qp = FUTILS_MIN(VENC_X264_MAX_QP, 51);

	VENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	/* Check the configuration */
	if (base->config.encoding != VDEF_ENCODING_H264) {
		ret = -EINVAL;
		VENC_LOG_ERRNO("invalid encoding: %s",
			       -ret,
			       vdef_encoding_to_str(base->config.encoding));
		return ret;
	}
	if ((base->config.output.preferred_format !=
	     VDEF_CODED_DATA_FORMAT_UNKNOWN) &&
	    (base->config.output.preferred_format !=
	     VDEF_CODED_DATA_FORMAT_BYTE_STREAM) &&
	    (base->config.output.preferred_format !=
	     VDEF_CODED_DATA_FORMAT_AVCC)) {
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

	specific = (struct venc_config_x264 *)venc_config_get_specific(
		&base->config, VENC_ENCODER_IMPLEM_X264);

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->base = base;
	base->derived = self;
	self->output_format = (struct vdef_coded_format){
		.encoding = base->config.encoding,
		.data_format = base->config.output.preferred_format,
	};
	if (self->output_format.data_format == VDEF_CODED_DATA_FORMAT_UNKNOWN)
		self->output_format.data_format =
			VDEF_CODED_DATA_FORMAT_BYTE_STREAM;

	queue_args.filter_userdata = self;

	ret = h264_reader_new(&h264_cbs, self, &self->h264_reader);
	if (ret < 0) {
		VENC_LOG_ERRNO("h264_reader_new", -ret);
		goto error;
	}

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

	/* Generate the parameters */
	preset =
		(specific && specific->preset) ? specific->preset : "superfast";
	tune = (specific && specific->tune) ? specific->tune : "psnr";
	ret = x264_param_default_preset(&x264_params, preset, tune);
	if (ret < 0) {
		VENC_LOG_ERRNO("x264_param_default_preset", -ret);
		goto error;
	}

	/* TODO */
	x264_params.pf_log = x264_log_cb;
	x264_params.b_pic_struct = !!base->config.h264.insert_pic_timing_sei;
	x264_params.i_width = base->config.input.info.resolution.width;
	x264_params.i_height = base->config.input.info.resolution.height;
	x264_params.i_fps_num = base->config.input.info.framerate.num;
	x264_params.i_fps_den = base->config.input.info.framerate.den;
	x264_params.i_level_idc = base->config.h264.level;
	x264_params.i_bframe = 0;
	x264_params.i_keyint_max =
		(int)(base->config.h264.gop_length_sec *
			      base->config.input.info.framerate.num /
			      base->config.input.info.framerate.den +
		      0.5);
	x264_params.analyse.i_weighted_pred = 0;
	x264_params.analyse.b_weighted_bipred = 0;
	x264_params.analyse.b_psnr = 1;
	x264_params.b_repeat_headers = 0;
	x264_params.b_annexb = 0; /* Always output AVCC */
	x264_params.rc.i_lookahead = 0;
	x264_params.rc.b_mb_tree = 0;
	x264_params.i_slice_max_mbs =
		base->config.h264.slice_size_mbrows * base->mb_width;
	/* The HRD in H.264 was not designed with VFR in mind.
	 * It is therefore not recommendeded to use NAL HRD with VFR.
	 * Furthermore, reconfiguring the VBV (via x264_encoder_reconfig) will
	 * currently generate invalid HRD. */
	x264_params.i_nal_hrd = X264_NAL_HRD_NONE;
	switch (base->config.h264.rate_control) {
	default:
	case VENC_RATE_CONTROL_CBR:
		x264_params.rc.i_rc_method = X264_RC_ABR;
		x264_params.rc.i_bitrate =
			(base->config.h264.max_bitrate + 500) / 1000;
		x264_params.rc.i_vbv_max_bitrate = x264_params.rc.i_bitrate;
		x264_params.rc.i_vbv_buffer_size = x264_params.rc.i_bitrate;
		break;
	case VENC_RATE_CONTROL_VBR:
		x264_params.rc.i_rc_method = X264_RC_ABR;
		x264_params.rc.i_bitrate =
			(base->config.h264.max_bitrate + 500) / 1000;
		break;
	case VENC_RATE_CONTROL_CQ:
		x264_params.rc.i_rc_method = X264_RC_CQP;
		x264_params.rc.i_qp_constant = base->config.h264.qp;
		break;
	}

	/* X264 encoder can output coded frames of size > (YUV size /2). Enforce
	 * the min_qp >= 10 to avoid this. */
	if (base->config.h264.min_qp >= 1 && base->config.h264.min_qp <= 51)
		min_qp = FUTILS_MAX(min_qp, base->config.h264.min_qp);

	if (base->config.h264.max_qp >= 1 && base->config.h264.max_qp <= 51)
		max_qp = FUTILS_MAX(max_qp, base->config.h264.max_qp);

	x264_params.rc.i_qp_min = min_qp;
	x264_params.rc.i_qp_max = max_qp;

	x264_params.b_intra_refresh =
		(base->config.h264.intra_refresh == VENC_INTRA_REFRESH_NONE)
			? 0
			: 1;

	x264_param_apply_fastfirstpass(&x264_params);

	ret = x264_param_apply_profile(&x264_params, "main");
	if (ret < 0) {
		VENC_LOG_ERRNO("x264_param_apply_profile", -ret);
		goto error;
	}

	/* Initialize the encoder */
	self->x264 = x264_encoder_open(&x264_params);
	if (self->x264 == NULL) {
		VENC_LOGE("x264_encoder_open failed");
		ret = -EPROTO;
		goto error;
	}

	VENC_LOGI("x264 implementation - build %d", X264_BUILD);
	VENC_LOGI("preset=%s tune=%s", preset, tune);

	x264_picture_init(&self->in_picture);

	fmt = &base->config.input.format;
	if (vdef_raw_format_cmp(fmt, &vdef_i420)) {
		self->in_picture.img.i_csp = X264_CSP_I420;
		self->in_picture.img.i_plane = 3;
		self->in_picture.img.i_stride[0] =
			base->config.input.info.resolution.width;
		self->in_picture.img.i_stride[1] =
			base->config.input.info.resolution.width / 2;
		self->in_picture.img.i_stride[2] =
			base->config.input.info.resolution.width / 2;
		self->in_picture.img.i_stride[3] = 0;
	} else if (vdef_raw_format_cmp(fmt, &vdef_nv21)) {
		self->in_picture.img.i_csp = X264_CSP_NV21;
		self->in_picture.img.i_plane = 2;
		self->in_picture.img.i_stride[0] =
			base->config.input.info.resolution.width;
		self->in_picture.img.i_stride[1] =
			base->config.input.info.resolution.width;
		self->in_picture.img.i_stride[2] = 0;
		self->in_picture.img.i_stride[3] = 0;
	} else if (vdef_raw_format_cmp(fmt, &vdef_nv12)) {
		self->in_picture.img.i_csp = X264_CSP_NV12;
		self->in_picture.img.i_plane = 2;
		self->in_picture.img.i_stride[0] =
			base->config.input.info.resolution.width;
		self->in_picture.img.i_stride[1] =
			base->config.input.info.resolution.width;
		self->in_picture.img.i_stride[2] = 0;
		self->in_picture.img.i_stride[3] = 0;
	} else if (vdef_raw_format_cmp(fmt, &vdef_gray)) {
		/* x264 does not support grayscale input, create a dummy
		 * UV plane and send it with every frame to "forge" NV12
		 */
		self->in_picture.img.i_csp = X264_CSP_NV12;
		self->in_picture.img.i_plane = 2;
		self->in_picture.img.i_stride[0] =
			base->config.input.info.resolution.width;
		self->in_picture.img.i_stride[1] =
			base->config.input.info.resolution.width;
		self->in_picture.img.i_stride[2] = 0;
		self->in_picture.img.i_stride[3] = 0;

		self->dummy_uv_plane_stride =
			base->config.input.info.resolution.width;
		self->dummy_uv_plane_len =
			base->config.input.info.resolution.width *
			base->config.input.info.resolution.height / 2;
		self->dummy_uv_plane = malloc(self->dummy_uv_plane_len);
		if (!self->dummy_uv_plane) {
			ret = -ENOMEM;
			goto error;
		}
		memset(self->dummy_uv_plane, 0x80, self->dummy_uv_plane_len);
	}

	self->in_picture.img.plane[0] = NULL;
	self->in_picture.img.plane[1] = NULL;
	self->in_picture.img.plane[2] = NULL;
	self->in_picture.img.plane[3] = NULL;

	/* Generate the SPS and PPS */
	ret = generate_sps_pps(self);
	if (ret < 0) {
		VENC_LOG_ERRNO("generate_sps_pps", -ret);
		goto error;
	}

	/* Initialize the H.264 writer */
	ret = venc_h264_writer_new(base->h264.sps,
				   base->h264.sps_size,
				   base->h264.pps,
				   base->h264.pps_size,
				   &base->h264.ctx);
	if (ret < 0) {
		VENC_LOG_ERRNO("venc_h264_writer_new", -ret);
		goto error;
	}

	/* Patch the PS */
	ret = venc_h264_patch_ps(base);
	if (ret < 0) {
		VENC_LOG_ERRNO("venc_h264_patch_ps", -ret);
		goto error;
	}

	/* Create the input buffers queue */
	ret = mbuf_raw_video_frame_queue_new_with_args(&queue_args,
						       &self->in_queue);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_new_with_args:input",
			       -ret);
		goto error;
	}

	/* Create the encoder input buffers queue */
	ret = mbuf_raw_video_frame_queue_new(&self->enc_in_queue);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_new:enc_in", -ret);
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
	struct venc_x264 *self;

	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	self = (struct venc_x264 *)base->derived;

	return self->in_queue;
}


static int get_dyn_config(struct venc_encoder *base,
			  struct venc_dyn_config *config)
{
	config->qp = base->config.h264.qp;
	config->target_bitrate = base->config.h264.target_bitrate;
	config->decimation = base->config.h264.decimation;

	return 0;
}


static int set_dyn_config(struct venc_encoder *base,
			  const struct venc_dyn_config *config)
{
	int ret, update_config = 0;
	x264_param_t x264_params;
	struct venc_x264 *self;

	self = base->derived;

	memset(&x264_params, 0, sizeof(x264_params));
	x264_encoder_parameters(self->x264, &x264_params);

	if (base->config.h264.rate_control == VENC_RATE_CONTROL_CQ &&
	    config->qp != 0 && base->config.h264.qp != config->qp) {
		base->config.h264.qp = config->qp;
		x264_params.rc.i_qp_constant = base->config.h264.qp;
		update_config = 1;
	}

	if (config->target_bitrate != 0 &&
	    base->config.h264.target_bitrate != config->target_bitrate &&
	    base->config.h264.rate_control != VENC_RATE_CONTROL_CQ) {
		base->config.h264.target_bitrate = config->target_bitrate;
		update_config = 1;
		x264_params.rc.i_bitrate =
			(base->config.h264.target_bitrate + 500) / 1000;
		x264_params.rc.i_vbv_max_bitrate = x264_params.rc.i_bitrate;
		x264_params.rc.i_vbv_buffer_size = x264_params.rc.i_bitrate;
	}

	if (config->decimation != 0 &&
	    base->config.h264.decimation != config->decimation)
		base->config.h264.decimation = config->decimation;

	/* Update encoder parameters only if configuration changed */
	if (update_config) {
		ret = x264_encoder_reconfig(self->x264, &x264_params);
		if (ret < 0) {
			VENC_LOG_ERRNO("x264_encoder_reconfig", -ret);
			return ret;
		}
	}

	return 0;
}


static int request_idr(struct venc_encoder *base)
{
	struct venc_x264 *self = base->derived;

	atomic_store(&self->insert_idr, true);

	return 0;
}


const struct venc_ops venc_x264_ops = {
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
	.request_idr = request_idr,
};
