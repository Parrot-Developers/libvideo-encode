/**
 * Copyright (c) 2021 Parrot Drones SAS
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

#define ULOG_TAG venc_x265
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "venc_x265_priv.h"


#define VENC_X265_MIN_QP 1
#define VENC_X265_MAX_QP 51


#define NB_SUPPORTED_ENCODINGS 1
#define NB_SUPPORTED_FORMATS 3
static struct vdef_raw_format supported_formats[NB_SUPPORTED_FORMATS];
static enum vdef_encoding supported_encodings[NB_SUPPORTED_ENCODINGS];
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;
static void initialize_supported_formats(void)
{
	supported_formats[0] = vdef_i420;
	supported_formats[1] = vdef_gray;
	supported_formats[2] = vdef_i420_10_16le;

	supported_encodings[0] = VDEF_ENCODING_H265;
}


static void call_flush_done(void *userdata)
{
	struct venc_x265 *self = userdata;

	venc_call_flush_cb(self->base);
}


static void call_stop_done(void *userdata)
{
	struct venc_x265 *self = userdata;

	venc_call_stop_cb(self->base);
}


static void mbox_cb(int fd, uint32_t revents, void *userdata)
{
	struct venc_x265 *self = userdata;
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
	struct venc_x265 *self = userdata;
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

		if (!atomic_load(&self->flush_discard))
			venc_call_frame_output_cb(self->base, 0, out_frame);
		else
			VENC_LOGD("discarding frame %d", out_info.info.index);

		mbuf_coded_video_frame_unref(out_frame);
	} while (err == 0);
}


static int generate_ps(struct venc_x265 *self)
{
	int ret;
	uint32_t i, nalu_count = 0;
	x265_nal *nalu = NULL;
	size_t len;

	ret = self->api->encoder_headers(self->x265, &nalu, &nalu_count);
	if (ret < 0) {
		VENC_LOG_ERRNO("x265_encoder_headers", -ret);
		return ret;
	}
	VENC_LOG_ERRNO_RETURN_ERR_IF(nalu_count < 3, ENODATA);

	xfree((void **)&self->base->h265.vps);
	self->base->h265.vps_size = 0;
	xfree((void **)&self->base->h265.sps);
	self->base->h265.sps_size = 0;
	xfree((void **)&self->base->h265.pps);
	self->base->h265.pps_size = 0;

	for (i = 0; i < nalu_count; i++) {
		if ((nalu[i].type == NAL_UNIT_VPS) && (!self->base->h265.vps)) {
			len = nalu[i].sizeBytes;
			VENC_LOG_ERRNO_RETURN_ERR_IF(len <= 4, ENODATA);
			self->base->h265.vps = malloc(len - 4);
			VENC_LOG_ERRNO_RETURN_ERR_IF(
				self->base->h265.vps == NULL, ENOMEM);
			memcpy(self->base->h265.vps,
			       nalu[i].payload + 4,
			       len - 4);
			self->base->h265.vps_size = len - 4;
		} else if ((nalu[i].type == NAL_UNIT_SPS) &&
			   (!self->base->h265.sps)) {
			len = nalu[i].sizeBytes;
			VENC_LOG_ERRNO_RETURN_ERR_IF(len <= 4, ENODATA);
			self->base->h265.sps = malloc(len - 4);
			VENC_LOG_ERRNO_RETURN_ERR_IF(
				self->base->h265.sps == NULL, ENOMEM);
			memcpy(self->base->h265.sps,
			       nalu[i].payload + 4,
			       len - 4);
			self->base->h265.sps_size = len - 4;
		} else if ((nalu[i].type == NAL_UNIT_PPS) &&
			   (!self->base->h265.pps)) {
			len = nalu[i].sizeBytes;
			VENC_LOG_ERRNO_RETURN_ERR_IF(len <= 4, ENODATA);
			self->base->h265.pps = malloc(len - 4);
			VENC_LOG_ERRNO_RETURN_ERR_IF(
				self->base->h265.pps == NULL, ENOMEM);
			memcpy(self->base->h265.pps,
			       nalu[i].payload + 4,
			       len - 4);
			self->base->h265.pps_size = len - 4;
		}
	}

	VENC_LOG_ERRNO_RETURN_ERR_IF(self->base->h265.vps == NULL, ENODATA);
	VENC_LOG_ERRNO_RETURN_ERR_IF(self->base->h265.vps_size == 0, ENODATA);
	VENC_LOG_ERRNO_RETURN_ERR_IF(self->base->h265.sps == NULL, ENODATA);
	VENC_LOG_ERRNO_RETURN_ERR_IF(self->base->h265.sps_size == 0, ENODATA);
	VENC_LOG_ERRNO_RETURN_ERR_IF(self->base->h265.pps == NULL, ENODATA);
	VENC_LOG_ERRNO_RETURN_ERR_IF(self->base->h265.pps_size == 0, ENODATA);

	ret = h265_reader_parse_nalu(self->h265_reader,
				     0,
				     self->base->h265.vps,
				     self->base->h265.vps_size);
	if (ret < 0) {
		VENC_LOG_ERRNO("h265_reader_parse", -ret);
		return ret;
	}

	ret = h265_reader_parse_nalu(self->h265_reader,
				     0,
				     self->base->h265.sps,
				     self->base->h265.sps_size);
	if (ret < 0) {
		VENC_LOG_ERRNO("h265_reader_parse", -ret);
		return ret;
	}

	ret = h265_reader_parse_nalu(self->h265_reader,
				     0,
				     self->base->h265.pps,
				     self->base->h265.pps_size);
	if (ret < 0) {
		VENC_LOG_ERRNO("h265_reader_parse", -ret);
		return ret;
	}

	return 0;
}


static enum vdef_coded_frame_type x265_to_vdef_frame_type(int frame_type)
{
	if (frame_type == X265_TYPE_IDR)
		return VDEF_CODED_FRAME_TYPE_IDR;
	else if (frame_type == X265_TYPE_I)
		return VDEF_CODED_FRAME_TYPE_I;
	else if (frame_type == X265_TYPE_P)
		return VDEF_CODED_FRAME_TYPE_P;
	else
		return VDEF_CODED_FRAME_TYPE_NOT_CODED;
}


static int add_x265_nalus(struct venc_x265 *self,
			  struct mbuf_coded_video_frame *out_frame,
			  struct vdef_coded_frame *out_info,
			  x265_nal *nalu,
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
		mem_size += nalu[i].sizeBytes;

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
		/* Discard VPS, SPS, PPS and SEI NAL units (we have already
		 * inserted these NAL units generated ourselves) */
		if ((nalu[i].type == NAL_UNIT_VPS) ||
		    (nalu[i].type == NAL_UNIT_SPS) ||
		    (nalu[i].type == NAL_UNIT_PPS) ||
		    (nalu[i].type == NAL_UNIT_PREFIX_SEI))
			continue;

		base_offset = offset;
		out_nalu.size = 0;
		out_nalu.importance = 0;
		nalu_len = nalu[i].sizeBytes - nalu_offset;
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
		memcpy(data + offset, nalu[i].payload + nalu_offset, nalu_len);
		out_nalu.size += nalu_len;
		out_nalu.h265.type = (enum h265_nalu_type)nalu[i].type;
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


static int fill_frame(struct venc_x265 *self,
		      struct mbuf_raw_video_frame *in_frame,
		      struct mbuf_coded_video_frame *out_frame,
		      struct vdef_coded_frame *out_info,
		      x265_picture *out_picture,
		      x265_nal *nalu,
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
	ret = venc_h265_generate_nalus(self->base, out_frame, out_info);
	if (ret < 0) {
		VENC_LOG_ERRNO("venc_h265_generate_nalus", -ret);
		goto out;
	}

	/* Add x265 NAL units */
	ret = add_x265_nalus(self, out_frame, out_info, nalu, nalu_count);

out:
	if (metadata)
		vmeta_frame_unref(metadata);
	return ret;
}


/* Can be called from any thread */
static void frame_release(struct mbuf_coded_video_frame *frame, void *userdata)
{
	struct venc_x265 *self = userdata;

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


static int encode_frame(struct venc_x265 *self,
			struct mbuf_raw_video_frame *in_frame)
{
	int ret, err;
	uint32_t nalu_count = 0;
	int frame_size = 0;
	x265_nal *nalu = NULL;
	x265_picture *out_picture = NULL;
	struct vdef_raw_frame info;
	const void *plane_data;
	const uint8_t *in_data;
	size_t len, nalu_len = 0, nalu_offset = 4;
	ssize_t i;
	struct mbuf_raw_video_frame *enc_frame = NULL;
	struct mbuf_coded_video_frame *out_frame = NULL;
	struct vdef_coded_frame out_info = {};
	struct vdef_raw_frame in_info;
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;
	unsigned int plane_count = 0;

	struct mbuf_coded_video_frame_cbs frame_cbs = {
		.pre_release = frame_release,
		.pre_release_userdata = (void *)self,
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
			    self->base->config.h265.decimation !=
		    0) {
			self->x265_pts++;
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
			self->in_picture.planes[1] =
				(uint8_t *)self->dummy_uv_plane;
			self->in_picture.stride[1] =
				self->dummy_uv_plane_stride;
			self->in_picture.planes[2] =
				(uint8_t *)self->dummy_uv_plane;
			self->in_picture.stride[2] =
				self->dummy_uv_plane_stride;
		} else if (vdef_raw_format_cmp(&info.format, &vdef_i420) ||
			   vdef_raw_format_cmp(&info.format,
					       &vdef_i420_10_16le)) {
			plane_count = 3;
		}

		for (unsigned int i = 0; i < plane_count; i++) {
			ret = mbuf_raw_video_frame_get_plane(
				in_frame, i, &plane_data, &len);
			if (ret == 0) {
				in_data = plane_data;
				self->in_picture.planes[i] = (uint8_t *)in_data;
				self->in_picture.stride[i] =
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

		self->in_picture.pts = self->x265_pts;
		self->in_picture.userData = in_frame;
		self->in_picture.sliceType = X265_TYPE_AUTO;
		if (atomic_load(&self->insert_idr)) {
			self->in_picture.sliceType = X265_TYPE_IDR;
			atomic_store(&self->insert_idr, false);
		}

		self->input_frame_cnt++;
		self->x265_pts++;
	}

	out_picture = calloc(1, sizeof(*out_picture));
	if (out_picture == NULL) {
		ret = -ENOMEM;
		VENC_LOG_ERRNO("calloc", -ret);
		return ret;
	}

	if (in_frame != NULL)
		self->base->counters.pushed++;
	else
		atomic_store(&self->flush_sent, true);
	ret = self->api->encoder_encode(self->x265,
					&nalu,
					&nalu_count,
					in_frame ? &self->in_picture : NULL,
					out_picture);
	if (ret < 0) {
		VENC_LOG_ERRNO("x265_encoder_encode", -ret);
		goto out;
	}
	frame_size = ret;

	/* Push the frame in the encoder input queue if provided*/
	if (in_frame) {
		ret = mbuf_raw_video_frame_queue_push(self->enc_in_queue,
						      in_frame);
		if (ret < 0) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_push:enc_in",
				       -ret);
			goto out;
		}
	}

	/* Return if no frame was output */
	if (out_picture->userData == NULL) {
		ret = 0;
		goto out;
	}

	self->base->counters.pulled++;

	/* Find the frame (non-blocking);
	 * drop frames until the frame corresponding to the
	 * x265 output is found */
	enc_frame = NULL;
	do {
		if (enc_frame != NULL) {
			mbuf_raw_video_frame_unref(enc_frame);
			enc_frame = NULL;
		}
		ret = mbuf_raw_video_frame_queue_pop(self->enc_in_queue,
						     &enc_frame);
		if (ret == -EAGAIN) {
			ret = 0;
			goto out;
		} else if (ret < 0) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_pop:enc_in",
				       -ret);
			goto out;
		}
	} while (enc_frame != out_picture->userData);

	ret = mbuf_raw_video_frame_get_frame_info(enc_frame, &in_info);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		goto out;
	}

	out_info.info = in_info.info;
	out_info.format = self->output_format;
	out_info.type = x265_to_vdef_frame_type(out_picture->sliceType);
	out_info.layer = 0;

	if (frame_size == 0)
		out_info.type = VDEF_CODED_FRAME_TYPE_NOT_CODED;

	self->recovery_point = false;
	for (i = 0; i < nalu_count; i++) {
		if ((*(nalu[i].payload + nalu_offset) & 0x3F) !=
		    H265_NALU_TYPE_PREFIX_SEI_NUT) {
			continue;
		}
		nalu_len = nalu[i].sizeBytes - nalu_offset;

		ret = h265_reader_parse_nalu(self->h265_reader,
					     0,
					     nalu[i].payload + nalu_offset,
					     nalu_len);
		if (ret < 0) {
			VENC_LOG_ERRNO("h265_reader_parse_nalu", -ret);
			goto out;
		}
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
			 out_picture,
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
	free(out_picture);

	return (ret >= 0) ? frame_size : ret;
}


static int complete_flush(struct venc_x265 *self)
{
	int ret;

	if (atomic_load(&self->flush_discard)) {
		do {
			/* To flush the encoder and retrieve delayed output
			 * pictures, pass pic_in as NULL */
			ret = encode_frame(self, NULL);
		} while (ret > 0);

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
	atomic_store(&self->flush_sent, false);

	/* Call the flush callback on the loop */
	char message = VENC_MSG_FLUSH;
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		VENC_LOG_ERRNO("mbox_push", -ret);

	return ret;
}


static void check_input_queue(struct venc_x265 *self)
{
	int ret, err = 0;
	int frame_size = 0;
	struct mbuf_raw_video_frame *in_frame;

	ret = mbuf_raw_video_frame_queue_peek(self->in_queue, &in_frame);
	while (ret >= 0) {
		/* Push the input frame */
		/* Encode the frame */
		ret = encode_frame(self, in_frame);
		if (ret < 0) {
			if (ret != -EAGAIN)
				VENC_LOG_ERRNO("encode_frame", -ret);
			err = -ENOSPC;
		} else {
			frame_size = ret;
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
				if (frame_size == 0 &&
				    atomic_load(&self->flush_sent)) {
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

	if ((ret == -EAGAIN) && (atomic_load(&self->flushing)) &&
	    (!atomic_load(&self->flush_discard))) {
		/* Else we proceed to call encode_frame()
		 * without an input frame to flush the
		 * encoder */
		while (true) {
			ret = encode_frame(self, NULL);
			if (ret < 0) {
				if (ret != -EAGAIN)
					VENC_LOG_ERRNO("encode_frame", -ret);
				break;
			}
			if (ret == 0 && atomic_load(&self->flush_sent)) {
				ret = complete_flush(self);
				if (ret < 0)
					VENC_LOG_ERRNO("complete_flush", -ret);
				break;
			}
		}
	}
}


static void input_event_cb(struct pomp_evt *evt, void *userdata)
{
	struct venc_x265 *self = userdata;
	check_input_queue(self);
}


static void *encoder_thread(void *ptr)
{
	int ret, timeout;
	struct venc_x265 *self = ptr;
	struct pomp_loop *loop = NULL;
	struct pomp_evt *in_queue_evt = NULL;
	char message;

	ret = pthread_setname_np(pthread_self(), "venc_x265");
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
	struct venc_config_x265 *specific = (struct venc_config_x265 *)impl_cfg;
	struct venc_config_x265 *copy = NULL;
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
	struct venc_config_x265 *specific = (struct venc_config_x265 *)impl_cfg;
	ULOG_ERRNO_RETURN_ERR_IF(specific == NULL, EINVAL);

	free((void *)specific->preset);
	free((void *)specific->tune);
	free((void *)specific);

	return 0;
}


static int flush(struct venc_encoder *base, int discard_)
{
	struct venc_x265 *self;
	bool discard = discard_;

	ULOG_ERRNO_RETURN_ERR_IF(base->derived == NULL, EINVAL);

	self = base->derived;

	atomic_store(&self->flushing, true);
	atomic_store(&self->flush_discard, discard);

	return 0;
}


static int stop(struct venc_encoder *base)
{
	struct venc_x265 *self;

	ULOG_ERRNO_RETURN_ERR_IF(base->derived == NULL, EINVAL);

	self = base->derived;

	/* Stop the encoding thread */
	atomic_store(&self->should_stop, true);

	return 0;
}


static int destroy(struct venc_encoder *base)
{
	int err;
	struct venc_x265 *self;

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
	if (self->h265_reader) {
		err = h265_reader_destroy(self->h265_reader);
		if (err < 0)
			VENC_LOG_ERRNO("h265_reader_destroy", -err);
	}
	if (self->x265 != NULL && self->api != NULL)
		self->api->encoder_close(self->x265);
	if (self->api != NULL)
		self->api->cleanup();
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
	struct venc_x265 *self = userdata;

	VENC_LOG_ERRNO_RETURN_ERR_IF(self == NULL, false);

	if (atomic_load(&self->should_stop))
		return false;

	if (atomic_load(&self->flushing))
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

static void sei_recovery_point_cb(struct h265_ctx *ctx,
				  const uint8_t *buf,
				  size_t len,
				  const struct h265_sei_recovery_point *sei,
				  void *userdata)
{
	struct venc_x265 *self = userdata;

	self->base->recovery_frame_cnt = sei->recovery_poc_cnt;
	self->recovery_point = true;
}


static const struct h265_ctx_cbs h265_cbs = {
	.sei_recovery_point = &sei_recovery_point_cb,
};


static char *bit_depth_to_profile(int bit_depth)
{
	switch (bit_depth) {
	case 0:
		/* If bitDepth is 0 the function is guarunteed
		 * to return a non-NULL x265_api pointer, from the linked
		 * libx265. */
	case 8:
		return "main";
	case 10:
		return "main10";
	default:
		return "unsupported";
	}
}


static int create(struct venc_encoder *base)
{
	int ret = 0;
	struct venc_x265 *self = NULL;
	x265_param *x265_params = NULL;
	struct venc_config_x265 *specific;
	const char *preset;
	const char *tune;
	struct vdef_raw_format *fmt;
	struct mbuf_raw_video_frame_queue_args queue_args = {
		.filter = input_filter,
	};
	unsigned int min_qp = VENC_X265_MIN_QP;
	unsigned int max_qp = VENC_X265_MAX_QP;

	VENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	/* Check the configuration */
	if (base->config.encoding != VDEF_ENCODING_H265) {
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

	specific = (struct venc_config_x265 *)venc_config_get_specific(
		&base->config, VENC_ENCODER_IMPLEM_X265);

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

	atomic_init(&self->flushing, false);
	atomic_init(&self->flush_discard, false);
	atomic_init(&self->flush_sent, false);

	ret = h265_reader_new(&h265_cbs, self, &self->h265_reader);
	if (ret < 0) {
		VENC_LOG_ERRNO("h265_reader_new", -ret);
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

	/* Get the API */
	self->api = x265_api_get(base->config.input.info.bit_depth);
	if (self->api == NULL) {
		ret = -EINVAL;
		VENC_LOG_ERRNO("no x265 API available for %d bits",
			       -ret,
			       base->config.input.info.bit_depth);
		goto error;
	}

	x265_params = calloc(1, sizeof(*x265_params));
	if (x265_params == NULL) {
		ret = -ENOMEM;
		VENC_LOG_ERRNO("calloc", -ret);
		goto error;
	}

	/* Generate the parameters */
	preset =
		(specific && specific->preset) ? specific->preset : "superfast";
	tune = (specific && specific->tune) ? specific->tune : "psnr";
	ret = self->api->param_default_preset(x265_params, preset, tune);
	if (ret < 0) {
		VENC_LOG_ERRNO("x265_param_default_preset", -ret);
		goto error;
	}

	/* Enforce profile >= 5.1 for 4K encoding. See x265 level constraints */
	if (base->config.h265.level < VENC_X265_LEVEL_5_1 &&
	    base->config.input.info.resolution.width > 1920 &&
	    base->config.input.info.resolution.height > 1080)
		base->config.h265.level = VENC_X265_LEVEL_5_1;

	/* TODO */
	x265_params->sourceWidth = base->config.input.info.resolution.width;
	x265_params->sourceHeight = base->config.input.info.resolution.height;
	x265_params->fpsNum = base->config.input.info.framerate.num;
	x265_params->fpsDenom = base->config.input.info.framerate.den;
	x265_params->levelIdc = base->config.h265.level;
	x265_params->bframes = 0;
	x265_params->sourceBitDepth = base->config.input.info.bit_depth;
	x265_params->keyframeMax =
		(int)(base->config.h265.gop_length_sec *
			      base->config.input.info.framerate.num /
			      base->config.input.info.framerate.den +
		      0.5);
	x265_params->bEnableWeightedPred = 0;
	x265_params->bEnableWeightedBiPred = 0;
	x265_params->bEnablePsnr = 1;
	x265_params->bRepeatHeaders = 0;
	x265_params->bAnnexB = 0;
	x265_params->bEmitIDRRecoverySEI =
		base->config.h265.insert_recovery_point_sei;
	switch (base->config.h265.rate_control) {
	default:
	case VENC_RATE_CONTROL_CBR:
		x265_params->rc.rateControlMode = X265_RC_ABR;
		x265_params->rc.bitrate =
			(base->config.h265.max_bitrate + 500) / 1000;
		x265_params->rc.vbvMaxBitrate = x265_params->rc.bitrate;
		x265_params->rc.vbvBufferSize = x265_params->rc.bitrate;
		break;
	case VENC_RATE_CONTROL_VBR:
		x265_params->rc.rateControlMode = X265_RC_ABR;
		x265_params->rc.bitrate =
			(base->config.h265.max_bitrate + 500) / 1000;
		break;
	case VENC_RATE_CONTROL_CQ:
		x265_params->rc.rateControlMode = X265_RC_CQP;
		x265_params->rc.qp = base->config.h265.qp;
		break;
	}

	if (base->config.h265.min_qp != 0 || base->config.h265.max_qp != 0) {
		min_qp = FUTILS_MAX(min_qp, base->config.h265.min_qp);
		max_qp = FUTILS_MIN(max_qp, base->config.h265.max_qp);
	}

	x265_params->rc.qpMin = min_qp;
	x265_params->rc.qpMax = max_qp;

	ret = self->api->param_apply_profile(
		x265_params,
		bit_depth_to_profile(base->config.input.info.bit_depth));
	if (ret < 0) {
		VENC_LOG_ERRNO("x265_param_apply_profile", -ret);
		goto error;
	}

	/* Initialize the encoder */
	self->x265 = self->api->encoder_open(x265_params);
	if (self->x265 == NULL) {
		VENC_LOGE("x265_encoder_open failed");
		ret = -EPROTO;
		goto error;
	}

	VENC_LOGI("x265 implementation - build %d", X265_BUILD);
	VENC_LOGI("preset=%s tune=%s", preset, tune);

	self->api->picture_init(x265_params, &self->in_picture);

	fmt = &base->config.input.format;
	self->in_picture.colorSpace = X265_CSP_I420;
	self->in_picture.stride[0] = base->config.input.info.resolution.width;
	self->in_picture.stride[1] =
		base->config.input.info.resolution.width / 2;
	self->in_picture.stride[2] =
		base->config.input.info.resolution.width / 2;

	if (vdef_raw_format_cmp(fmt, &vdef_gray)) {
		/* x265 does not support grayscale input, create a dummy
		 * UV plane and send it with every frame to "forge" I420
		 */
		self->dummy_uv_plane_stride =
			base->config.input.info.resolution.width / 2;
		self->dummy_uv_plane_len =
			base->config.input.info.resolution.width / 2 *
			base->config.input.info.resolution.height / 2;
		self->dummy_uv_plane = malloc(self->dummy_uv_plane_len);
		if (!self->dummy_uv_plane) {
			ret = -ENOMEM;
			goto error;
		}
		memset(self->dummy_uv_plane, 0x80, self->dummy_uv_plane_len);
	}

	self->in_picture.planes[0] = NULL;
	self->in_picture.planes[1] = NULL;
	self->in_picture.planes[2] = NULL;

	/* Generate the VPS, SPS and PPS */
	ret = generate_ps(self);
	if (ret < 0) {
		VENC_LOG_ERRNO("generate_ps", -ret);
		goto error;
	}

	/* Initialize the H.265 writer */
	ret = venc_h265_writer_new(base->h265.vps,
				   base->h265.vps_size,
				   base->h265.sps,
				   base->h265.sps_size,
				   base->h265.pps,
				   base->h265.pps_size,
				   &base->h265.ctx);
	if (ret < 0) {
		VENC_LOG_ERRNO("venc_h265_writer_new", -ret);
		goto error;
	}

	/* Patch the PS */
	ret = venc_h265_patch_ps(base);
	if (ret < 0) {
		VENC_LOG_ERRNO("venc_h265_patch_ps", -ret);
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

	free(x265_params);

	return 0;

error:
	free(x265_params);

	/* Cleanup on error */
	destroy(base);
	base->derived = NULL;
	return ret;
}


static struct mbuf_pool *get_input_buffer_pool(struct venc_encoder *base)
{
	/* No input buffer pool allocated: use the application's */
	return NULL;
}


static struct mbuf_raw_video_frame_queue *
get_input_buffer_queue(struct venc_encoder *base)
{
	struct venc_x265 *self;

	ULOG_ERRNO_RETURN_VAL_IF(base->derived == NULL, EINVAL, NULL);

	self = (struct venc_x265 *)base->derived;

	return self->in_queue;
}


static int get_dyn_config(struct venc_encoder *base,
			  struct venc_dyn_config *config)
{
	config->qp = base->config.h265.qp;
	config->target_bitrate = base->config.h265.target_bitrate;
	config->decimation = base->config.h265.decimation;

	return 0;
}


static int set_dyn_config(struct venc_encoder *base,
			  const struct venc_dyn_config *config)
{
	int ret, update_config = 0;
	x265_param *x265_params = NULL;
	struct venc_x265 *self;

	self = base->derived;

	VENC_LOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	x265_params = calloc(1, sizeof(*x265_params));
	if (x265_params == NULL) {
		ret = -ENOMEM;
		VENC_LOG_ERRNO("calloc", -ret);
		return ret;
	}

	self->api->encoder_parameters(self->x265, x265_params);

	if (base->config.h265.rate_control == VENC_RATE_CONTROL_CQ &&
	    config->qp != 0 && base->config.h265.qp != config->qp) {
		base->config.h265.qp = config->qp;
		x265_params->rc.qp = base->config.h265.qp;
		update_config = 1;
	}

	if (config->target_bitrate != 0 &&
	    base->config.h265.target_bitrate != config->target_bitrate &&
	    base->config.h265.rate_control != VENC_RATE_CONTROL_CQ) {
		base->config.h265.target_bitrate = config->target_bitrate;
		update_config = 1;
		x265_params->rc.bitrate =
			(base->config.h265.target_bitrate + 500) / 1000;
	}

	if (config->decimation != 0 &&
	    base->config.h265.decimation != config->decimation)
		base->config.h265.decimation = config->decimation;

	/* Update encoder parameters only if configuration changed */
	if (update_config) {
		ret = self->api->encoder_reconfig(self->x265, x265_params);
		if (ret < 0) {
			VENC_LOG_ERRNO("x265_encoder_reconfig", -ret);
			goto out;
		}
	}

	ret = 0;

out:
	free(x265_params);
	return ret;
}


static int request_idr(struct venc_encoder *base)
{
	struct venc_x265 *self = base->derived;

	VENC_LOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	atomic_store(&self->insert_idr, true);

	return 0;
}


const struct venc_ops venc_x265_ops = {
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
