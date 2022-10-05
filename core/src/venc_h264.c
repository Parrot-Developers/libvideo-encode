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

#define ULOG_TAG venc_core
#include <ulog.h>

#include "venc_core_priv.h"


int venc_h264_writer_new(const uint8_t *sps,
			 size_t sps_size,
			 const uint8_t *pps,
			 size_t pps_size,
			 struct h264_ctx **ret_obj)
{
	int res;
	struct h264_ctx *h264 = NULL;
	struct h264_sps *_sps = NULL;
	struct h264_pps *_pps = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(sps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps_size == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps_size == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	_sps = calloc(1, sizeof(*_sps));
	if (_sps == NULL) {
		res = -ENOMEM;
		goto error;
	}
	_pps = calloc(1, sizeof(*_pps));
	if (_pps == NULL) {
		res = -ENOMEM;
		goto error;
	}

	res = h264_ctx_new(&h264);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_new", -res);
		goto error;
	}

	res = h264_parse_sps(sps, sps_size, _sps);
	if (res < 0) {
		ULOG_ERRNO("h264_parse_sps", -res);
		goto error;
	}

	res = h264_parse_pps(pps, pps_size, _sps, _pps);
	if (res < 0) {
		ULOG_ERRNO("h264_parse_pps", -res);
		goto error;
	}

	res = h264_ctx_set_sps(h264, _sps);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_set_sps", -res);
		goto error;
	}

	res = h264_ctx_set_pps(h264, _pps);
	if (res != 0) {
		ULOG_ERRNO("h264_ctx_set_pps", -res);
		goto error;
	}

	*ret_obj = h264;
	free(_sps);
	free(_pps);
	return 0;

error:
	venc_h264_writer_destroy(h264);
	free(_sps);
	free(_pps);
	return res;
}


int venc_h264_writer_destroy(struct h264_ctx *h264)
{
	int res;

	if (h264 == NULL)
		return 0;

	res = h264_ctx_destroy(h264);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_destroy", -res);
		return res;
	}

	return 0;
}


int venc_h264_aud_write(struct h264_ctx *h264,
			struct mbuf_coded_video_frame *frame)
{
	int res = 0, err;
	struct vdef_nalu nalu;
	size_t size, len;
	uint8_t *data, *start;
	void *void_data;
	struct mbuf_mem *mem = NULL;
	struct h264_nalu_header nh;
	struct vdef_coded_frame info;

	ULOG_ERRNO_RETURN_ERR_IF(h264 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);

	res = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		return res;
	}

	res = h264_ctx_clear_nalu(h264);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_clear_nalu", -res);
		return res;
	}

	struct h264_aud aud;
	aud.primary_pic_type = ((info.type == VDEF_CODED_FRAME_TYPE_IDR) ||
				(info.type == VDEF_CODED_FRAME_TYPE_I))
				       ? 0
				       : 1;
	/* TODO: define primary_pic_type values in libh264 */
	res = h264_ctx_set_aud(h264, &aud);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_set_aud", -res);
		return res;
	}

	nh.forbidden_zero_bit = 0;
	nh.nal_ref_idc = 0;
	nh.nal_unit_type = H264_NALU_TYPE_AUD;
	res = h264_ctx_set_nalu_header(h264, &nh);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_set_nalu_header", -res);
		return res;
	}

	struct h264_bitstream bs;
	memset(&bs, 0, sizeof(bs));
	len = 256;
	res = mbuf_mem_generic_new(len, &mem);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_generic_new", -res);
		goto out;
	}
	res = mbuf_mem_get_data(mem, &void_data, &len);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_get_data", -res);
		goto out;
	}
	start = data = void_data;
	size = 0;
	if (info.format.data_format == VDEF_CODED_DATA_FORMAT_BYTE_STREAM) {
		if (len <= 4) {
			res = -ENOBUFS;
			ULOG_ERRNO("", -res);
			goto out;
		}
		data[0] = data[1] = data[2] = 0;
		data[3] = 1;
		size += 4;
		data += size;
		len -= size;
	} else if (info.format.data_format == VDEF_CODED_DATA_FORMAT_AVCC) {
		if (len <= 4) {
			res = -ENOBUFS;
			ULOG_ERRNO("", -res);
			goto out;
		}
		size += 4;
		data += size;
		len -= size;
	}
	h264_bs_init(&bs, data, len, 1);
	res = h264_write_nalu(&bs, h264);
	if (res < 0) {
		ULOG_ERRNO("h264_write_nalu", -res);
		goto out;
	}

	if (info.format.data_format == VDEF_CODED_DATA_FORMAT_AVCC) {
		uint32_t sz = htonl(bs.off);
		memcpy(start, &sz, 4);
	}

	size += bs.off;
	nalu.h264.type = H264_NALU_TYPE_AUD;
	nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;
	nalu.h264.slice_mb_count = 0;
	nalu.size = size;

	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_add_nalu", -res);
		goto out;
	}

out:
	err = mbuf_mem_unref(mem);
	if (err != 0)
		ULOG_ERRNO("mbuf_mem_unref", -err);

	return res;
}


int venc_h264_sps_pps_copy(struct h264_ctx *h264,
			   struct mbuf_coded_video_frame *frame,
			   const uint8_t *sps,
			   size_t sps_size,
			   const uint8_t *pps,
			   size_t pps_size)
{
	int res = 0, err;
	uint32_t sz, start_code = htonl(0x00000001);
	struct vdef_nalu sps_nalu, pps_nalu;
	size_t size, len;
	uint8_t *sps_data, *pps_data;
	void *void_data;
	struct mbuf_mem *sps_mem = NULL, *pps_mem = NULL;
	struct vdef_coded_frame info;

	ULOG_ERRNO_RETURN_ERR_IF(h264 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps_size == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps_size == 0, EINVAL);

	res = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		return res;
	}

	/* SPS */
	len = sps_size + sizeof(start_code);
	res = mbuf_mem_generic_new(len, &sps_mem);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_generic_new", -res);
		goto out;
	}
	res = mbuf_mem_get_data(sps_mem, &void_data, &len);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_get_data", -res);
		goto out;
	}
	sps_data = void_data;
	size = 0;
	if (info.format.data_format == VDEF_CODED_DATA_FORMAT_BYTE_STREAM) {
		if (len <= 4) {
			ULOG_ERRNO("", ENOBUFS);
			return -ENOBUFS;
		}
		memcpy(sps_data, &start_code, sizeof(uint32_t));
		size += 4;
		sps_data += size;
		len -= size;
	} else if (info.format.data_format == VDEF_CODED_DATA_FORMAT_AVCC) {
		if (len <= 4) {
			ULOG_ERRNO("", ENOBUFS);
			return -ENOBUFS;
		}
		sz = htonl(sps_size);
		memcpy(sps_data, &sz, sizeof(uint32_t));
		size += 4;
		sps_data += size;
		len -= size;
	}
	if (sps_size > len) {
		ULOG_ERRNO("", ENOBUFS);
		return -ENOBUFS;
	}
	memcpy(sps_data, sps, sps_size);
	size += sps_size;
	sps_nalu.h264.type = H264_NALU_TYPE_SPS;
	sps_nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;
	sps_nalu.h264.slice_mb_count = 0;
	sps_nalu.size = size;

	res = mbuf_coded_video_frame_add_nalu(frame, sps_mem, 0, &sps_nalu);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_add_nalu", -res);
		goto out;
	}

	/* PPS */
	len = pps_size + sizeof(start_code);
	res = mbuf_mem_generic_new(len, &pps_mem);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_generic_new", -res);
		goto out;
	}
	res = mbuf_mem_get_data(pps_mem, &void_data, &len);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_get_data", -res);
		goto out;
	}
	pps_data = void_data;
	size = 0;
	if (info.format.data_format == VDEF_CODED_DATA_FORMAT_BYTE_STREAM) {
		if (len <= 4) {
			ULOG_ERRNO("", ENOBUFS);
			return -ENOBUFS;
		}
		memcpy(pps_data, &start_code, sizeof(uint32_t));
		size += 4;
		pps_data += size;
		len -= size;
	} else if (info.format.data_format == VDEF_CODED_DATA_FORMAT_AVCC) {
		if (len <= 4) {
			ULOG_ERRNO("", ENOBUFS);
			return -ENOBUFS;
		}
		sz = htonl(pps_size);
		memcpy(pps_data, &sz, sizeof(uint32_t));
		size += 4;
		pps_data += size;
		len -= size;
	}
	if (pps_size > len) {
		ULOG_ERRNO("", ENOBUFS);
		return -ENOBUFS;
	}
	memcpy(pps_data, pps, pps_size);
	size += pps_size;
	pps_nalu.h264.type = H264_NALU_TYPE_PPS;
	pps_nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;
	pps_nalu.h264.slice_mb_count = 0;
	pps_nalu.size = size;

	res = mbuf_coded_video_frame_add_nalu(frame, pps_mem, 0, &pps_nalu);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_add_nalu", -res);
		goto out;
	}

out:
	err = mbuf_mem_unref(sps_mem);
	if (err != 0)
		ULOG_ERRNO("mbuf_mem_unref", -err);
	err = mbuf_mem_unref(pps_mem);
	if (err != 0)
		ULOG_ERRNO("mbuf_mem_unref", -err);
	return res;
}


int venc_h264_sei_reset(struct h264_ctx *h264)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(h264 == NULL, EINVAL);

	res = h264_ctx_clear_nalu(h264);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_clear_nalu", -res);
		return res;
	}

	return 0;
}


int venc_h264_sei_add_recovery_point(struct h264_ctx *h264,
				     unsigned int recovery_frame_cnt)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(h264 == NULL, EINVAL);

	struct h264_sei sei = {0};
	sei.type = H264_SEI_TYPE_RECOVERY_POINT;
	sei.recovery_point.recovery_frame_cnt = recovery_frame_cnt;
	sei.recovery_point.exact_match_flag = 1;
	sei.recovery_point.broken_link_flag = 0;
	sei.recovery_point.changing_slice_group_idc = 0;

	res = h264_ctx_add_sei(h264, &sei);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_add_sei", -res);
		return res;
	}

	return 0;
}


int venc_h264_sei_add_picture_timing(struct h264_ctx *h264, uint64_t timestamp)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(h264 == NULL, EINVAL);

	const struct h264_sps *sps = h264_ctx_get_sps(h264);
	if (!sps->vui_parameters_present_flag)
		return -ENOENT;

	if (!sps->vui.nal_hrd_parameters_present_flag &&
	    !sps->vui.vcl_hrd_parameters_present_flag &&
	    !sps->vui.pic_struct_present_flag)
		return -ENOENT;

	if ((!sps->vui.timing_info_present_flag) ||
	    (sps->vui.time_scale == 0) || (sps->vui.num_units_in_tick == 0))
		return -EPROTO;

	uint32_t time_scale = sps->vui.time_scale;
	uint32_t nuit = sps->vui.num_units_in_tick;
	uint64_t clock_timestamp = (timestamp * time_scale + 500000) / 1000000;

	uint32_t hours_value =
		(uint32_t)(clock_timestamp / time_scale / (60 * 60));
	uint32_t minutes_value =
		(uint32_t)((clock_timestamp -
			    (uint64_t)hours_value * 60 * 60 * time_scale) /
			   time_scale / 60);
	uint32_t seconds_value =
		(uint32_t)((clock_timestamp -
			    (uint64_t)hours_value * 60 * 60 * time_scale -
			    (uint64_t)minutes_value * 60 * time_scale) /
			   time_scale);
	uint32_t n_frames =
		(uint32_t)((clock_timestamp -
			    (uint64_t)hours_value * 60 * 60 * time_scale -
			    (uint64_t)minutes_value * 60 * time_scale -
			    (uint64_t)seconds_value * time_scale) /
			   nuit);
	int32_t time_offset = (int32_t)(
		(int64_t)clock_timestamp -
		(int64_t)hours_value * 60 * 60 * time_scale -
		(int64_t)minutes_value * 60 * time_scale -
		(int64_t)seconds_value * time_scale - (int64_t)n_frames * nuit);

	struct h264_sei sei = {0};
	sei.type = H264_SEI_TYPE_PIC_TIMING;
	sei.pic_timing.cpb_removal_delay = 0;
	sei.pic_timing.dpb_output_delay = 0;
	sei.pic_timing.pic_struct = 0;
	sei.pic_timing.clk_ts[0].clock_timestamp_flag = 1;
	sei.pic_timing.clk_ts[0].ct_type = 1;
	sei.pic_timing.clk_ts[0].nuit_field_based_flag = 0;
	sei.pic_timing.clk_ts[0].counting_type = 1;
	sei.pic_timing.clk_ts[0].full_timestamp_flag = 1;
	sei.pic_timing.clk_ts[0].discontinuity_flag = 0;
	sei.pic_timing.clk_ts[0].cnt_dropped_flag = 0;
	sei.pic_timing.clk_ts[0].n_frames = n_frames;
	sei.pic_timing.clk_ts[0].seconds_value = seconds_value;
	sei.pic_timing.clk_ts[0].minutes_value = minutes_value;
	sei.pic_timing.clk_ts[0].hours_value = hours_value;
	sei.pic_timing.clk_ts[0].time_offset = time_offset;
	sei.pic_timing.clk_ts[1].clock_timestamp_flag = 0;
	sei.pic_timing.clk_ts[2].clock_timestamp_flag = 0;

	res = h264_ctx_add_sei(h264, &sei);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_add_sei", -res);
		return res;
	}

	return 0;
}


int venc_h264_sei_add_parrot_streaming_v2_user_data(struct h264_ctx *h264,
						    unsigned int slice_count,
						    unsigned int slice_mb_count)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(h264 == NULL, EINVAL);

	struct vstrm_h264_sei_streaming_v2 strm = {
		.slice_count = slice_count,
		.slice_mb_count = slice_mb_count,
	};
	size_t len = vstrm_h264_sei_streaming_v2_get_size(&strm);
	uint8_t *data = malloc(len);
	if (data == NULL) {
		ULOG_ERRNO("malloc", ENOMEM);
		return -ENOMEM;
	}

	struct h264_sei sei = {0};
	sei.type = H264_SEI_TYPE_USER_DATA_UNREGISTERED;
	sei.user_data_unregistered.buf = data;

	res = vstrm_h264_sei_streaming_v2_write(
		&strm, sei.user_data_unregistered.uuid, data, &len);
	if (res < 0) {
		ULOG_ERRNO("vstrm_h264_sei_streaming_v2_write", -res);
		goto out;
	}
	sei.user_data_unregistered.len = len;

	res = h264_ctx_add_sei(h264, &sei);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_add_sei", -res);
		goto out;
	}

out:
	free(data);

	return res;
}


int venc_h264_sei_add_parrot_streaming_v4_user_data(
	struct h264_ctx *h264,
	unsigned int slice_mb_count,
	unsigned int slice_mb_count_recovery_point)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(h264 == NULL, EINVAL);

	struct vstrm_h264_sei_streaming_v4 strm = {
		.slice_mb_count = slice_mb_count,
		.slice_mb_count_recovery_point = slice_mb_count_recovery_point,
	};
	size_t len = vstrm_h264_sei_streaming_v4_get_size(&strm);
	uint8_t *data = malloc(len);
	if (data == NULL) {
		ULOG_ERRNO("malloc", ENOMEM);
		return -ENOMEM;
	}

	struct h264_sei sei = {0};
	sei.type = H264_SEI_TYPE_USER_DATA_UNREGISTERED;
	sei.user_data_unregistered.buf = data;

	res = vstrm_h264_sei_streaming_v4_write(
		&strm, sei.user_data_unregistered.uuid, data, &len);
	if (res < 0) {
		ULOG_ERRNO("vstrm_h264_sei_streaming_v4_write", -res);
		goto out;
	}
	sei.user_data_unregistered.len = len;

	res = h264_ctx_add_sei(h264, &sei);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_add_sei", -res);
		goto out;
	}

out:
	free(data);

	return res;
}


int venc_h264_sei_add_user_data(struct h264_ctx *h264,
				const uint8_t *data,
				size_t len)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(h264 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(data == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len < 16, EINVAL);

	struct h264_sei sei = {0};
	sei.type = H264_SEI_TYPE_USER_DATA_UNREGISTERED;
	sei.user_data_unregistered.buf = data + 16;
	sei.user_data_unregistered.len = len - 16;
	memcpy(sei.user_data_unregistered.uuid,
	       data,
	       sizeof(sei.user_data_unregistered.uuid));

	res = h264_ctx_add_sei(h264, &sei);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_add_sei", -res);
		return res;
	}

	return 0;
}


int venc_h264_sei_write(struct h264_ctx *h264,
			struct mbuf_coded_video_frame *frame)
{
	int res, count, err;
	struct vdef_nalu nalu;
	struct mbuf_mem *mem = NULL;
	void *void_data;
	uint8_t *data, *start;
	struct h264_bitstream bs;
	size_t len, size;
	struct vdef_coded_frame info;

	ULOG_ERRNO_RETURN_ERR_IF(h264 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);

	res = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		return res;
	}

	count = h264_ctx_get_sei_count(h264);
	if (count < 0) {
		ULOG_ERRNO("h264_ctx_get_sei_count", -count);
		return count;
	}
	if (count == 0)
		return 0;

	struct h264_nalu_header nh;
	nh.forbidden_zero_bit = 0;
	nh.nal_ref_idc = 0;
	nh.nal_unit_type = H264_NALU_TYPE_SEI;
	res = h264_ctx_set_nalu_header(h264, &nh);
	if (res < 0) {
		ULOG_ERRNO("h264_ctx_set_nalu_header", -res);
		return res;
	}

	memset(&bs, 0, sizeof(bs));
	len = 1024;
	res = mbuf_mem_generic_new(len, &mem);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_generic_new", -res);
		goto out;
	}
	res = mbuf_mem_get_data(mem, &void_data, &len);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_get_data", -res);
		goto out;
	}
	start = data = void_data;
	size = 0;
	if (info.format.data_format == VDEF_CODED_DATA_FORMAT_BYTE_STREAM) {
		if (len <= 4) {
			ULOG_ERRNO("", ENOBUFS);
			return -ENOBUFS;
		}
		data[0] = data[1] = data[2] = 0;
		data[3] = 1;
		size += 4;
		data += size;
		len -= size;
	} else if (info.format.data_format == VDEF_CODED_DATA_FORMAT_AVCC) {
		if (len <= 4) {
			ULOG_ERRNO("", ENOBUFS);
			return -ENOBUFS;
		}
		size += 4;
		data += size;
		len -= size;
	}
	h264_bs_init(&bs, data, len, 1);
	res = h264_write_nalu(&bs, h264);
	if (res < 0) {
		ULOG_ERRNO("h264_write_nalu", -res);
		return res;
	}

	if (info.format.data_format == VDEF_CODED_DATA_FORMAT_AVCC) {
		uint32_t sz = htonl(bs.off);
		memcpy(start, &sz, 4);
	}
	size += bs.off;
	nalu.h264.type = H264_NALU_TYPE_SEI;
	nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;
	nalu.h264.slice_mb_count = 0;
	nalu.size = size;

	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_add_nalu", -res);
		goto out;
	}

out:
	err = mbuf_mem_unref(mem);
	if (err != 0)
		ULOG_ERRNO("mbuf_mem_unref", -err);

	return res;
}


int venc_h264_generate_nalus(struct venc_encoder *self,
			     struct mbuf_coded_video_frame *frame,
			     const struct vdef_coded_frame *info)
{
	int ret = 0, sei_count = 0;

	/* AUD insertion */
	if (self->config.h264.insert_aud) {
		ret = venc_h264_aud_write(self->h264.ctx, frame);
		if (ret < 0)
			goto out;
	}

	/* SPS/PPS insertion */
	if ((self->config.h264.insert_ps) && (self->h264.sps != NULL) &&
	    (self->h264.sps_size > 0) && (self->h264.pps != NULL) &&
	    (self->h264.pps_size > 0) &&
	    ((info->type == VDEF_CODED_FRAME_TYPE_IDR) ||
	     (info->type == VDEF_CODED_FRAME_TYPE_I) ||
	     (info->type == VDEF_CODED_FRAME_TYPE_P_IR_START))) {
		ret = venc_h264_sps_pps_copy(self->h264.ctx,
					     frame,
					     self->h264.sps,
					     self->h264.sps_size,
					     self->h264.pps,
					     self->h264.pps_size);
		if (ret < 0)
			goto out;
	}

	/* Generate SEI */
	ret = venc_h264_sei_reset(self->h264.ctx);
	if (ret < 0)
		goto out;

	if ((self->config.h264.insert_recovery_point_sei) &&
	    (info->type == VDEF_CODED_FRAME_TYPE_P_IR_START)) {
		/* Recovery point sei */
		ret = venc_h264_sei_add_recovery_point(
			self->h264.ctx, self->recovery_frame_cnt);
		if (ret < 0)
			goto out;
		sei_count++;
	}
	if ((self->config.h264.insert_pic_timing_sei) &&
	    (info->info.capture_timestamp != 0)) {
		/* Picture timing sei */
		ret = venc_h264_sei_add_picture_timing(
			self->h264.ctx, info->info.capture_timestamp);
		if (ret < 0)
			goto out;
		sei_count++;
	}
	if (self->config.h264.streaming_user_data_sei_version &&
	    ((info->type == VDEF_CODED_FRAME_TYPE_IDR) ||
	     (info->type == VDEF_CODED_FRAME_TYPE_I) ||
	     (info->type == VDEF_CODED_FRAME_TYPE_P_IR_START))) {
		switch (self->config.h264.streaming_user_data_sei_version) {
		case 2:
			/* "Parrot Streaming" v2 user data sei */
			ret = venc_h264_sei_add_parrot_streaming_v2_user_data(
				self->h264.ctx,
				self->slice_count,
				self->slice_mb_count);
			break;
		case 4:
			/* "Parrot Streaming" v4 user data sei */
			ret = venc_h264_sei_add_parrot_streaming_v4_user_data(
				self->h264.ctx,
				self->slice_mb_count,
				self->slice_mb_count_recovery_point);
			break;
		default:
			ret = -ENOSYS;
			ULOG_ERRNO("unsupported streaming_user_data format: %d",
				   -ret,
				   self->config.h264
					   .streaming_user_data_sei_version);
			break;
		}
		if (ret < 0)
			goto out;
		sei_count++;
	}
	if (self->config.h264.serialize_user_data) {
		struct mbuf_ancillary_data *data;
		const void *raw_data;
		size_t len;

		/* User data sei */
		ret = mbuf_coded_video_frame_get_ancillary_data(
			frame, MBUF_ANCILLARY_KEY_USERDATA_SEI, &data);
		if (ret < 0)
			goto out;
		raw_data = mbuf_ancillary_data_get_buffer(data, &len);
		if ((raw_data != NULL) && (len >= 16)) {
			ret = venc_h264_sei_add_user_data(
				self->h264.ctx, raw_data, len);
			if (ret < 0)
				goto out;
			sei_count++;
		}
		mbuf_ancillary_data_unref(data);
	}
	ret = venc_h264_sei_write(self->h264.ctx, frame);
	if (ret < 0)
		goto out;

out:
	return ret;
}


int venc_h264_format_convert(struct mbuf_coded_video_frame *frame,
			     const struct vdef_coded_format *target_format)
{
	int res = 0;
	uint8_t *data;
	uint32_t start_code = htonl(0x00000001);
	const void *nalu_data;
	struct vdef_nalu nalu;
	int nalu_count;
	struct vdef_coded_frame info;
	const struct vdef_coded_format *current_format;

	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(target_format == NULL, EINVAL);

	res = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		return res;
	}
	current_format = &info.format;

	/* No conversion needed */
	if (vdef_coded_format_cmp(current_format, target_format))
		return 0;

	/* Only support byte stream to AVCC or AVCC to byte stream */
	if ((current_format->data_format == VDEF_CODED_DATA_FORMAT_RAW_NALU) ||
	    (target_format->data_format == VDEF_CODED_DATA_FORMAT_RAW_NALU)) {
		res = -ENOSYS;
		ULOG_ERRNO("format", -res);
		return res;
	}

	nalu_count = mbuf_coded_video_frame_get_nalu_count(frame);
	if (nalu_count < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_nalu_count",
			   -nalu_count);
		return nalu_count;
	}

	for (int i = 0; i < nalu_count; i++) {
		res = mbuf_coded_video_frame_get_nalu(
			frame, i, &nalu_data, &nalu);
		if (res < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_get_nalu", -res);
			return res;
		}
		data = (uint8_t *)nalu_data;

		if ((current_format->data_format ==
		     VDEF_CODED_DATA_FORMAT_AVCC) &&
		    (target_format->data_format ==
		     VDEF_CODED_DATA_FORMAT_BYTE_STREAM)) {
			/* AVCC to byte stream */
			memcpy(data, &start_code, sizeof(uint32_t));
		} else if ((current_format->data_format ==
			    VDEF_CODED_DATA_FORMAT_BYTE_STREAM) &&
			   (target_format->data_format ==
			    VDEF_CODED_DATA_FORMAT_AVCC)) {
			/* Byte stream to AVCC */
			memcpy(data, &nalu.size, sizeof(uint32_t));
		} else {
			res = -ENOSYS;
			ULOG_ERRNO("format", -res);
			return res;
		}

		res = mbuf_coded_video_frame_release_nalu(frame, i, nalu_data);
		if (res < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_release_nalu", -res);
			return res;
		}
	}

	return 0;
}
