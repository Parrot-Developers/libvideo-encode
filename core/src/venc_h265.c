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


int venc_h265_writer_new(const uint8_t *vps,
			 size_t vps_size,
			 const uint8_t *sps,
			 size_t sps_size,
			 const uint8_t *pps,
			 size_t pps_size,
			 struct h265_ctx **ret_obj)
{
	int res;
	struct h265_ctx *h265 = NULL;
	struct h265_vps *_vps = NULL;
	struct h265_sps *_sps = NULL;
	struct h265_pps *_pps = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(vps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(vps_size == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps_size == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps_size == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	_vps = calloc(1, sizeof(*_vps));
	if (_vps == NULL) {
		res = -ENOMEM;
		goto error;
	}
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

	res = h265_ctx_new(&h265);
	if (res < 0) {
		ULOG_ERRNO("h265_ctx_new", -res);
		goto error;
	}

	res = h265_parse_vps(vps, vps_size, _vps);
	if (res < 0) {
		ULOG_ERRNO("h265_parse_vps", -res);
		goto error;
	}

	res = h265_parse_sps(sps, sps_size, _sps);
	if (res < 0) {
		ULOG_ERRNO("h265_parse_sps", -res);
		goto error;
	}

	res = h265_parse_pps(pps, pps_size, _pps);
	if (res < 0) {
		ULOG_ERRNO("h265_parse_pps", -res);
		goto error;
	}

	res = h265_ctx_set_vps(h265, _vps);
	if (res < 0) {
		ULOG_ERRNO("h265_ctx_set_vps", -res);
		goto error;
	}

	res = h265_ctx_set_sps(h265, _sps);
	if (res < 0) {
		ULOG_ERRNO("h265_ctx_set_sps", -res);
		goto error;
	}

	res = h265_ctx_set_pps(h265, _pps);
	if (res != 0) {
		ULOG_ERRNO("h265_ctx_set_pps", -res);
		goto error;
	}

	*ret_obj = h265;
	free(_vps);
	free(_sps);
	free(_pps);
	return 0;

error:
	venc_h265_writer_destroy(h265);
	free(_vps);
	free(_sps);
	free(_pps);
	return res;
}


int venc_h265_writer_destroy(struct h265_ctx *h265)
{
	int res;

	if (h265 == NULL)
		return 0;

	res = h265_ctx_destroy(h265);
	if (res < 0) {
		ULOG_ERRNO("h265_ctx_destroy", -res);
		return res;
	}

	return 0;
}


int venc_h265_aud_write(struct h265_ctx *h265,
			struct mbuf_coded_video_frame *frame)
{
	int res = 0, err;
	struct vdef_nalu nalu;
	size_t size, len;
	uint8_t *data, *start;
	void *void_data;
	struct mbuf_mem *mem = NULL;
	struct h265_nalu_header nh;
	struct vdef_coded_frame info;

	ULOG_ERRNO_RETURN_ERR_IF(h265 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);

	res = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		return res;
	}

	res = h265_ctx_clear_nalu(h265);
	if (res < 0) {
		ULOG_ERRNO("h265_ctx_clear_nalu", -res);
		return res;
	}

	struct h265_aud aud;
	aud.pic_type = ((info.type == VDEF_CODED_FRAME_TYPE_IDR) ||
			(info.type == VDEF_CODED_FRAME_TYPE_I))
			       ? 0
			       : 1;
	/* TODO: define pic_type values in libh265 */
	res = h265_ctx_set_aud(h265, &aud);
	if (res < 0) {
		ULOG_ERRNO("h265_ctx_set_aud", -res);
		return res;
	}

	nh.forbidden_zero_bit = 0;
	nh.nal_unit_type = H265_NALU_TYPE_AUD_NUT;
	nh.nuh_layer_id = 0;
	nh.nuh_temporal_id_plus1 = 1;
	res = h265_ctx_set_nalu_header(h265, &nh);
	if (res < 0) {
		ULOG_ERRNO("h265_ctx_set_nalu_header", -res);
		return res;
	}

	struct h265_bitstream bs;
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
	h265_bs_init(&bs, data, len, 1);
	res = h265_write_nalu(&bs, h265);
	if (res < 0) {
		ULOG_ERRNO("h265_write_nalu", -res);
		goto out;
	}

	if (info.format.data_format == VDEF_CODED_DATA_FORMAT_AVCC) {
		uint32_t sz = htonl(bs.off);
		memcpy(start, &sz, 4);
	}

	size += bs.off;
	nalu.h265.type = H265_NALU_TYPE_AUD_NUT;
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


int venc_h265_ps_copy(struct h265_ctx *h265,
		      struct mbuf_coded_video_frame *frame,
		      const uint8_t *vps,
		      size_t vps_size,
		      const uint8_t *sps,
		      size_t sps_size,
		      const uint8_t *pps,
		      size_t pps_size)
{
	int res = 0, err;
	uint32_t sz, start_code = htonl(0x00000001);
	struct vdef_nalu vps_nalu, sps_nalu, pps_nalu;
	size_t size, len;
	uint8_t *vps_data, *sps_data, *pps_data;
	void *void_data;
	struct mbuf_mem *vps_mem = NULL, *sps_mem = NULL, *pps_mem = NULL;
	struct vdef_coded_frame info;

	ULOG_ERRNO_RETURN_ERR_IF(h265 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(vps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(vps_size == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps_size == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps_size == 0, EINVAL);

	res = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		return res;
	}

	/* VPS */
	len = vps_size + sizeof(start_code);
	res = mbuf_mem_generic_new(len, &vps_mem);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_generic_new", -res);
		goto out;
	}
	res = mbuf_mem_get_data(vps_mem, &void_data, &len);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_get_data", -res);
		goto out;
	}
	vps_data = (uint8_t *)void_data;
	size = 0;
	if (info.format.data_format == VDEF_CODED_DATA_FORMAT_BYTE_STREAM) {
		if (len <= 4) {
			ULOG_ERRNO("", ENOBUFS);
			return -ENOBUFS;
		}
		memcpy(vps_data, &start_code, sizeof(uint32_t));
		size += 4;
		vps_data += size;
		len -= size;
	} else if (info.format.data_format == VDEF_CODED_DATA_FORMAT_AVCC) {
		if (len <= 4) {
			ULOG_ERRNO("", ENOBUFS);
			return -ENOBUFS;
		}
		sz = htonl(vps_size);
		memcpy(vps_data, &sz, sizeof(uint32_t));
		size += 4;
		vps_data += size;
		len -= size;
	}
	if (vps_size > len) {
		ULOG_ERRNO("", ENOBUFS);
		return -ENOBUFS;
	}
	memcpy(vps_data, vps, vps_size);
	size += vps_size;
	vps_nalu.h265.type = H265_NALU_TYPE_VPS_NUT;
	vps_nalu.size = size;

	res = mbuf_coded_video_frame_add_nalu(frame, vps_mem, 0, &vps_nalu);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_add_nalu", -res);
		goto out;
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
	sps_data = (uint8_t *)void_data;
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
	sps_nalu.h265.type = H265_NALU_TYPE_SPS_NUT;
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
	pps_data = (uint8_t *)void_data;
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
	pps_nalu.h265.type = H265_NALU_TYPE_PPS_NUT;
	pps_nalu.size = size;

	res = mbuf_coded_video_frame_add_nalu(frame, pps_mem, 0, &pps_nalu);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_add_nalu", -res);
		goto out;
	}

out:
	err = mbuf_mem_unref(vps_mem);
	if (err != 0)
		ULOG_ERRNO("mbuf_mem_unref", -err);
	err = mbuf_mem_unref(sps_mem);
	if (err != 0)
		ULOG_ERRNO("mbuf_mem_unref", -err);
	err = mbuf_mem_unref(pps_mem);
	if (err != 0)
		ULOG_ERRNO("mbuf_mem_unref", -err);
	return res;
}


int venc_h265_sei_reset(struct h265_ctx *h265)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(h265 == NULL, EINVAL);

	res = h265_ctx_clear_nalu(h265);
	if (res < 0) {
		ULOG_ERRNO("h265_ctx_clear_nalu", -res);
		return res;
	}

	return 0;
}


int venc_h265_sei_add_recovery_point(struct h265_ctx *h265,
				     unsigned int recovery_poc_cnt)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(h265 == NULL, EINVAL);

	struct h265_sei sei = {0};
	sei.type = H265_SEI_TYPE_RECOVERY_POINT;
	sei.recovery_point.recovery_poc_cnt = recovery_poc_cnt;
	sei.recovery_point.exact_match_flag = 1;
	sei.recovery_point.broken_link_flag = 0;

	res = h265_ctx_add_sei(h265, &sei);
	if (res < 0) {
		ULOG_ERRNO("h265_ctx_add_sei", -res);
		return res;
	}

	return 0;
}


int venc_h265_sei_add_time_code(struct h265_ctx *h265, uint64_t timestamp)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(h265 == NULL, EINVAL);

	const struct h265_sps *sps = h265_ctx_get_sps(h265);

	if ((sps->vui.vui_time_scale == 0) ||
	    (sps->vui.vui_num_units_in_tick == 0))
		return -EPROTO;

	uint32_t time_scale = sps->vui.vui_time_scale;
	uint32_t nuit = sps->vui.vui_num_units_in_tick;
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
	uint32_t time_offset_abs =
		(time_offset >= 0) ? time_offset : -time_offset;
	uint8_t time_offset_len = 31;
	while ((time_offset_abs >> time_offset_len) && (time_offset_len > 0))
		time_offset_len--;
	if (time_offset_len < 31)
		time_offset_len++;

	struct h265_sei sei = {0};
	sei.type = H265_SEI_TYPE_TIME_CODE;
	sei.time_code.num_clock_ts = 1;
	sei.time_code.clock_ts[0].clock_timestamp_flag = 1;
	sei.time_code.clock_ts[0].units_field_based_flag = 0;
	sei.time_code.clock_ts[0].counting_type = 1;
	sei.time_code.clock_ts[0].full_timestamp_flag = 1;
	sei.time_code.clock_ts[0].discontinuity_flag = 0;
	sei.time_code.clock_ts[0].cnt_dropped_flag = 0;
	sei.time_code.clock_ts[0].n_frames = n_frames;
	sei.time_code.clock_ts[0].seconds_value = seconds_value;
	sei.time_code.clock_ts[0].minutes_value = minutes_value;
	sei.time_code.clock_ts[0].hours_value = hours_value;
	sei.time_code.clock_ts[0].time_offset_length = time_offset_len;
	sei.time_code.clock_ts[0].time_offset_value = time_offset;

	res = h265_ctx_add_sei(h265, &sei);
	if (res < 0) {
		ULOG_ERRNO("h265_ctx_add_sei", -res);
		return res;
	}

	return 0;
}


int venc_h265_sei_add_mdcv(struct h265_ctx *h265,
			   const struct vdef_format_info *format)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(h265 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(format == NULL, EINVAL);

	const struct vdef_color_primaries_value *primaries =
		&format->mdcv.display_primaries_val;
	if (format->mdcv.display_primaries != VDEF_COLOR_PRIMARIES_UNKNOWN) {
		primaries = &vdef_color_primaries_values
				    [format->mdcv.display_primaries];
	}

	struct h265_sei sei = {0};
	sei.type = H265_SEI_TYPE_MASTERING_DISPLAY_COLOUR_VOLUME;
	for (unsigned int k = 0; k < 3; k++) {
		sei.mastering_display_colour_volume.display_primaries_x[k] =
			roundf(primaries->color_primaries[k].x * 50000.);
		sei.mastering_display_colour_volume.display_primaries_y[k] =
			roundf(primaries->color_primaries[k].y * 50000.);
	}
	sei.mastering_display_colour_volume.white_point_x =
		roundf(primaries->white_point.x * 50000.);
	sei.mastering_display_colour_volume.white_point_y =
		roundf(primaries->white_point.y * 50000.);
	sei.mastering_display_colour_volume.max_display_mastering_luminance =
		roundf(format->mdcv.max_display_mastering_luminance * 10000.);
	sei.mastering_display_colour_volume.min_display_mastering_luminance =
		roundf(format->mdcv.min_display_mastering_luminance * 10000.);

	res = h265_ctx_add_sei(h265, &sei);
	if (res < 0) {
		ULOG_ERRNO("h265_ctx_add_sei", -res);
		return res;
	}

	return 0;
}


int venc_h265_sei_add_cll(struct h265_ctx *h265,
			  const struct vdef_format_info *format)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(h265 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(format == NULL, EINVAL);

	struct h265_sei sei = {0};
	sei.type = H265_SEI_TYPE_CONTENT_LIGHT_LEVEL;
	sei.content_light_level.max_content_light_level = format->cll.max_cll;
	sei.content_light_level.max_pic_average_light_level =
		format->cll.max_fall;

	res = h265_ctx_add_sei(h265, &sei);
	if (res < 0) {
		ULOG_ERRNO("h265_ctx_add_sei", -res);
		return res;
	}

	return 0;
}


int venc_h265_sei_add_user_data(struct h265_ctx *h265,
				const uint8_t *data,
				size_t len)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(h265 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(data == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len < 16, EINVAL);

	struct h265_sei sei = {0};
	sei.type = H265_SEI_TYPE_USER_DATA_UNREGISTERED;
	sei.user_data_unregistered.buf = data + 16;
	sei.user_data_unregistered.len = len - 16;
	memcpy(sei.user_data_unregistered.uuid,
	       data,
	       sizeof(sei.user_data_unregistered.uuid));

	res = h265_ctx_add_sei(h265, &sei);
	if (res < 0) {
		ULOG_ERRNO("h265_ctx_add_sei", -res);
		return res;
	}

	return 0;
}


int venc_h265_sei_write(struct h265_ctx *h265,
			struct mbuf_coded_video_frame *frame)
{
	int res, count, err;
	struct vdef_nalu nalu;
	struct mbuf_mem *mem = NULL;
	void *void_data;
	uint8_t *data, *start;
	struct h265_bitstream bs;
	size_t len, size;
	struct vdef_coded_frame info;

	ULOG_ERRNO_RETURN_ERR_IF(h265 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);

	res = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		return res;
	}

	count = h265_ctx_get_sei_count(h265);
	if (count < 0) {
		ULOG_ERRNO("h265_ctx_get_sei_count", -count);
		return count;
	}
	if (count == 0)
		return 0;

	struct h265_nalu_header nh;
	nh.forbidden_zero_bit = 0;
	nh.nal_unit_type = H265_NALU_TYPE_PREFIX_SEI_NUT;
	nh.nuh_layer_id = 0;
	nh.nuh_temporal_id_plus1 = 1;
	res = h265_ctx_set_nalu_header(h265, &nh);
	if (res < 0) {
		ULOG_ERRNO("h265_ctx_set_nalu_header", -res);
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
	start = data = (uint8_t *)void_data;
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
	} else if (info.format.data_format == VDEF_CODED_DATA_FORMAT_HVCC) {
		if (len <= 4) {
			res = -ENOBUFS;
			ULOG_ERRNO("", -res);
			goto out;
		}
		size += 4;
		data += size;
		len -= size;
	}
	h265_bs_init(&bs, data, len, 1);
	res = h265_write_nalu(&bs, h265);
	if (res < 0) {
		ULOG_ERRNO("h265_write_nalu", -res);
		goto out;
	}

	if (info.format.data_format == VDEF_CODED_DATA_FORMAT_AVCC) {
		uint32_t sz = htonl(bs.off);
		memcpy(start, &sz, 4);
	}
	size += bs.off;
	nalu.h265.type = H265_NALU_TYPE_PREFIX_SEI_NUT;
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


int venc_h265_generate_nalus(struct venc_encoder *self,
			     struct mbuf_coded_video_frame *frame,
			     const struct vdef_coded_frame *info)
{
	int ret = 0, sei_count = 0;

	/* AUD insertion */
	if (self->config.h265.insert_aud) {
		ret = venc_h265_aud_write(self->h265.ctx, frame);
		if (ret < 0)
			goto out;
	}

	/* VPS/SPS/PPS insertion */
	if ((self->config.h265.insert_ps) && (self->h265.vps != NULL) &&
	    (self->h265.vps_size > 0) && (self->h265.sps != NULL) &&
	    (self->h265.sps_size > 0) && (self->h265.pps != NULL) &&
	    (self->h265.pps_size > 0) &&
	    ((info->type == VDEF_CODED_FRAME_TYPE_IDR) ||
	     (info->type == VDEF_CODED_FRAME_TYPE_I) ||
	     (info->type == VDEF_CODED_FRAME_TYPE_P_IR_START))) {
		ret = venc_h265_ps_copy(self->h265.ctx,
					frame,
					self->h265.vps,
					self->h265.vps_size,
					self->h265.sps,
					self->h265.sps_size,
					self->h265.pps,
					self->h265.pps_size);
		if (ret < 0)
			goto out;
	}

	/* Generate SEI */
	ret = venc_h265_sei_reset(self->h265.ctx);
	if (ret < 0)
		goto out;

	if ((self->config.h265.insert_recovery_point_sei) &&
	    (info->type == VDEF_CODED_FRAME_TYPE_P_IR_START)) {
		/* Time code sei */
		ret = venc_h265_sei_add_recovery_point(
			self->h265.ctx, self->recovery_frame_cnt);
		if (ret < 0)
			goto out;
		sei_count++;
	}

	if ((self->config.h265.insert_time_code_sei) &&
	    (info->info.capture_timestamp != 0)) {
		/* Picture timing sei */
		ret = venc_h265_sei_add_time_code(self->h265.ctx,
						  info->info.capture_timestamp);
		if (ret < 0)
			goto out;
		sei_count++;
	}

	if ((self->config.h265.insert_mdcv_sei) &&
	    ((info->type == VDEF_CODED_FRAME_TYPE_IDR) ||
	     (info->type == VDEF_CODED_FRAME_TYPE_I) ||
	     (info->type == VDEF_CODED_FRAME_TYPE_P_IR_START)) &&
	    ((self->config.input.info.mdcv.display_primaries !=
	      VDEF_COLOR_PRIMARIES_UNKNOWN) ||
	     ((self->config.input.info.mdcv.display_primaries_val
		       .color_primaries[0]
		       .x != 0.) &&
	      (self->config.input.info.mdcv.display_primaries_val
		       .color_primaries[0]
		       .y != 0.) &&
	      (self->config.input.info.mdcv.display_primaries_val
		       .color_primaries[1]
		       .x != 0.) &&
	      (self->config.input.info.mdcv.display_primaries_val
		       .color_primaries[1]
		       .y != 0.) &&
	      (self->config.input.info.mdcv.display_primaries_val
		       .color_primaries[2]
		       .x != 0.) &&
	      (self->config.input.info.mdcv.display_primaries_val
		       .color_primaries[2]
		       .y != 0.) &&
	      (self->config.input.info.mdcv.display_primaries_val.white_point
		       .x != 0.) &&
	      (self->config.input.info.mdcv.display_primaries_val.white_point
		       .y != 0.))) &&
	    (self->config.input.info.mdcv.max_display_mastering_luminance !=
	     0.) &&
	    (self->config.input.info.mdcv.min_display_mastering_luminance !=
	     0.)) {
		/* Mastering display colour volume SEI */
		ret = venc_h265_sei_add_mdcv(self->h265.ctx,
					     &self->config.input.info);
		if (ret == 0)
			sei_count++;
	}

	if ((self->config.h265.insert_cll_sei) &&
	    ((info->type == VDEF_CODED_FRAME_TYPE_IDR) ||
	     (info->type == VDEF_CODED_FRAME_TYPE_I) ||
	     (info->type == VDEF_CODED_FRAME_TYPE_P_IR_START)) &&
	    (self->config.input.info.cll.max_cll != 0) &&
	    (self->config.input.info.cll.max_fall != 0)) {
		/* Content light level information SEI */
		ret = venc_h265_sei_add_cll(self->h265.ctx,
					    &self->config.input.info);
		if (ret < 0)
			goto out;
		sei_count++;
	}

	if (self->config.h265.serialize_user_data) {
		struct mbuf_ancillary_data *data;
		const void *raw_data;
		size_t len;

		/* User data SEI */
		ret = mbuf_coded_video_frame_get_ancillary_data(
			frame, MBUF_ANCILLARY_KEY_USERDATA_SEI, &data);
		if (ret == 0) {
			raw_data = mbuf_ancillary_data_get_buffer(data, &len);
			if (raw_data && (len >= 16)) {
				ret = venc_h265_sei_add_user_data(
					self->h265.ctx, raw_data, len);
				if (ret < 0)
					goto out;
				sei_count++;
			}
			mbuf_ancillary_data_unref(data);
		}
	}

	if (sei_count == 0)
		return 0;

	ret = venc_h265_sei_write(self->h265.ctx, frame);
	if (ret < 0)
		goto out;

out:
	return ret;
}


int venc_h265_format_convert(struct mbuf_coded_video_frame *frame,
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

	/* Only support byte stream to HVCC or HVCC to byte stream */
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
		     VDEF_CODED_DATA_FORMAT_HVCC) &&
		    (target_format->data_format ==
		     VDEF_CODED_DATA_FORMAT_BYTE_STREAM)) {
			/* HVCC to byte stream */
			memcpy(data, &start_code, sizeof(uint32_t));
		} else if ((current_format->data_format ==
			    VDEF_CODED_DATA_FORMAT_BYTE_STREAM) &&
			   (target_format->data_format ==
			    VDEF_CODED_DATA_FORMAT_HVCC)) {
			/* Byte stream to HVCC */
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
