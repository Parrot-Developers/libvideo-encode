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
ULOG_DECLARE_TAG(ULOG_TAG);

#include "venc_core_priv.h"
#include <futils/timetools.h>


enum venc_encoder_implem venc_encoder_implem_from_str(const char *str)
{
	if (strcasecmp(str, "X264") == 0)
		return VENC_ENCODER_IMPLEM_X264;
	else if (strcasecmp(str, "HISI") == 0)
		return VENC_ENCODER_IMPLEM_HISI;
	else if (strcasecmp(str, "FAKEH264") == 0)
		return VENC_ENCODER_IMPLEM_FAKEH264;
	else if (strcasecmp(str, "VIDEOTOOLBOX") == 0)
		return VENC_ENCODER_IMPLEM_VIDEOTOOLBOX;
	else
		return VENC_ENCODER_IMPLEM_AUTO;
}


const char *venc_encoder_implem_to_str(enum venc_encoder_implem implem)
{
	switch (implem) {
	case VENC_ENCODER_IMPLEM_X264:
		return "X264";
	case VENC_ENCODER_IMPLEM_HISI:
		return "HISI";
	case VENC_ENCODER_IMPLEM_FAKEH264:
		return "FAKEH264";
	case VENC_ENCODER_IMPLEM_VIDEOTOOLBOX:
		return "VIDEOTOOLBOX";
	case VENC_ENCODER_IMPLEM_AUTO:
	default:
		return "UNKNOWN";
	}
}


enum venc_rate_control venc_rate_control_from_str(const char *str)
{
	if (strcasecmp(str, "CBR") == 0)
		return VENC_RATE_CONTROL_CBR;
	else if (strcasecmp(str, "VBR") == 0)
		return VENC_RATE_CONTROL_VBR;
	else if (strcasecmp(str, "CQ") == 0)
		return VENC_RATE_CONTROL_CQ;
	else
		return VENC_RATE_CONTROL_CBR;
}


const char *venc_rate_control_to_str(enum venc_rate_control rc)
{
	switch (rc) {
	case VENC_RATE_CONTROL_CBR:
		return "CBR";
	case VENC_RATE_CONTROL_VBR:
		return "VBR";
	case VENC_RATE_CONTROL_CQ:
		return "CQ";
	default:
		return "UNKNOWN";
	}
}


enum venc_entropy_coding venc_entropy_coding_from_str(const char *str)
{
	if (strcasecmp(str, "CABAC") == 0)
		return VENC_ENTROPY_CODING_CABAC;
	else if (strcasecmp(str, "CAVLC") == 0)
		return VENC_ENTROPY_CODING_CAVLC;
	else
		return VENC_ENTROPY_CODING_CABAC;
}


const char *venc_entropy_coding_to_str(enum venc_entropy_coding coding)
{
	switch (coding) {
	case VENC_ENTROPY_CODING_CABAC:
		return "CABAC";
	case VENC_ENTROPY_CODING_CAVLC:
		return "CAVLC";
	default:
		return "UNKNOWN";
	}
}


enum venc_intra_refresh venc_intra_refresh_from_str(const char *str)
{
	if (strcasecmp(str, "VERTICAL_SCAN") == 0)
		return VENC_INTRA_REFRESH_VERTICAL_SCAN;
	else if (strcasecmp(str, "SMART_SCAN") == 0)
		return VENC_INTRA_REFRESH_SMART_SCAN;
	else
		return VENC_INTRA_REFRESH_NONE;
}


const char *venc_intra_refresh_to_str(enum venc_intra_refresh ir)
{
	switch (ir) {
	case VENC_INTRA_REFRESH_NONE:
		return "NONE";
	case VENC_INTRA_REFRESH_VERTICAL_SCAN:
		return "VERTICAL_SCAN";
	case VENC_INTRA_REFRESH_SMART_SCAN:
		return "SMART_SCAN";
	default:
		return "UNKNOWN";
	}
}


bool venc_default_input_filter(struct mbuf_raw_video_frame *frame,
			       void *userdata)
{
	int ret;
	bool accept;
	struct venc_encoder *encoder = userdata;
	const struct vdef_raw_format *supported_formats;
	struct vdef_raw_frame frame_info;

	if (!frame || !encoder)
		return false;

	ret = mbuf_raw_video_frame_get_frame_info(frame, &frame_info);
	if (ret != 0)
		return false;

	ret = encoder->ops->get_supported_input_formats(&supported_formats);
	if (ret < 0)
		return false;
	accept = venc_default_input_filter_internal(
		encoder, frame, &frame_info, supported_formats, ret);
	if (accept)
		venc_default_input_filter_internal_confirm_frame(
			encoder, frame, &frame_info);
	return accept;
}


bool venc_default_input_filter_internal(
	struct venc_encoder *encoder,
	struct mbuf_raw_video_frame *frame,
	struct vdef_raw_frame *frame_info,
	const struct vdef_raw_format *supported_formats,
	unsigned int nb_supported_formats)
{
	if (!vdef_raw_format_intersect(&frame_info->format,
				       supported_formats,
				       nb_supported_formats)) {
		ULOG_ERRNO(
			"unsupported format:"
			" " VDEF_RAW_FORMAT_TO_STR_FMT,
			EPROTO,
			VDEF_RAW_FORMAT_TO_STR_ARG(&frame_info->format));
		return false;
	}

	if (frame_info->info.timestamp <= encoder->last_timestamp &&
	    encoder->last_timestamp != UINT64_MAX) {
		ULOG_ERRNO("non-strictly-monotonic timestamp (%" PRIu64
			   " <= %" PRIu64 ")",
			   EPROTO,
			   frame_info->info.timestamp,
			   encoder->last_timestamp);
		return false;
	}

	if ((encoder->config.input.info.bit_depth !=
	     frame_info->info.bit_depth) ||
	    (encoder->config.input.info.full_range !=
	     frame_info->info.full_range) ||
	    !vdef_dim_cmp(&encoder->config.input.info.resolution,
			  &frame_info->info.resolution)) {
		ULOG_ERRNO(
			"invalid frame information "
			"expected (resolution:%ux%u, bit_depth:%d, range:%s) "
			"got (resolution:%ux%u, bit_depth:%d, range:%s)",
			EPROTO,
			encoder->config.input.info.resolution.width,
			encoder->config.input.info.resolution.height,
			encoder->config.input.info.bit_depth,
			encoder->config.input.info.full_range ? "full"
							      : "limited",
			frame_info->info.resolution.width,
			frame_info->info.resolution.height,
			frame_info->info.bit_depth,
			frame_info->info.full_range ? "full" : "limited");
		return false;
	}

	return true;
}

void venc_default_input_filter_internal_confirm_frame(
	struct venc_encoder *encoder,
	struct mbuf_raw_video_frame *frame,
	struct vdef_raw_frame *frame_info)
{
	int err;
	uint64_t ts_us;
	struct timespec cur_ts = {0, 0};

	/* Save frame timestamp to last_timestamp */
	encoder->last_timestamp = frame_info->info.timestamp;

	/* Set the input time ancillary data to the frame */
	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);
	err = mbuf_raw_video_frame_add_ancillary_buffer(
		frame, VENC_ANCILLARY_KEY_INPUT_TIME, &ts_us, sizeof(ts_us));
	if (err < 0)
		ULOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer", -err);
}


struct venc_config_impl *
venc_config_get_specific(struct venc_config *config,
			 enum venc_encoder_implem implem)
{
	/* Check if specific config is present */
	if (!config->implem_cfg)
		return NULL;

	/* Check if implementation is the right one */
	if (config->implem != implem) {
		ULOGI("specific config found, but implementation is %s "
		      "instead of %s. ignoring specific config",
		      venc_encoder_implem_to_str(config->implem),
		      venc_encoder_implem_to_str(implem));
		return NULL;
	}

	/* Check if specific config implementation matches the base one */
	if (config->implem_cfg->implem != config->implem) {
		ULOGW("specific config implem (%s) does not match"
		      " base config implem (%s). ignoring specific config",
		      venc_encoder_implem_to_str(config->implem_cfg->implem),
		      venc_encoder_implem_to_str(config->implem));
		return NULL;
	}

	/* All tests passed, return specific config */
	return config->implem_cfg;
}


int venc_copy_raw_frame_as_metadata(struct mbuf_raw_video_frame *frame,
				    struct mbuf_mem *mem,
				    struct mbuf_raw_video_frame **ret_obj)
{
	int ret = 0, err;
	unsigned int plane_count = 0;
	struct vdef_raw_frame info;
	struct vmeta_frame *metadata = NULL;
	struct mbuf_raw_video_frame *meta_frame = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mem == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	ret = mbuf_raw_video_frame_get_frame_info(frame, &info);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_new", -ret);
		return ret;
	}

	ret = mbuf_raw_video_frame_new(&info, &meta_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_new", -ret);
		return ret;
	}

	/* libmedia-buffers does not accept to finalize a mbuf_raw_video_frame
	 * that does not provide the proper number of planes */
	plane_count = vdef_get_raw_frame_plane_count(&info.format);
	for (unsigned int i = 0; i < plane_count; i++) {
		ret = mbuf_raw_video_frame_set_plane(meta_frame, i, mem, 0, 0);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane", -ret);
			goto failure;
		}
	}

	/* Copy ancillary data */
	ret = mbuf_raw_video_frame_foreach_ancillary_data(
		frame, mbuf_raw_video_frame_ancillary_data_copier, meta_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_foreach_ancillary_data", -ret);
		goto failure;
	}

	/* Frame metadata */
	ret = mbuf_raw_video_frame_get_metadata(frame, &metadata);
	if (ret == 0) {
		ret = mbuf_raw_video_frame_set_metadata(meta_frame, metadata);
		vmeta_frame_unref(metadata);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_metadata", -ret);
			goto failure;
		}
	} else if (ret != -ENOENT) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_metadata", -ret);
		goto failure;
	}

	ret = mbuf_raw_video_frame_finalize(meta_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_finalize", -ret);
		goto failure;
	}

	*ret_obj = meta_frame;

failure:
	if ((ret < 0) && (meta_frame)) {
		err = mbuf_raw_video_frame_unref(meta_frame);
		if (err < 0)
			ULOG_ERRNO("mbuf_raw_video_frame_unref", -ret);
	}

	return ret;
}
