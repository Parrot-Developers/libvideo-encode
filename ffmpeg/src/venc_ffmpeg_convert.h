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


const char *profile_to_av(enum vdef_encoding encoding,
			  enum venc_ffmpeg_backend_type be,
			  int bit_depth,
			  int profile);


const char *level_to_av(enum vdef_encoding encoding,
			enum venc_ffmpeg_backend_type be,
			int level);


int rate_control_to_av(enum venc_ffmpeg_backend_type be,
		       enum venc_rate_control rc,
		       bool low_delay,
		       bool high_quality,
		       const char **key_str,
		       const char **val_str);


const char *entroy_coding_to_av(enum venc_ffmpeg_backend_type be,
				enum venc_entropy_coding coding);


static const struct vdef_raw_format *
format_from_av_pixel_format(enum AVPixelFormat format)
{
	switch (format) {
	case AV_PIX_FMT_YUV420P:
	case AV_PIX_FMT_YUVJ420P:
		return &vdef_i420;

	case AV_PIX_FMT_NV12:
		return &vdef_nv12;

	case AV_PIX_FMT_NONE:
	default:
		return NULL;
	}
}


static enum AVPixelFormat
format_to_av_pixel_format(const struct vdef_raw_format *format)
{
	if (vdef_raw_format_cmp(format, &vdef_i420))
		return AV_PIX_FMT_YUV420P;
	if (vdef_raw_format_cmp(format, &vdef_nv12))
		return AV_PIX_FMT_NV12;
	return AV_PIX_FMT_NONE;
}


static inline void
h264_update_frame_type_and_layer(enum h264_nalu_type nalu_type,
				 uint8_t nri,
				 enum h264_slice_type slice_type,
				 enum vdef_coded_frame_type *frame_type,
				 uint8_t *frame_layer)
{
	switch (nalu_type) {
	case H264_NALU_TYPE_SLICE:
		switch (slice_type) {
		case H264_SLICE_TYPE_P:
			*frame_type = (nri == 0)
					      ? VDEF_CODED_FRAME_TYPE_P_NON_REF
					      : VDEF_CODED_FRAME_TYPE_P;
			if (frame_layer == NULL)
				break;
			switch (nri) {
			case 0:
				*frame_layer = 2;
				break;
			case 3:
				*frame_layer = 0;
				break;
			default:
				*frame_layer = (*frame_layer == 0) ? 0 : 1;
				break;
			}
			break;
		case H264_SLICE_TYPE_I:
			*frame_type =
				((*frame_type ==
				  VDEF_CODED_FRAME_TYPE_P_IR_START) ||
				 (*frame_type == VDEF_CODED_FRAME_TYPE_P) ||
				 (*frame_type ==
				  VDEF_CODED_FRAME_TYPE_P_NON_REF))
					? *frame_type
					: VDEF_CODED_FRAME_TYPE_I;
			if (frame_layer == NULL)
				break;
			switch (nri) {
			case 0:
				*frame_layer = 2;
				break;
			case 3:
				*frame_layer = 0;
				break;
			default:
				*frame_layer = (*frame_layer == 0) ? 0 : 1;
				break;
			}
			break;
		case H264_SLICE_TYPE_B:
			ULOGW("found B-slice NALU");
			break;
		default:
			break;
		}
		break;
	case H264_NALU_TYPE_SLICE_IDR:
		if ((*frame_type != VDEF_CODED_FRAME_TYPE_NOT_CODED) &&
		    (*frame_type != VDEF_CODED_FRAME_TYPE_IDR))
			ULOGW("mixing IDR and non-IDR NALU types");
		*frame_type = VDEF_CODED_FRAME_TYPE_IDR;
		if (frame_layer != NULL)
			*frame_layer = 0;
		break;
	case H264_NALU_TYPE_SEI:
	case H264_NALU_TYPE_SPS:
	case H264_NALU_TYPE_PPS:
	default:
		break;
	}
}


static inline enum vdef_coded_frame_type
h265_nalu_type_to_vdef_coded_frame_type(enum h265_nalu_type nalu_type)
{
	switch (nalu_type) {
	case H265_NALU_TYPE_TRAIL_R:
		return VDEF_CODED_FRAME_TYPE_P;
	case H265_NALU_TYPE_IDR_W_RADL:
		return VDEF_CODED_FRAME_TYPE_IDR;
	case H265_NALU_TYPE_PREFIX_SEI_NUT:
	case H265_NALU_TYPE_SUFFIX_SEI_NUT:
	case H265_NALU_TYPE_VPS_NUT:
	case H265_NALU_TYPE_SPS_NUT:
	case H265_NALU_TYPE_PPS_NUT:
	default:
		return VDEF_CODED_FRAME_TYPE_NOT_CODED;
	}
}
