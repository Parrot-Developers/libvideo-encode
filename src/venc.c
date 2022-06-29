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

#define ULOG_TAG venc
#include <ulog.h>
ULOG_DECLARE_TAG(venc);

#include "venc_priv.h"


static const struct venc_ops *implem_ops(enum venc_encoder_implem implem)
{
	switch (implem) {
#ifdef BUILD_LIBVIDEO_ENCODE_HISI
	case VENC_ENCODER_IMPLEM_HISI:
		return &venc_hisi_ops;
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_QCOM
	case VENC_ENCODER_IMPLEM_QCOM:
		return &venc_qcom_ops;
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_MEDIACODEC
	case VENC_ENCODER_IMPLEM_MEDIACODEC:
		return &venc_mediacodec_ops;
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_VIDEOTOOLBOX
	case VENC_ENCODER_IMPLEM_VIDEOTOOLBOX:
		return &venc_videotoolbox_ops;
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_X264
	case VENC_ENCODER_IMPLEM_X264:
		return &venc_x264_ops;
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_TURBOJPEG
	case VENC_ENCODER_IMPLEM_TURBOJPEG:
		return &venc_turbojpeg_ops;
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_FAKEH264
	case VENC_ENCODER_IMPLEM_FAKEH264:
		return &venc_fakeh264_ops;
#endif
	default:
		return NULL;
	}
}


static int venc_get_implem(enum venc_encoder_implem *implem)
{
	ULOG_ERRNO_RETURN_ERR_IF(implem == NULL, EINVAL);

	/* Put preferred implementation first for autoselection */

#ifdef BUILD_LIBVIDEO_ENCODE_HISI
	if ((*implem == VENC_ENCODER_IMPLEM_AUTO) ||
	    (*implem == VENC_ENCODER_IMPLEM_HISI)) {
		*implem = VENC_ENCODER_IMPLEM_HISI;
		return 0;
	}
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_QCOM
	if ((*implem == VENC_ENCODER_IMPLEM_AUTO) ||
	    (*implem == VENC_ENCODER_IMPLEM_QCOM)) {
		*implem = VENC_ENCODER_IMPLEM_QCOM;
		return 0;
	}
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_MEDIACODEC
	if ((*implem == VENC_ENCODER_IMPLEM_AUTO) ||
	    (*implem == VENC_ENCODER_IMPLEM_MEDIACODEC)) {
		*implem = VENC_ENCODER_IMPLEM_MEDIACODEC;
		return 0;
	}
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_VIDEOTOOLBOX
	if ((*implem == VENC_ENCODER_IMPLEM_AUTO) ||
	    (*implem == VENC_ENCODER_IMPLEM_VIDEOTOOLBOX)) {
		*implem = VENC_ENCODER_IMPLEM_VIDEOTOOLBOX;
		return 0;
	}
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_X264
	if ((*implem == VENC_ENCODER_IMPLEM_AUTO) ||
	    (*implem == VENC_ENCODER_IMPLEM_X264)) {
		*implem = VENC_ENCODER_IMPLEM_X264;
		return 0;
	}
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_FAKEH264
	if ((*implem == VENC_ENCODER_IMPLEM_AUTO) ||
	    (*implem == VENC_ENCODER_IMPLEM_FAKEH264)) {
		*implem = VENC_ENCODER_IMPLEM_FAKEH264;
		return 0;
	}
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_TURBOJPEG
	if ((*implem == VENC_ENCODER_IMPLEM_AUTO) ||
	    (*implem == VENC_ENCODER_IMPLEM_TURBOJPEG)) {
		*implem = VENC_ENCODER_IMPLEM_TURBOJPEG;
		return 0;
	}
#endif

	/* No suitable implementation found */
	return -ENOSYS;
}


int venc_get_supported_encodings(enum venc_encoder_implem implem,
				 const enum vdef_encoding **encodings)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(!encodings, EINVAL);

	ret = venc_get_implem(&implem);
	ULOG_ERRNO_RETURN_VAL_IF(ret < 0, -ret, 0);

	return implem_ops(implem)->get_supported_encodings(encodings);
}


int venc_get_supported_input_formats(enum venc_encoder_implem implem,
				     const struct vdef_raw_format **formats)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(!formats, EINVAL);

	ret = venc_get_implem(&implem);
	ULOG_ERRNO_RETURN_VAL_IF(ret < 0, -ret, 0);

	return implem_ops(implem)->get_supported_input_formats(formats);
}


enum venc_encoder_implem venc_get_auto_implem(void)
{
	int ret;
	enum venc_encoder_implem implem = VENC_ENCODER_IMPLEM_AUTO;

	ret = venc_get_implem(&implem);
	ULOG_ERRNO_RETURN_VAL_IF(ret < 0, -ret, VENC_ENCODER_IMPLEM_AUTO);

	return implem;
}


enum venc_encoder_implem
venc_get_auto_implem_by_encoding(enum vdef_encoding encoding)
{
	int res = 0;
	const enum vdef_encoding *encodings;

	ULOG_ERRNO_RETURN_ERR_IF(!encoding, EINVAL);

	for (enum venc_encoder_implem implem = VENC_ENCODER_IMPLEM_AUTO + 1;
	     implem < VENC_ENCODER_IMPLEM_MAX;
	     implem++) {

		res = venc_get_implem(&implem);
		if (res < 0)
			continue;

		res = implem_ops(implem)->get_supported_encodings(&encodings);
		if (res < 0)
			continue;

		for (int i = 0; i < res; i++) {
			if (encodings[i] == encoding)
				return implem;
		}
	}

	return VENC_ENCODER_IMPLEM_AUTO;
}


int venc_new(struct pomp_loop *loop,
	     const struct venc_config *config,
	     const struct venc_cbs *cbs,
	     void *userdata,
	     struct venc_encoder **ret_obj)
{
	int res;
	struct venc_encoder *self = NULL;
	const struct vdef_raw_format *supported_formats;
	int nb_supported_formats;

	ULOG_ERRNO_RETURN_ERR_IF(loop == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(config == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	self = calloc(1, sizeof(*self));
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, ENOMEM);

	self->loop = loop;
	self->cbs = *cbs;
	self->userdata = userdata;
	self->config = *config;
	self->config.name = xstrdup(config->name);
	self->last_timestamp = UINT64_MAX;

	res = venc_get_implem(&self->config.implem);
	if (res < 0)
		goto error;
	self->ops = implem_ops(self->config.implem);

	nb_supported_formats =
		self->ops->get_supported_input_formats(&supported_formats);
	if (nb_supported_formats < 0) {
		res = nb_supported_formats;
		goto error;
	}
	if (!vdef_raw_format_intersect(&config->input.format,
				       supported_formats,
				       nb_supported_formats)) {
		res = -EINVAL;
		ULOG_ERRNO(
			"unsupported input format: " VDEF_RAW_FORMAT_TO_STR_FMT,
			-res,
			VDEF_RAW_FORMAT_TO_STR_ARG(&config->input.format));
		goto error;
	}

	/* Enforce configuration and provide default values */
	if ((self->config.input.info.resolution.width == 0) ||
	    (self->config.input.info.resolution.height == 0)) {
		ULOGE("invalid input dimensions %dx%d",
		      self->config.input.info.resolution.width,
		      self->config.input.info.resolution.height);
		res = -EINVAL;
		goto error;
	}
	if ((self->config.input.info.framerate.num == 0) ||
	    (self->config.input.info.framerate.den == 0)) {
		ULOGE("invalid input framerate %d/%d",
		      self->config.input.info.framerate.num,
		      self->config.input.info.framerate.den);
		res = -EINVAL;
		goto error;
	}
	if ((self->config.input.info.sar.width == 0) ||
	    (self->config.input.info.sar.height == 0)) {
		self->config.input.info.sar.width = 1;
		self->config.input.info.sar.height = 1;
	}
	self->mb_width =
		VDEF_ROUND_UP(self->config.input.info.resolution.width, 16);
	self->mb_height =
		VDEF_ROUND_UP(self->config.input.info.resolution.height, 16);
	if (self->config.input.info.color_primaries ==
	    VDEF_COLOR_PRIMARIES_UNKNOWN)
		self->config.input.info.color_primaries =
			VDEF_COLOR_PRIMARIES_BT709;
	if (self->config.input.info.transfer_function ==
	    VDEF_TRANSFER_FUNCTION_UNKNOWN)
		self->config.input.info.transfer_function =
			VDEF_TRANSFER_FUNCTION_BT709;

	switch (self->config.encoding) {
	case VDEF_ENCODING_H264:
		if (self->config.h264.profile == 0)
			self->config.h264.profile = VENC_H264_MAIN_PROFILE;
		if (self->config.h264.level == 0) {
			/* TODO: this could be set dynamically according to the
			 * resolution and bitrate */
			self->config.h264.level = VENC_H264_LEVEL_4_0;
		}
		if ((self->config.h264.rate_control == VENC_RATE_CONTROL_CQ) &&
		    (self->config.h264.qp == 0)) {
			ULOGE("invalid QP");
			res = -EINVAL;
			goto error;
		}
		if (((self->config.h264.rate_control ==
		      VENC_RATE_CONTROL_CBR) ||
		     (self->config.h264.rate_control ==
		      VENC_RATE_CONTROL_VBR)) &&
		    (self->config.h264.max_bitrate == 0)) {
			ULOGE("invalid bitrate");
			res = -EINVAL;
			goto error;
		}
		if (self->config.h264.target_bitrate == 0) {
			self->config.h264.target_bitrate =
				self->config.h264.max_bitrate;
		}
		if (self->config.h264.gop_length_sec == 0.f) {
			self->config.h264.gop_length_sec =
				VENC_DEFAULT_GOP_LENGTH_SEC;
		}
		if (self->config.h264.decimation == 0)
			self->config.h264.decimation = 1;
		if (self->config.h264.ref_frame_interval == 0)
			self->config.h264.ref_frame_interval = 1;
		if (self->config.h264.base_frame_interval <
		    self->config.h264.ref_frame_interval) {
			self->config.h264.base_frame_interval =
				self->config.h264.ref_frame_interval;
		}
		if ((self->config.h264.base_frame_interval %
		     self->config.h264.ref_frame_interval) != 0) {
			ULOGE("invalid base and ref frame intervals "
			      "(base=%d, ref=%d)",
			      self->config.h264.base_frame_interval,
			      self->config.h264.ref_frame_interval);
			res = -EINVAL;
			goto error;
		}
		if (self->config.h264.slice_size_mbrows == 0)
			self->config.h264.slice_size_mbrows = self->mb_height;
		self->slice_count = (self->mb_height +
				     self->config.h264.slice_size_mbrows - 1) /
				    self->config.h264.slice_size_mbrows;
		self->slice_mb_count =
			self->mb_width * self->config.h264.slice_size_mbrows;
		self->slice_mb_count_recovery_point = self->slice_mb_count;
		self->recovery_frame_cnt =
			((self->config.h264.intra_refresh !=
			  VENC_INTRA_REFRESH_NONE) &&
			 (self->config.h264.intra_refresh_length > 0))
				? (self->config.h264.intra_refresh_length -
				   1) * self->config.h264.base_frame_interval /
					  self->config.h264.ref_frame_interval
				: 0;
		if ((self->config.h264.streaming_user_data_sei_version != 0) &&
		    (self->config.h264.streaming_user_data_sei_version != 2) &&
		    (self->config.h264.streaming_user_data_sei_version != 4)) {
			res = -EINVAL;
			ULOGE("invalid streaming user data SEI version: %d",
			      self->config.h264
				      .streaming_user_data_sei_version);
			goto error;
		}
		break;
	case VDEF_ENCODING_H265:
		if (self->config.h265.profile == 0)
			self->config.h265.profile = VENC_H265_MAIN_PROFILE;
		if (self->config.h265.level == 0) {
			/* TODO: this could be set dynamically according to the
			 * resolution and bitrate */
			self->config.h265.level = VENC_H265_LEVEL_4_0;
		}
		if ((self->config.h265.rate_control == VENC_RATE_CONTROL_CQ) &&
		    (self->config.h265.qp == 0)) {
			ULOGE("invalid QP");
			res = -EINVAL;
			goto error;
		}
		if (((self->config.h265.rate_control ==
		      VENC_RATE_CONTROL_CBR) ||
		     (self->config.h265.rate_control ==
		      VENC_RATE_CONTROL_VBR)) &&
		    (self->config.h265.max_bitrate == 0)) {
			ULOGE("invalid bitrate");
			res = -EINVAL;
			goto error;
		}
		if (self->config.h265.target_bitrate == 0)
			self->config.h265.target_bitrate =
				self->config.h265.max_bitrate;
		if (self->config.h265.gop_length_sec == 0.f)
			self->config.h265.gop_length_sec =
				VENC_DEFAULT_GOP_LENGTH_SEC;
		if (self->config.h265.decimation == 0)
			self->config.h265.decimation = 1;
		if ((self->config.h265.streaming_user_data_sei_version != 0) &&
		    (self->config.h265.streaming_user_data_sei_version != 2) &&
		    (self->config.h265.streaming_user_data_sei_version != 4)) {
			res = -EINVAL;
			ULOGE("invalid streaming user data SEI version: %d",
			      self->config.h265
				      .streaming_user_data_sei_version);
			goto error;
		}
		break;
	case VDEF_ENCODING_MJPEG:
		if ((self->config.mjpeg.rate_control == VENC_RATE_CONTROL_CQ) &&
		    (self->config.mjpeg.quality == 0)) {
			ULOGE("invalid quality factor");
			res = -EINVAL;
			goto error;
		}
		if (((self->config.mjpeg.rate_control ==
		      VENC_RATE_CONTROL_CBR) ||
		     (self->config.mjpeg.rate_control ==
		      VENC_RATE_CONTROL_VBR)) &&
		    (self->config.mjpeg.max_bitrate == 0)) {
			ULOGE("invalid bitrate");
			res = -EINVAL;
			goto error;
		}
		if (self->config.mjpeg.target_bitrate == 0)
			self->config.mjpeg.target_bitrate =
				self->config.mjpeg.max_bitrate;
		break;
	default:
		res = -EINVAL;
		goto error;
	}


	res = self->ops->create(self);
	if (res < 0)
		goto error;

	*ret_obj = self;
	return 0;

error:
	venc_destroy(self);
	*ret_obj = NULL;
	return res;
}


int venc_flush(struct venc_encoder *self, int discard)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->ops->flush(self, discard);
}


int venc_stop(struct venc_encoder *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->ops->stop(self);
}


int venc_destroy(struct venc_encoder *self)
{
	int res = 0;

	if (self == NULL)
		return 0;

	if (self->ops != NULL)
		res = self->ops->destroy(self);

	if (res == 0) {
		switch (self->config.encoding) {
		case VDEF_ENCODING_H264:
			venc_h264_writer_destroy(self->h264.ctx);
			free(self->h264.sps);
			free(self->h264.pps);
			break;
		case VDEF_ENCODING_H265:
			venc_h265_writer_destroy(self->h265.ctx);
			free(self->h265.vps);
			free(self->h265.sps);
			free(self->h265.pps);
			break;
		default:
			break;
		}
		free((void *)self->config.name);
		free(self);
	}

	return res;
}


struct mbuf_pool *venc_get_input_buffer_pool(struct venc_encoder *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, NULL);

	return self->ops->get_input_buffer_pool(self);
}


struct mbuf_raw_video_frame_queue *
venc_get_input_buffer_queue(struct venc_encoder *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, NULL);

	return self->ops->get_input_buffer_queue(self);
}


int venc_get_h264_ps(struct venc_encoder *self,
		     uint8_t *sps,
		     size_t *sps_size,
		     uint8_t *pps,
		     size_t *pps_size)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((sps != NULL) && (sps_size == NULL), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((pps != NULL) && (pps_size == NULL), EINVAL);

	if ((self->h264.sps == NULL) || (self->h264.pps == NULL))
		return -EAGAIN;

	if (sps != NULL) {
		if (*sps_size < self->h264.sps_size)
			return -ENOBUFS;

		memcpy(sps, self->h264.sps, self->h264.sps_size);
		*sps_size = self->h264.sps_size;
	} else if (sps_size != NULL) {
		*sps_size = self->h264.sps_size;
	}

	if (pps != NULL) {
		if (*pps_size < self->h264.pps_size)
			return -ENOBUFS;

		memcpy(pps, self->h264.pps, self->h264.pps_size);
		*pps_size = self->h264.pps_size;
	} else if (pps_size != NULL) {
		*pps_size = self->h264.pps_size;
	}

	return 0;
}


int venc_get_h265_ps(struct venc_encoder *self,
		     uint8_t *vps,
		     size_t *vps_size,
		     uint8_t *sps,
		     size_t *sps_size,
		     uint8_t *pps,
		     size_t *pps_size)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((vps != NULL) && (vps_size == NULL), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((sps != NULL) && (sps_size == NULL), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((pps != NULL) && (pps_size == NULL), EINVAL);

	if ((self->h265.vps == NULL) || (self->h265.sps == NULL) ||
	    (self->h265.pps == NULL))
		return -EAGAIN;

	if (vps != NULL) {
		if (*vps_size < self->h265.vps_size)
			return -ENOBUFS;

		memcpy(vps, self->h265.vps, self->h265.vps_size);
		*vps_size = self->h265.vps_size;
	} else if (vps_size != NULL) {
		*vps_size = self->h265.vps_size;
	}

	if (sps != NULL) {
		if (*sps_size < self->h265.sps_size)
			return -ENOBUFS;

		memcpy(sps, self->h265.sps, self->h265.sps_size);
		*sps_size = self->h265.sps_size;
	} else if (sps_size != NULL) {
		*sps_size = self->h265.sps_size;
	}

	if (pps != NULL) {
		if (*pps_size < self->h265.pps_size)
			return -ENOBUFS;

		memcpy(pps, self->h265.pps, self->h265.pps_size);
		*pps_size = self->h265.pps_size;
	} else if (pps_size != NULL) {
		*pps_size = self->h265.pps_size;
	}

	return 0;
}


int venc_get_dyn_config(struct venc_encoder *self,
			struct venc_dyn_config *config)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(config == NULL, EINVAL);

	return self->ops->get_dyn_config(self, config);
}


int venc_set_dyn_config(struct venc_encoder *self,
			const struct venc_dyn_config *config)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(config == NULL, EINVAL);

	return self->ops->set_dyn_config(self, config);
}


int venc_get_input_buffer_constraints(
	enum venc_encoder_implem implem,
	const struct vdef_raw_format *format,
	struct venc_input_buffer_constraints *constraints)
{
	int ret;
	unsigned int nb_planes;

	ULOG_ERRNO_RETURN_ERR_IF(format == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(constraints == NULL, EINVAL);

	ret = venc_get_implem(&implem);
	ULOG_ERRNO_RETURN_VAL_IF(ret < 0, -ret, 0);

	if (implem_ops(implem)->get_input_buffer_constraints != NULL) {
		return implem_ops(implem)->get_input_buffer_constraints(
			format, constraints);
	} else {
		nb_planes = vdef_get_raw_frame_plane_count(format);
		memset(constraints->plane_stride_align,
		       0,
		       nb_planes * sizeof(*constraints->plane_stride_align));
		memset(constraints->plane_scanline_align,
		       0,
		       nb_planes * sizeof(*constraints->plane_scanline_align));
		memset(constraints->plane_size_align,
		       0,
		       nb_planes * sizeof(*constraints->plane_size_align));
	}

	return 0;
}


enum venc_encoder_implem venc_get_used_implem(struct venc_encoder *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(
		self == NULL, EINVAL, VENC_ENCODER_IMPLEM_AUTO);

	return self->config.implem;
}


VENC_API int venc_request_idr(struct venc_encoder *self)
{
	int ret = -ENOSYS;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		(self->config.encoding != VDEF_ENCODING_H264) &&
			(self->config.encoding != VDEF_ENCODING_H265),
		EINVAL);

	if (self->ops->request_idr)
		ret = self->ops->request_idr(self);

	return ret;
}
