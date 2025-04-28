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


/* Put preferred implementation first for autoselection */
static const enum venc_encoder_implem supported_implems[] = {
#ifdef BUILD_LIBVIDEO_ENCODE_HISI
	VENC_ENCODER_IMPLEM_HISI,
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_QCOM
	VENC_ENCODER_IMPLEM_QCOM,
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_QCOM_JPEG
	VENC_ENCODER_IMPLEM_QCOM_JPEG,
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_MEDIACODEC
	VENC_ENCODER_IMPLEM_MEDIACODEC,
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_VIDEOTOOLBOX
	VENC_ENCODER_IMPLEM_VIDEOTOOLBOX,
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_X264
	VENC_ENCODER_IMPLEM_X264,
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_X265
	VENC_ENCODER_IMPLEM_X265,
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_FAKEH264
	VENC_ENCODER_IMPLEM_FAKEH264,
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_TURBOJPEG
	VENC_ENCODER_IMPLEM_TURBOJPEG,
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_PNG
	VENC_ENCODER_IMPLEM_PNG,
#endif
};


/* Forward declaration */
static int venc_config_free_allocated(struct venc_config *config);


static const int supported_implems_count =
	sizeof(supported_implems) / sizeof(supported_implems[0]);


static atomic_int s_instance_counter;


static pthread_once_t instance_counter_is_init = PTHREAD_ONCE_INIT;
static void initialize_instance_counter(void)
{
	atomic_init(&s_instance_counter, 0);
}


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
#ifdef BUILD_LIBVIDEO_ENCODE_QCOM_JPEG
	case VENC_ENCODER_IMPLEM_QCOM_JPEG:
		return &venc_qcom_jpeg_ops;
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_MEDIACODEC
	case VENC_ENCODER_IMPLEM_MEDIACODEC:
		return &venc_mediacodec_ops;
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_VIDEOTOOLBOX
	case VENC_ENCODER_IMPLEM_VIDEOTOOLBOX:
		return &venc_videotoolbox_ops;
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_TURBOJPEG
	case VENC_ENCODER_IMPLEM_TURBOJPEG:
		return &venc_turbojpeg_ops;
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_X264
	case VENC_ENCODER_IMPLEM_X264:
		return &venc_x264_ops;
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_X265
	case VENC_ENCODER_IMPLEM_X265:
		return &venc_x265_ops;
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_FAKEH264
	case VENC_ENCODER_IMPLEM_FAKEH264:
		return &venc_fakeh264_ops;
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_PNG
	case VENC_ENCODER_IMPLEM_PNG:
		return &venc_png_ops;
#endif
	default:
		return NULL;
	}
}


static int venc_get_implem(enum venc_encoder_implem *implem)
{
	ULOG_ERRNO_RETURN_ERR_IF(implem == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!supported_implems_count, ENOSYS);

	if (*implem == VENC_ENCODER_IMPLEM_AUTO) {
		*implem = supported_implems[0];
		return 0;
	}

	for (int i = 0; i < supported_implems_count; i++)
		if (*implem == supported_implems[i])
			return 0;

	/* No suitable implementation found */
	return -ENOSYS;
}


int venc_get_supported_implems(const enum venc_encoder_implem **implems)
{
	ULOG_ERRNO_RETURN_ERR_IF(!implems, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!supported_implems_count, ENOSYS);

	*implems = supported_implems;

	return supported_implems_count;
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
	int res;
	const enum vdef_encoding *encodings;

	ULOG_ERRNO_RETURN_VAL_IF(encoding == VDEF_ENCODING_UNKNOWN,
				 EINVAL,
				 VENC_ENCODER_IMPLEM_AUTO);
	ULOG_ERRNO_RETURN_VAL_IF(
		!supported_implems_count, ENOSYS, VENC_ENCODER_IMPLEM_AUTO);

	for (int i = 0; i < supported_implems_count; i++) {
		enum venc_encoder_implem implem = supported_implems[i];

		res = implem_ops(implem)->get_supported_encodings(&encodings);
		if (res < 0)
			continue;

		for (int j = 0; j < res; j++) {
			if (encodings[j] == encoding)
				return implem;
		}
	}

	return VENC_ENCODER_IMPLEM_AUTO;
}


enum venc_encoder_implem venc_get_auto_implem_by_encoding_and_format(
	enum vdef_encoding encoding,
	const struct vdef_raw_format *format)
{
	int res;
	const enum vdef_encoding *encodings;
	const struct vdef_raw_format *supported_formats;

	ULOG_ERRNO_RETURN_VAL_IF(encoding == VDEF_ENCODING_UNKNOWN,
				 EINVAL,
				 VENC_ENCODER_IMPLEM_AUTO);
	ULOG_ERRNO_RETURN_VAL_IF(!format, EINVAL, VENC_ENCODER_IMPLEM_AUTO);
	ULOG_ERRNO_RETURN_VAL_IF(
		!supported_implems_count, ENOSYS, VENC_ENCODER_IMPLEM_AUTO);

	for (int i = 0; i < supported_implems_count; i++) {
		enum venc_encoder_implem implem = supported_implems[i];

		res = implem_ops(implem)->get_supported_encodings(&encodings);
		if (res < 0)
			continue;

		bool encoding_supported = false;
		for (int j = 0; j < res && !encoding_supported; j++) {
			if (encodings[j] == encoding)
				encoding_supported = true;
		}

		if (encoding_supported) {
			res = implem_ops(implem)->get_supported_input_formats(
				&supported_formats);
			if (res < 0)
				continue;
			if (vdef_raw_format_intersect(
				    format, supported_formats, res))
				return implem;
		}
	}

	return VENC_ENCODER_IMPLEM_AUTO;
}


static int venc_config_copy_allocated(const struct venc_config *config,
				      struct venc_config *copy)
{
	int ret;
	enum venc_encoder_implem implem;
	const struct venc_ops *ops = NULL;
	struct venc_config_impl *impl_cfg = NULL;

	implem = config->implem;

	ret = venc_get_implem(&implem);
	ULOG_ERRNO_RETURN_ERR_IF(ret < 0, -ret);

	ops = implem_ops(implem);
	ULOG_ERRNO_RETURN_ERR_IF(ops == NULL, ENOSYS);

	if (config->implem_cfg != NULL) {
		ULOG_ERRNO_RETURN_ERR_IF((ops->copy_implem_cfg == NULL),
					 ENOSYS);
		ULOG_ERRNO_RETURN_ERR_IF((ops->free_implem_cfg == NULL),
					 ENOSYS);
	}

	/* Deep copy config */
	*copy = *config;
	copy->name = xstrdup(config->name);
	copy->device = xstrdup(config->device);
	copy->implem_cfg = NULL;

	/* Handle implem specific */
	if (config->implem_cfg != NULL) {
		impl_cfg = venc_config_get_specific(config, implem);
		if (impl_cfg == NULL) {
			ret = -EPROTO;
			goto error;
		}
		/* Copy implem specific */
		ret = ops->copy_implem_cfg(impl_cfg, &copy->implem_cfg);
		if (ret < 0)
			goto error;
	}

	return 0;

error:
	(void)venc_config_free_allocated(copy);
	return ret;
}


int venc_config_copy(const struct venc_config *config,
		     struct venc_config **ret_obj)
{
	int ret;
	struct venc_config *copy = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(config == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	copy = calloc(1, sizeof(*copy));
	if (copy == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("calloc", -ret);
		return ret;
	}

	ret = venc_config_copy_allocated(config, copy);
	if (ret < 0)
		goto error;

	*ret_obj = copy;

	return 0;

error:
	if (copy != NULL)
		(void)venc_config_free(copy);
	return ret;
}


static int venc_config_free_allocated(struct venc_config *config)
{
	int ret, err;
	enum venc_encoder_implem implem;
	const struct venc_ops *ops = NULL;

	implem = config->implem;

	ret = venc_get_implem(&implem);
	ULOG_ERRNO_RETURN_ERR_IF(ret < 0, -ret);

	ops = implem_ops(implem);
	ULOG_ERRNO_RETURN_ERR_IF(ops == NULL, ENOSYS);

	/* Handle implem specific */
	if (config->implem_cfg != NULL) {
		ULOG_ERRNO_RETURN_ERR_IF((ops->free_implem_cfg == NULL),
					 ENOSYS);
		/* Free implem specific */
		err = ops->free_implem_cfg(config->implem_cfg);
		if (err < 0)
			ULOG_ERRNO("free_implem_cfg", -err);
	}

	free((void *)config->name);
	config->name = NULL;
	free((void *)config->device);
	config->device = NULL;

	return 0;
}


int venc_config_free(struct venc_config *config)
{
	int ret;

	if (config == NULL)
		return 0;

	ret = venc_config_free_allocated(config);

	free(config);

	return ret;
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

	(void)pthread_once(&instance_counter_is_init,
			   initialize_instance_counter);

	ULOG_ERRNO_RETURN_ERR_IF(loop == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(config == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	self = calloc(1, sizeof(*self));
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, ENOMEM);

	self->base = self; /* For logging */
	self->loop = loop;
	self->cbs = *cbs;
	self->userdata = userdata;
	self->last_timestamp = UINT64_MAX;
	self->enc_id = (atomic_fetch_add(&s_instance_counter, 1) + 1);
	atomic_init(&self->counters.in, 0);
	atomic_init(&self->counters.out, 0);
	atomic_init(&self->counters.pulled, 0);
	atomic_init(&self->counters.pushed, 0);

	res = venc_config_copy_allocated(config, &self->config);
	if (res < 0) {
		ULOG_ERRNO("venc_config_copy_allocated", -res);
		goto error;
	}

	if (self->config.name != NULL)
		res = asprintf(&self->enc_name, "%s", self->config.name);
	else
		res = asprintf(&self->enc_name, "%02d", self->enc_id);
	if (res < 0) {
		res = -ENOMEM;
		ULOG_ERRNO("asprintf", -res);
		goto error;
	}

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
		VENC_LOG_ERRNO(
			"unsupported input format: " VDEF_RAW_FORMAT_TO_STR_FMT,
			-res,
			VDEF_RAW_FORMAT_TO_STR_ARG(&config->input.format));
		goto error;
	}

	/* Enforce configuration and provide default values */
	if ((self->config.input.info.resolution.width == 0) ||
	    (self->config.input.info.resolution.height == 0)) {
		VENC_LOGE("invalid input dimensions %dx%d",
			  self->config.input.info.resolution.width,
			  self->config.input.info.resolution.height);
		res = -EINVAL;
		goto error;
	}
	if ((self->config.input.info.framerate.num == 0) ||
	    (self->config.input.info.framerate.den == 0)) {
		VENC_LOGE("invalid input framerate %d/%d",
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
			VENC_LOGE("invalid QP");
			res = -EINVAL;
			goto error;
		}
		if (((self->config.h264.rate_control ==
		      VENC_RATE_CONTROL_CBR) ||
		     (self->config.h264.rate_control ==
		      VENC_RATE_CONTROL_VBR)) &&
		    (self->config.h264.max_bitrate == 0)) {
			VENC_LOGE("invalid bitrate");
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
			VENC_LOGE(
				"invalid base and ref frame intervals "
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
			VENC_LOGE("invalid streaming user data SEI version: %d",
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
			VENC_LOGE("invalid QP");
			res = -EINVAL;
			goto error;
		}
		if (((self->config.h265.rate_control ==
		      VENC_RATE_CONTROL_CBR) ||
		     (self->config.h265.rate_control ==
		      VENC_RATE_CONTROL_VBR)) &&
		    (self->config.h265.max_bitrate == 0)) {
			VENC_LOGE("invalid bitrate");
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
			VENC_LOGE("invalid streaming user data SEI version: %d",
				  self->config.h265
					  .streaming_user_data_sei_version);
			goto error;
		}
		break;
	case VDEF_ENCODING_MJPEG:
		if ((self->config.mjpeg.rate_control == VENC_RATE_CONTROL_CQ) &&
		    (self->config.mjpeg.quality == 0)) {
			VENC_LOGE("invalid quality factor");
			res = -EINVAL;
			goto error;
		}
		if (((self->config.mjpeg.rate_control ==
		      VENC_RATE_CONTROL_CBR) ||
		     (self->config.mjpeg.rate_control ==
		      VENC_RATE_CONTROL_VBR)) &&
		    (self->config.mjpeg.max_bitrate == 0)) {
			VENC_LOGE("invalid bitrate");
			res = -EINVAL;
			goto error;
		}
		if (self->config.mjpeg.target_bitrate == 0)
			self->config.mjpeg.target_bitrate =
				self->config.mjpeg.max_bitrate;
		break;

	case VDEF_ENCODING_PNG:
		if (self->config.png.compression_level > 9) {
			VENC_LOGE("invalid compression level %u",
				  self->config.png.compression_level);
			res = -EINVAL;
			goto error;
		}
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
	int res = 0, err;

	if (self == NULL)
		return 0;

	if (self->ops != NULL)
		res = self->ops->destroy(self);

	VENC_LOGI("venc instance stats: [%u [%u %u] %u]",
		  self->counters.in,
		  self->counters.pushed,
		  self->counters.pulled,
		  self->counters.out);

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

		err = venc_config_free_allocated(&self->config);
		if (err < 0)
			VENC_LOG_ERRNO("venc_config_free_allocated", -err);

		free(self->enc_name);
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
