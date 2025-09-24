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

#define ULOG_TAG venc_ffmpeg
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "venc_ffmpeg_priv.h"

#include <futils/futils.h>


#define VENC_FFMPEG_MIN_QP 1
#define VENC_FFMPEG_MAX_QP 51


#define MAKE_BACKEND(enum_id, name, is_hw, encoding)                           \
	{                                                                      \
		VENC_FFMPEG_BACKEND_TYPE_##enum_id, name, NULL, is_hw, false,  \
			false,                                                 \
			{                                                      \
				{},                                            \
			},                                                     \
			0, {VDEF_ENCODING_##encoding}, 1,                      \
	}

#define MAKE_HW_BACKEND(enum_id, name, encoding)                               \
	MAKE_BACKEND(enum_id, name, true, encoding)

#define MAKE_SW_BACKEND(enum_id, name, encoding)                               \
	MAKE_BACKEND(enum_id, name, false, encoding)


/* Backends are sorted by priority, with HW-encoders first */
static struct venc_ffmpeg_backend s_backend_map[] = {
	/* HW-encoders */
	MAKE_HW_BACKEND(H264_NVENC, "h264_nvenc", H264),
	MAKE_HW_BACKEND(HEVC_NVENC, "hevc_nvenc", H265),
	/* SW-encoders */
	MAKE_SW_BACKEND(OPENH264, "libopenh264", H264),
};


static inline bool backend_has_format(const struct venc_ffmpeg_backend *backend,
				      const struct vdef_raw_format *format)
{
	for (size_t i = 0; i < backend->nb_supported_formats; i++) {
		if (vdef_raw_format_cmp(&backend->supported_formats[i], format))
			return true;
	}
	return false;
}


static inline bool
backend_has_encoding(const struct venc_ffmpeg_backend *backend,
		     enum vdef_encoding encoding)
{
	for (size_t i = 0; i < backend->nb_supported_encodings; i++) {
		if (backend->supported_encodings[i] == encoding)
			return true;
	}
	return false;
}


static int ensure_backend_is_supported(struct venc_ffmpeg_backend *backend)
{
	int ret;
	ULOG_ERRNO_RETURN_ERR_IF(backend == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(backend->codec->pix_fmts == NULL, EPROTO);

	if (backend->is_tested)
		return 0;

	AVCodecContext *ctx = avcodec_alloc_context3(backend->codec);
	if (!ctx) {
		ret = -EPROTO;
		ULOG_ERRNO("avcodec_alloc_context3", -ret);
		return ret;
	}
	ctx->width = 1280;
	ctx->height = 720;
	ctx->time_base = (AVRational){1, 30};
	ctx->framerate = (AVRational){30, 1};
	ctx->pix_fmt = backend->codec->pix_fmts[0];
	ret = avcodec_open2(ctx, backend->codec, NULL);
	if (ret < 0) {
		char errbuf[256];
		av_strerror(ret, errbuf, sizeof(errbuf));
		ULOGI("cannot open codec '%s': %s", backend->name, errbuf);
	} else {
		backend->is_supported = true;
	}
	avcodec_free_context(&ctx);
	backend->is_tested = true;
	return 0;
}


static struct venc_ffmpeg_backend *
get_preferred_backend_by_encoding(enum vdef_encoding encoding)
{
	for (size_t i = 0; i < SIZEOF_ARRAY(s_backend_map); i++) {
		struct venc_ffmpeg_backend *backend = &s_backend_map[i];
		if (!backend_has_encoding(backend, encoding))
			continue;
		backend->codec = avcodec_find_encoder_by_name(backend->name);
		if (backend->codec != NULL) {
			int err = ensure_backend_is_supported(backend);
			if (err < 0)
				ULOG_ERRNO("ensure_backend_is_supported", -err);
			if (backend->is_supported)
				return backend;
		}
	}
	return NULL;
}


static int av_to_ulog_level(int level)
{
	switch (level) {
	case AV_LOG_PANIC:
		return ULOG_CRIT;
	case AV_LOG_FATAL:
	case AV_LOG_ERROR:
		return ULOG_ERR;
	case AV_LOG_WARNING:
		return ULOG_WARN;
	case AV_LOG_INFO:
		return ULOG_INFO;
	case AV_LOG_VERBOSE:
		return ULOG_INFO;
	case AV_LOG_DEBUG:
	case AV_LOG_TRACE:
	default:
		return 0;
	}
}


static void av_log_cb(void *avcl, int level, const char *fmt, va_list vl)
{
	char *str = NULL;
	int l = av_to_ulog_level(level);
	if (l == 0)
		return;
	int ret = asprintf(&str, "av: %s", fmt);
#ifdef __clang__
#	pragma clang diagnostic push
#	pragma clang diagnostic ignored "-Wformat-nonliteral"
#endif
	if (ret > 0 && str != NULL)
		ULOG_PRI_VA(l, str, vl);
#ifdef __clang__
#	pragma clang diagnostic pop
#endif
	free(str);
}


static enum vdef_encoding supported_encodings[MAX_SUPPORTED_ENCODINGS];
static size_t nb_supported_encodings;
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;
static void initialize_supported_formats(void)
{
	nb_supported_encodings = 0;

	/* Find the encoder (hide errors) */
	av_log_set_level(AV_LOG_PANIC);

	enum vdef_encoding encoding_list[] = {
		VDEF_ENCODING_H264,
		VDEF_ENCODING_H265,
	};

	for (size_t i = 0; i < SIZEOF_ARRAY(encoding_list); i++) {
		enum vdef_encoding enc = encoding_list[i];
		struct venc_ffmpeg_backend *backend =
			get_preferred_backend_by_encoding(enc);
		if (backend == NULL)
			continue;
		/* Fill implem input formats */
		for (const enum AVPixelFormat *p = backend->codec->pix_fmts;
		     *p != AV_PIX_FMT_NONE;
		     p++) {
			if (p == NULL)
				break;
			const struct vdef_raw_format *format =
				format_from_av_pixel_format(*p);
			if ((format != NULL) &&
			    !backend_has_format(backend, format)) {
				backend->supported_formats
					[backend->nb_supported_formats] =
					*format;
				backend->nb_supported_formats++;
			}
		}
		backend->supported_formats[backend->nb_supported_formats] =
			vdef_gray;
		backend->nb_supported_formats++;

		ULOGD("supported_encodings[%zu] = %s",
		      nb_supported_encodings,
		      vdef_encoding_to_str(enc));
		for (size_t i = 0; i < backend->nb_supported_formats; i++) {
			ULOGD("  - supported_formats[%zu]"
			      " = " VDEF_RAW_FORMAT_TO_STR_FMT,
			      i,
			      VDEF_RAW_FORMAT_TO_STR_ARG(
				      &backend->supported_formats[i]));
		}

		supported_encodings[nb_supported_encodings] = enc;
		nb_supported_encodings++;
	}

	/* Show errors */
	av_log_set_level(AV_LOG_VERBOSE);
}


static void call_flush_done(void *userdata)
{
	struct venc_ffmpeg *self = userdata;

	venc_call_flush_cb(self->base);
}


static void call_stop_done(void *userdata)
{
	struct venc_ffmpeg *self = userdata;

	venc_call_stop_cb(self->base);
}


static void mbox_cb(int fd, uint32_t revents, void *userdata)
{
	struct venc_ffmpeg *self = userdata;
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
	struct venc_ffmpeg *self = userdata;
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


static inline enum h264_nalu_type extract_h264_type(uint8_t *data)
{
	return *data & 0x1F;
}


static inline enum h265_nalu_type extract_h265_type(uint8_t *data)
{
	return (*data >> 1) & 0x3F;
}


static int save_h264_ps(struct venc_ffmpeg *self, uint8_t *data, size_t len)
{
	bool sps_found = false;
	bool pps_found = false;
	int ret;

	while (len > 0 && (!sps_found || !pps_found)) {
		enum h264_nalu_type type;
		uint8_t **ps = NULL;
		size_t *ps_size = 0;
		size_t start, end;
		size_t size;

		/* Find next NALU */
		ret = h264_find_nalu(data, len, &start, &end);
		if (ret < 0 && ret != -EAGAIN) {
			VENC_LOG_ERRNO("h264_find_nalu", -ret);
			return ret;
		}
		data += start;
		len -= end;
		size = end - start;

		/* Parse NALU header */
		type = extract_h264_type(data);
		switch (type) {
		case H264_NALU_TYPE_SPS:
			ps = &self->base->h264.sps;
			ps_size = &self->base->h264.sps_size;
			sps_found = true;
			break;
		case H264_NALU_TYPE_PPS:
			ps = &self->base->h264.pps;
			ps_size = &self->base->h264.pps_size;
			pps_found = true;
			break;
		default:
			data += size;
			continue;
		}

		/* Save PS */
		if (size != *ps_size) {
			uint8_t *tmp = realloc(*ps, size);
			if (!tmp) {
				ret = -errno;
				VENC_LOG_ERRNO("realloc", -ret);
				return ret;
			}
			*ps = tmp;
			*ps_size = size;
			memcpy(*ps, data, size);
		}
		data += size;
	}

	/* Check SPS and PPS are found */
	if (!sps_found || !pps_found) {
		VENC_LOGE("missing PS NALUs: sps=%d pps=%d",
			  sps_found,
			  pps_found);
		return -EINVAL;
	}

	return 0;
}


static int save_h265_ps(struct venc_ffmpeg *self, uint8_t *data, size_t len)
{
	bool vps_found = false;
	bool sps_found = false;
	bool pps_found = false;
	int ret;

	while (len > 0 && (!vps_found || !sps_found || !pps_found)) {
		enum h265_nalu_type type;
		uint8_t **ps = NULL;
		size_t *ps_size = 0;
		size_t start, end;
		size_t size;

		/* Find next NALU */
		ret = h265_find_nalu(data, len, &start, &end);
		if (ret < 0 && ret != -EAGAIN) {
			VENC_LOG_ERRNO("h265_find_nalu", -ret);
			return ret;
		}
		data += start;
		len -= end;
		size = end - start;

		/* Parse NALU header */
		type = extract_h265_type(data);
		switch (type) {
		case H265_NALU_TYPE_VPS_NUT:
			ps = &self->base->h265.vps;
			ps_size = &self->base->h265.vps_size;
			vps_found = true;
			break;
		case H265_NALU_TYPE_SPS_NUT:
			ps = &self->base->h265.sps;
			ps_size = &self->base->h265.sps_size;
			sps_found = true;
			break;
		case H265_NALU_TYPE_PPS_NUT:
			ps = &self->base->h265.pps;
			ps_size = &self->base->h265.pps_size;
			pps_found = true;
			break;
		default:
			data += size;
			continue;
		}

		/* Save PS */
		if (size != *ps_size) {
			uint8_t *tmp = realloc(*ps, size);
			if (!tmp) {
				ret = -errno;
				VENC_LOG_ERRNO("realloc", -ret);
				return ret;
			}
			*ps = tmp;
			*ps_size = size;
			memcpy(*ps, data, size);
		}
		data += size;
	}

	/* Check VPS, SPS and PPS are found */
	if (!vps_found || !sps_found || !pps_found) {
		VENC_LOGE("missing PS NALUs: vps=%d sps=%d pps=%d",
			  vps_found,
			  sps_found,
			  pps_found);
		return -EINVAL;
	}

	return 0;
}


static int init_h264_writer(struct venc_ffmpeg *self)
{
	int ret;

	if ((self->base->h264.ctx != NULL) || (self->base->h264.sps == NULL) ||
	    (self->base->h264.pps == NULL))
		return 0;

	/* Initialize the H.264 writer */
	ret = venc_h264_writer_new(self->base->h264.sps,
				   self->base->h264.sps_size,
				   self->base->h264.pps,
				   self->base->h264.pps_size,
				   &self->base->h264.ctx);
	if (ret < 0) {
		VENC_LOG_ERRNO("venc_h264_writer_new", -ret);
		return ret;
	}
	ret = venc_h264_patch_ps(self->base);
	if (ret < 0) {
		VENC_LOG_ERRNO("venc_h264_patch_ps", -ret);
		return ret;
	}

	self->ps_ready = true;
	return 0;
}


static int init_h265_writer(struct venc_ffmpeg *self)
{
	int ret;

	if ((self->base->h265.ctx != NULL) || (self->base->h265.vps == NULL) ||
	    (self->base->h265.sps == NULL) || (self->base->h265.pps == NULL))
		return 0;

	/* Create the H.265 writer */
	ret = venc_h265_writer_new(self->base->h265.vps,
				   self->base->h265.vps_size,
				   self->base->h265.sps,
				   self->base->h265.sps_size,
				   self->base->h265.pps,
				   self->base->h265.pps_size,
				   &self->base->h265.ctx);
	if (ret < 0) {
		VENC_LOG_ERRNO("venc_h265_writer_new", -ret);
		return ret;
	}
	ret = venc_h265_patch_ps(self->base);
	if (ret < 0) {
		VENC_LOG_ERRNO("venc_h265_patch_ps", -ret);
		return ret;
	}

	self->ps_ready = true;
	return 0;
}


static int save_ps(struct venc_ffmpeg *self, uint8_t *data, size_t len)
{
	int res;

	if (self->ps_ready)
		return 0;

	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264:
		res = save_h264_ps(self, data, len);
		if (res < 0) {
			VENC_LOG_ERRNO("save_h264_ps", -res);
			return res;
		}
		res = init_h264_writer(self);
		if (res < 0) {
			VENC_LOG_ERRNO("init_h264_writer", -res);
			return res;
		}
		break;
	case VDEF_ENCODING_H265:
		res = save_h265_ps(self, data, len);
		if (res < 0) {
			VENC_LOG_ERRNO("save_h265_ps", -res);
			return res;
		}
		res = init_h265_writer(self);
		if (res < 0) {
			VENC_LOG_ERRNO("init_h265_writer", -res);
			return res;
		}
		break;
	default:
		VENC_LOGW("encoding not supported");
		return -1;
	}

	return 0;
}


static void
venc_ffmpeg_avframe_mbuf_free(void *data, size_t len, void *userdata)
{
	AVPacket *packet = userdata;
	av_packet_free(&packet);
}


static bool is_slice(struct venc_ffmpeg *self, const struct vdef_nalu *nalu)
{
	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264:
		return (nalu->h264.type == H264_NALU_TYPE_SLICE) ||
		       (nalu->h264.type == H264_NALU_TYPE_SLICE_IDR);
	case VDEF_ENCODING_H265:
		return (nalu->h265.type == H265_NALU_TYPE_TRAIL_N) ||
		       (nalu->h265.type == H265_NALU_TYPE_TRAIL_R) ||
		       (nalu->h265.type == H265_NALU_TYPE_RADL_N) ||
		       (nalu->h265.type == H265_NALU_TYPE_RADL_R) ||
		       (nalu->h265.type == H265_NALU_TYPE_RASL_N) ||
		       (nalu->h265.type == H265_NALU_TYPE_RASL_R) ||
		       (nalu->h265.type == H265_NALU_TYPE_BLA_W_LP) ||
		       (nalu->h265.type == H265_NALU_TYPE_BLA_W_RADL) ||
		       (nalu->h265.type == H265_NALU_TYPE_BLA_N_LP) ||
		       (nalu->h265.type == H265_NALU_TYPE_IDR_W_RADL) ||
		       (nalu->h265.type == H265_NALU_TYPE_IDR_N_LP) ||
		       (nalu->h265.type == H265_NALU_TYPE_CRA_NUT);
	default:
		return false;
	}
}


static bool detect_ps(struct venc_ffmpeg *self, const struct vdef_nalu *nalu)
{
	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264:
		switch (nalu->h264.type) {
		case H264_NALU_TYPE_SPS:
			return true;
		case H264_NALU_TYPE_PPS:
			return true;
		default:
			return false;
		}
	case VDEF_ENCODING_H265:
		switch (nalu->h265.type) {
		case H265_NALU_TYPE_VPS_NUT:
			return true;
		case H265_NALU_TYPE_SPS_NUT:
			return true;
		case H265_NALU_TYPE_PPS_NUT:
			return true;
		default:
			return false;
		}
	default:
		return false;
	}
}


static bool detect_sei(struct venc_ffmpeg *self, const struct vdef_nalu *nalu)
{
	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264:
		if (nalu->h264.type == H264_NALU_TYPE_SEI)
			return true;
		break;
	case VDEF_ENCODING_H265:
		if (nalu->h265.type == H265_NALU_TYPE_PREFIX_SEI_NUT ||
		    nalu->h265.type == H265_NALU_TYPE_SUFFIX_SEI_NUT)
			return true;
		break;
	default:
		break;
	}

	return false;
}


static bool is_valid_nalu(struct venc_ffmpeg *self,
			  const struct vdef_nalu *nalu)
{
	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264:
		return (nalu->h264.type > H264_NALU_TYPE_UNKNOWN) &&
		       (nalu->h264.type <= H264_NALU_TYPE_FILLER);
	case VDEF_ENCODING_H265:
		return (nalu->h265.type > H265_NALU_TYPE_UNKNOWN) &&
		       (nalu->h265.type <= H265_NALU_TYPE_UNSPEC63);
	default:
		return false;
	}
}


static int parse_buffer(struct venc_ffmpeg *self,
			uint8_t *data,
			size_t len,
			struct vdef_coded_frame *out_info)
{
	uint32_t mb_width = self->base->mb_width;
	uint32_t mb_height = self->base->mb_height;
	size_t offset = 0;
	int ret = 0;

	/* Reset NALU count */
	self->nalu_count = 0;

	/* Reset slice context */
	self->slice.cur_index = 0;
	self->slice.nb_slice = 0;

	/* Parse buffer to extract NALUs */
	while (len > 0) {
		struct venc_ffmpeg_nalu_info *nalu;
		size_t start, end;
		size_t size;
		uint8_t nri = 0;

		/* Find next NALU */
		switch (self->base->config.encoding) {
		case VDEF_ENCODING_H264:
			ret = h264_find_nalu(data, len, &start, &end);
			break;
		case VDEF_ENCODING_H265:
			ret = h265_find_nalu(data, len, &start, &end);
			break;
		default:
			ret = -ENOSYS;
			break;
		}
		if (ret < 0 && ret != -EAGAIN)
			break;
		data += start;
		len -= end;
		size = end - start;

		/* Resize NALU array to hold current NALU */
		if (self->nalu_max_count <= self->nalu_count) {
			void *tmp = realloc(self->nalus,
					    sizeof(*self->nalus) *
						    (self->nalu_count + 32));
			if (!tmp) {
				ret = -errno;
				VENC_LOG_ERRNO("failed to resize NALU array",
					       -ret);
				return ret;
			}
			self->nalus = tmp;
			self->nalu_max_count = self->nalu_count + 32;
		}
		nalu = &self->nalus[self->nalu_count];

		/* Extract NALU type */
		switch (self->base->config.encoding) {
		case VDEF_ENCODING_H264:
			nalu->n.h264.type = extract_h264_type(data);
			break;
		case VDEF_ENCODING_H265:
			nalu->n.h265.type = extract_h265_type(data);
			break;
		default:
			break;
		}

		/* Set NALU offset and size */
		nalu->n.size = end;
		nalu->offset = offset;
		offset += end;

		/* Handle slices */
		if (is_slice(self, &nalu->n)) {
			/* Extract H264 slice type and compute slice_mb_count */
			if (self->base->config.encoding == VDEF_ENCODING_H264) {
				struct h264_bitstream bs;
				uint32_t tmp;

				self->slice.cur_index++;

				/* Read slice type */
				h264_bs_cinit(&bs, data + 1, size, 1);
				h264_bs_read_bits_ue(&bs, &tmp);
				h264_bs_read_bits_ue(&bs, &tmp);
				nalu->n.h264.slice_type = H264_SLICE_TYPE(tmp);
				h264_bs_clear(&bs);

				/* Compute slice_mb_count */
				nalu->n.h264.slice_mb_count =
					(self->slice.cur_index ==
					 self->slice.nb_slice)
						? self->slice.last_size
						: self->slice.first_size;
			}
			self->slice.nb_slice++;
		}

		/* Find frame type from NALU type */
		switch (self->base->config.encoding) {
		case VDEF_ENCODING_H264:
			nri = 0; /* TODO */
			h264_update_frame_type_and_layer(
				nalu->n.h264.type,
				nri,
				nalu->n.h264.slice_type,
				&out_info->type,
				NULL);
			nalu->n.importance =
				venc_h264_get_nalu_importance(nalu->n.h264.type,
							      nri,
							      out_info->type,
							      out_info->layer);
			break;
		case VDEF_ENCODING_H265:
			out_info->type =
				h265_nalu_type_to_vdef_coded_frame_type(
					nalu->n.h265.type);
			nalu->n.importance = venc_h265_get_nalu_importance(
				nalu->n.h265.type, out_info->type);
			break;
		default:
			break;
		}

		/* Go to next NALU */
		self->nalu_count++;
		data += size;
	}

	/* Update slice context */
	if (self->slice.nb_slice == 0) {
		self->slice.first_size = 0;
		self->slice.last_size = 0;
	} else if (self->slice.nb_slice == 1) {
		self->slice.first_size = mb_height;
		self->slice.last_size = mb_height;
	} else {
		self->slice.first_size =
			VDEF_ROUND_UP(mb_height, self->slice.nb_slice);
		self->slice.last_size =
			mb_height -
			(self->slice.nb_slice - 1) * self->slice.first_size;
	}
	self->slice.first_size *= mb_width;
	self->slice.last_size *= mb_width;

	return 0;
}


static void frame_release(struct mbuf_coded_video_frame *frame, void *userdata)
{
	struct venc_ffmpeg *self = userdata;
	venc_call_pre_release_cb(self->base, frame);
}


static int
venc_ffmpeg_set_frame_metadata(struct venc_ffmpeg *self,
			       struct mbuf_raw_video_frame *in_frame,
			       struct mbuf_coded_video_frame **out_frame,
			       AVPacket *packet)
{
	int ret = 0, err;
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;
	struct mbuf_mem *out_mem = NULL;
	struct vmeta_frame *metadata = NULL;
	struct vdef_coded_frame out_info;
	struct vdef_raw_frame in_info;

	struct mbuf_coded_video_frame_cbs frame_cbs = {
		.pre_release = frame_release,
		.pre_release_userdata = (void *)self,
	};

	ret = mbuf_raw_video_frame_get_frame_info(in_frame, &in_info);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -ret);
		goto out;
	}

	out_info.info = in_info.info;
	out_info.type = VDEF_CODED_FRAME_TYPE_NOT_CODED;
	out_info.layer = UINT8_MAX;
	out_info.format.encoding = self->base->config.encoding;
	out_info.format.data_format =
		self->base->config.output.preferred_format;

	/* Parse buffer */
	err = parse_buffer(self, packet->data, packet->size, &out_info);
	if (err) {
		VENC_LOG_ERRNO("failed to parse output buffer", -err);
		goto out;
	}

	ret = mbuf_coded_video_frame_new(&out_info, out_frame);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_new", -ret);
		goto out;
	}
	ret = mbuf_coded_video_frame_set_callbacks(*out_frame, &frame_cbs);
	if (ret < 0)
		VENC_LOG_ERRNO("mbuf_coded_video_frame_set_callbacks", -ret);

	/* PS insertion */
	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264:
		/* Add generated NAL units */
		err = venc_h264_generate_nalus(
			self->base, *out_frame, &out_info);
		if (err < 0) {
			VENC_LOG_ERRNO("venc_h264_generate_nalus", -err);
			goto out;
		}
		break;
	case VDEF_ENCODING_H265:
		/* Add generated NAL units */
		err = venc_h265_generate_nalus(
			self->base, *out_frame, &out_info);
		if (err < 0) {
			VENC_LOG_ERRNO("venc_h265_generate_nalus", -err);
			goto out;
		}
		break;
	default:
		break;
	}

	/* set NALU to frames */
	ret = mbuf_mem_generic_wrap(packet->data,
				    packet->size,
				    venc_ffmpeg_avframe_mbuf_free,
				    packet,
				    &out_mem);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_mem_generic_wrap", -ret);
		goto out;
	}

	/* Add NALUs to frame */
	for (unsigned int i = 0; i < self->nalu_count; i++) {
		struct venc_ffmpeg_nalu_info *nalu = &self->nalus[i];
		uint32_t nalu_size;

		/* Filter out *PS, SEI NAL units and unknown
		 * or invalid NAL unit types  */
		bool is_ps = detect_ps(self, &nalu->n);
		bool is_sei = detect_sei(self, &nalu->n);
		bool is_valid = is_valid_nalu(self, &nalu->n);
		if (is_ps || is_sei || !is_valid)
			continue;

		/* Patch output based on required format */
		switch (self->base->config.output.preferred_format) {
		case VDEF_CODED_DATA_FORMAT_BYTE_STREAM:
			/* Nothing to change */
			break;
		/* same as VDEF_CODED_DATA_FORMAT_HVCC */
		case VDEF_CODED_DATA_FORMAT_AVCC:
			/* Patch start codes */
			nalu_size = htonl(nalu->n.size - sizeof(nalu_size));
			memcpy(packet->data + nalu->offset,
			       &nalu_size,
			       sizeof(nalu_size));
			break;
		case VDEF_CODED_DATA_FORMAT_RAW_NALU:
			/* Skip start codes */
			nalu->offset += sizeof(nalu_size);
			nalu->n.size -= sizeof(nalu_size);
			break;
		default:
			/* Format is sanitized in new, this should
			never happen */
			break;
		}

		err = mbuf_coded_video_frame_add_nalu(
			*out_frame, out_mem, nalu->offset, &nalu->n);
		if (err < 0) {
			VENC_LOG_ERRNO("mbuf_coded_video_frame_add_nalu", -err);
			goto out;
		}
	}

	ret = mbuf_raw_video_frame_foreach_ancillary_data(
		in_frame,
		mbuf_coded_video_frame_ancillary_data_copier,
		*out_frame);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_foreach_ancillary_data",
			       -ret);
		goto out;
	}

	ret = mbuf_raw_video_frame_get_metadata(in_frame, &metadata);
	if (ret == 0) {
		ret = mbuf_coded_video_frame_set_metadata(*out_frame, metadata);
		if (ret < 0) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_set_metadata",
				       -ret);
			goto out;
		}
	} else if (ret == -ENOENT) {
		/* No metadata, nothing to do */
	} else {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_get_metadata", -ret);
		goto out;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		*out_frame,
		VENC_ANCILLARY_KEY_OUTPUT_TIME,
		&ts_us,
		sizeof(ts_us));
	if (ret < 0)
		VENC_LOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
			       -ret);

	ret = mbuf_coded_video_frame_finalize(*out_frame);
	if (ret < 0)
		VENC_LOG_ERRNO("mbuf_raw_video_frame_finalize", -ret);

out:
	err = mbuf_mem_unref(out_mem);
	if (err != 0)
		VENC_LOG_ERRNO("mbuf_mem_unref", -err);
	if (ret < 0 && *out_frame) {
		mbuf_coded_video_frame_unref(*out_frame);
		*out_frame = NULL;
	}
	if (metadata)
		vmeta_frame_unref(metadata);
	return ret;
}


static void venc_ffmpeg_complete_flush(struct venc_ffmpeg *self)
{
	/* Flush the encoder queue (just in case) */
	int ret = mbuf_raw_video_frame_queue_flush(self->enc_in_queue);
	if (ret < 0)
		VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_flush:enc", -ret);

	avcodec_flush_buffers(self->avcodec);
	atomic_store(&self->flushing, false);
	atomic_store(&self->flush_discard, false);

	/* Call the flush callback on the loop */
	char message = VENC_MSG_FLUSH;
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		VENC_LOG_ERRNO("mbox_push", -ret);
}


static int venc_ffmpeg_start_flush(struct venc_ffmpeg *self)
{
	int ret;

	if (atomic_load(&self->flush_discard)) {
		/* Flush the input queue */
		ret = mbuf_raw_video_frame_queue_flush(self->in_queue);
		if (ret < 0) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_flush:input",
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

	/* Push NULL to enter draining mode */
	ret = avcodec_send_frame(self->avcodec, NULL);
	/*
	 * AVERROR_EOF and -EAGAIN are not treated as errors here.
	 * The encoder can return AVERROR_EOF if it is already flushed.
	 * The encoder can return -EAGAIN if its input queue is filled (i.e. it
	 * is waiting for an avcodec_receive_packet call), but it will still
	 * register the flush request.
	 */
	if (ret < 0 && ret != AVERROR_EOF && ret != -EAGAIN) {
		VENC_LOG_ERRNO("avcodec_send_frame", -ret);
		return ret;
	}

	atomic_store(&self->flush_requested, false);
	atomic_store(&self->flushing, true);

	/* Already flushed, just call flush done */
	if (ret == AVERROR_EOF)
		venc_ffmpeg_complete_flush(self);

	return ret;
}


static int venc_ffmpeg_buffer_push_one(struct venc_ffmpeg *self,
				       struct mbuf_raw_video_frame *in_frame)
{
	int ret = 0, err;
	struct vdef_raw_frame in_info;
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;
	const void *frame_data = NULL;
	size_t frame_len;
	size_t plane_count = 0;
	const void *plane_data;
	const uint8_t *in_data;
	size_t len;

	ret = mbuf_raw_video_frame_get_frame_info(in_frame, &in_info);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		goto out;
	}

	if (!vdef_raw_format_intersect(&in_info.format,
				       self->backend->supported_formats,
				       self->backend->nb_supported_formats)) {
		ret = -ENOSYS;
		VENC_LOG_ERRNO(
			"unsupported format: " VDEF_RAW_FORMAT_TO_STR_FMT,
			-ret,
			VDEF_RAW_FORMAT_TO_STR_ARG(&in_info.format));
		goto out;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	if (vdef_raw_format_cmp(&in_info.format, &vdef_gray))
		plane_count = 1;
	else if (vdef_raw_format_cmp(&in_info.format, &vdef_nv12))
		plane_count = 2;
	else if (vdef_raw_format_cmp(&in_info.format, &vdef_i420))
		plane_count = 3;

	for (size_t i = 0; i < AV_NUM_DATA_POINTERS; i++) {
		self->avframe->data[i] = NULL;
		self->avframe->linesize[i] = 0;
	}

	for (size_t i = 0; i < plane_count; i++) {
		ret = mbuf_raw_video_frame_get_plane(
			in_frame, i, &plane_data, &len);
		if (ret == 0) {
			in_data = plane_data;
			self->avframe->data[i] = (uint8_t *)in_data;
			self->avframe->linesize[i] = in_info.plane_stride[i];
			mbuf_raw_video_frame_release_plane(
				in_frame, i, plane_data);
		}
		if (ret < 0) {
			/* TODO: don't forget to drop the frame
			 * otherwise it remains in the queue. */
			VENC_LOG_ERRNO(
				"mbuf_raw_video_frame_get_plane(%zu)", -ret, i);
			return ret;
		}
	}
	for (size_t i = plane_count;
	     i < (plane_count + self->dummy_uv_plane.count);
	     i++) {
		/* Set dummy UV plane */
		self->avframe->data[i] = (uint8_t *)self->dummy_uv_plane.buf;
		self->avframe->linesize[i] = self->dummy_uv_plane.stride;
	}

	ret = mbuf_raw_video_frame_get_packed_buffer(
		in_frame, &frame_data, &frame_len);
	if (ret != 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_get_packed_buffer",
			       -ret);
		goto out;
	}

	self->avframe->width = in_info.info.resolution.width;
	self->avframe->height = in_info.info.resolution.height;
	self->avframe->format = format_to_av_pixel_format(&self->input_format);
	self->avframe->extended_data = self->avframe->data;
	self->avframe->pts = in_info.info.timestamp;
	if (atomic_load(&self->insert_idr)) {
		self->avframe->pict_type = AV_PICTURE_TYPE_I;
		atomic_store(&self->insert_idr, false);
	} else {
		/* Reset flag */
		self->avframe->pict_type = AV_PICTURE_TYPE_NONE;
	}

	if (atomic_load(&self->update_bitrate)) {
		/* TODO */
		self->avcodec->bit_rate = self->dynconf.target_bitrate;
		atomic_store(&self->update_bitrate, false);
	} else {
		/* Reset flag */
		self->avframe->opaque = NULL;
	}

	ret = avcodec_send_frame(self->avcodec, self->avframe);
	if (ret < 0) {
		if (ret != -EAGAIN)
			VENC_LOG_ERRNO("avcodec_send_frame", -ret);
		goto out;
	}

	ret = mbuf_raw_video_frame_queue_push(self->enc_in_queue, in_frame);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_queue_push:encoder",
			       -ret);
		goto out;
	}

	err = mbuf_raw_video_frame_add_ancillary_buffer(
		in_frame,
		VENC_ANCILLARY_KEY_DEQUEUE_TIME,
		&ts_us,
		sizeof(ts_us));
	if (err != 0)
		VENC_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
			       -err);

out:
	if (ret != 0 && frame_data)
		mbuf_raw_video_frame_release_packed_buffer(in_frame,
							   frame_data);
	mbuf_raw_video_frame_unref(in_frame);
	return ret;
}


static void release_encoder_frame(struct mbuf_raw_video_frame *frame)
{
	/* The frame has its packed buffer referenced from the push_one
	 * function, so we need to release it here. */
	const void *frame_data;
	size_t frame_len;

	if (mbuf_raw_video_frame_get_packed_buffer(
		    frame, &frame_data, &frame_len) == 0) {
		/* Release is needed twice, because the frame packed buffer has
		 * been acquired in venc_ffmpeg_buffer_push_one() and here. */
		mbuf_raw_video_frame_release_packed_buffer(frame, frame_data);
		mbuf_raw_video_frame_release_packed_buffer(frame, frame_data);
	}
	mbuf_raw_video_frame_unref(frame);
}


static int venc_ffmpeg_discard_frame(struct venc_ffmpeg *self)
{
	int ret, avcodec_ret;
	struct mbuf_raw_video_frame *in_frame = NULL;
	struct vdef_raw_frame in_info;

	/* Dequeue the frame from the encoder and drop it */
	avcodec_ret = avcodec_receive_packet(self->avcodec, self->dummy_packet);
	if ((avcodec_ret == -EAGAIN) || (avcodec_ret == AVERROR_EOF)) {
		/* No frame available (all frames were output),
		 * do not dequeue the input buffer */
		if ((avcodec_ret == AVERROR_EOF) &&
		    atomic_load(&self->flushing))
			venc_ffmpeg_complete_flush(self);
		return avcodec_ret;
	} else if (avcodec_ret < 0) {
		VENC_LOG_ERRNO("avcodec_receive_packet", -avcodec_ret);
	}

	atomic_fetch_add(&self->base->counters.pulled, 1);

	/* Get the input buffer (non-blocking);
	 * drop input buffers until the correct timestamp is met,
	 * in case ffmpeg has internally dropped some frames */
	do {
		if (in_frame != NULL)
			release_encoder_frame(in_frame);
		ret = mbuf_raw_video_frame_queue_pop(self->enc_in_queue,
						     &in_frame);
		if ((ret < 0) || (in_frame == NULL)) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_pop:enc",
				       -ret);
			break;
		}
		ret = mbuf_raw_video_frame_get_frame_info(in_frame, &in_info);
		if (ret < 0) {
			VENC_LOG_ERRNO(
				"mbuf_raw_video_frame_get_frame_info:enc",
				-ret);
			break;
		}
	} while (self->dummy_packet->pts > (int64_t)in_info.info.timestamp);

	if (in_frame != NULL)
		release_encoder_frame(in_frame);

	return 0;
}


static int venc_ffmpeg_buffer_pop_all(struct venc_ffmpeg *self)
{
	int ret = 0, err, avcodec_ret;
	AVPacket *packet;
	struct mbuf_raw_video_frame *in_frame = NULL;
	struct mbuf_coded_video_frame *out_frame = NULL;
	struct vdef_raw_frame in_info;
	bool need_av_free = false;

	do {
		/* Discard frames if flush is in progress */
		if (atomic_load(&self->flushing) &&
		    atomic_load(&self->flush_discard)) {
			err = venc_ffmpeg_discard_frame(self);
			if (err == AVERROR_EOF)
				break;
			else
				continue;
		}

		packet = av_packet_alloc();
		if (packet == NULL) {
			ret = -ENOMEM;
			VENC_LOG_ERRNO("av_frame_alloc", -ret);
			break;
		}
		need_av_free = true;

		/* Dequeue the frame from the encoder */
		avcodec_ret = avcodec_receive_packet(self->avcodec, packet);
		if ((avcodec_ret == -EAGAIN) || (avcodec_ret == AVERROR_EOF)) {
			/* No frame available (all frames were output),
			 * do not dequeue the input buffer */
			if ((avcodec_ret == AVERROR_EOF) &&
			    atomic_load(&self->flushing))
				venc_ffmpeg_complete_flush(self);
			break;
		}

		atomic_fetch_add(&self->base->counters.pulled, 1);

		/* Get the input buffer (non-blocking);
		 * drop input buffers until the correct timestamp is met,
		 * in case ffmpeg has internally dropped some frames */
		do {
			if (in_frame != NULL)
				release_encoder_frame(in_frame);
			ret = mbuf_raw_video_frame_queue_pop(self->enc_in_queue,
							     &in_frame);
			if ((ret < 0) || (in_frame == NULL)) {
				VENC_LOG_ERRNO(
					"mbuf_raw_video_frame_queue_pop:enc",
					-ret);
				break;
			}
			ret = mbuf_raw_video_frame_get_frame_info(in_frame,
								  &in_info);
			if (ret < 0) {
				VENC_LOG_ERRNO(
					"mbuf_raw_video_frame_get_frame_info:enc",
					-ret);
				break;
			}
		} while (packet->pts > (int64_t)in_info.info.timestamp);

		if (ret < 0 || in_frame == NULL)
			break;

		if (avcodec_ret < 0) {
			/* Decoding error,
			 * unref the input buffer */
			ret = avcodec_ret;
			VENC_LOG_ERRNO("avcodec_receive_packet", -avcodec_ret);
			break;
		}

		/* Set the metadata */
		ret = venc_ffmpeg_set_frame_metadata(
			self, in_frame, &out_frame, packet);
		if (ret < 0)
			break;
		need_av_free = false;

		/* Push the frame */
		ret = mbuf_coded_video_frame_queue_push(self->enc_out_queue,
							out_frame);
		if (ret < 0)
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_push:output",
				       -ret);

		/* Unref the buffers */
		release_encoder_frame(in_frame);
		in_frame = NULL;
		mbuf_coded_video_frame_unref(out_frame);
		out_frame = NULL;
	} while ((!atomic_load(&self->flush_requested) &&
		  !atomic_load(&self->should_stop)) ||
		 atomic_load(&self->flushing));

	if (need_av_free)
		av_packet_free(&packet);

	return ret;
}


static void check_input_queue(struct venc_ffmpeg *self)
{
	int ret;
	struct mbuf_raw_video_frame *frame;

	ret = mbuf_raw_video_frame_queue_peek(self->in_queue, &frame);
	while (ret == 0) {
		/* Frame skipping in case of decimation */
		if (atomic_load(&self->base->counters.pushed) %
			    self->dynconf.decimation !=
		    0) {
			mbuf_raw_video_frame_unref(frame);
		} else {
			/* Push the input frame */
			ret = venc_ffmpeg_buffer_push_one(self, frame);
			if (ret < 0) {
				if (ret == -EAGAIN) {
					ret = -ENOSPC;
					break;
				}
				VENC_LOG_ERRNO("venc_ffmpeg_buffer_push_one",
					       -ret);
			}
		}

		atomic_fetch_add(&self->base->counters.pushed, 1);

		/* Pop the frame for real */
		ret = mbuf_raw_video_frame_queue_pop(self->in_queue, &frame);
		if (ret < 0) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_pop", -ret);
			break;
		}
		mbuf_raw_video_frame_unref(frame);
		/* Once flush is requested, frames shall be pushed and popped
		 * one by one to avoid loosing the last one. */
		if (!atomic_load(&self->flush_requested)) {
			/* Peek the next frame */
			ret = mbuf_raw_video_frame_queue_peek(self->in_queue,
							      &frame);
		} else {
			/* Exit the loop to make sure only one frame
			 * is processed */
			ret = -ENOSPC;
		}
	}
	if (ret != -EAGAIN && ret != -ENOSPC)
		VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_peek", -ret);
	if (atomic_load(&self->flush_requested) && ret == -EAGAIN) {
		/* Flush without discarding frames */
		ret = venc_ffmpeg_start_flush(self);
		if (ret < 0 && ret)
			VENC_LOG_ERRNO("venc_ffmpeg_start_flush", -ret);
	}
	/* Pop output frames */
	ret = venc_ffmpeg_buffer_pop_all(self);
	if (ret < 0 && ret != -EAGAIN)
		VENC_LOG_ERRNO("venc_ffmpeg_buffer_pop_all", -ret);
}


static void input_event_cb(struct pomp_evt *evt, void *userdata)
{
	struct venc_ffmpeg *self = userdata;
	check_input_queue(self);
}


static void *encoder_thread(void *ptr)
{
	int ret, timeout;
	struct venc_ffmpeg *self = ptr;
	struct pomp_loop *loop = NULL;
	struct pomp_evt *in_queue_evt = NULL;
	char message;

	ret = pthread_setname_np(pthread_self(), "venc_ffmpeg");
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
			ret = venc_ffmpeg_start_flush(self);
			if (ret < 0)
				VENC_LOG_ERRNO("venc_ffmpeg_start_flush", -ret);
			/* Don't exit thread here, let the encoder check the
			 * input queue and pop all frames to exit properly */
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
	return nb_supported_encodings;
}


static int get_supported_input_formats(enum vdef_encoding encoding,
				       const struct vdef_raw_format **formats)
{
	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	struct venc_ffmpeg_backend *backend =
		get_preferred_backend_by_encoding(encoding);
	if (backend == NULL)
		return -ENOSYS;

	*formats = backend->supported_formats;
	return backend->nb_supported_formats;
}


static int copy_implem_cfg(const struct venc_config_impl *impl_cfg,
			   struct venc_config_impl **ret_obj)
{
	struct venc_config_ffmpeg *specific =
		(struct venc_config_ffmpeg *)impl_cfg;
	struct venc_config_ffmpeg *copy = NULL;
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
	struct venc_config_ffmpeg *specific =
		(struct venc_config_ffmpeg *)impl_cfg;
	ULOG_ERRNO_RETURN_ERR_IF(specific == NULL, EINVAL);

	free((void *)specific->preset);
	free((void *)specific->tune);
	free((void *)specific);


	return 0;
}


static int flush(struct venc_encoder *base, int discard)
{
	struct venc_ffmpeg *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	atomic_store(&self->flush_requested, true);
	atomic_store(&self->flush_discard, discard);

	return 0;
}


static int stop(struct venc_encoder *base)
{
	struct venc_ffmpeg *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	/* Stop the encoding thread */
	atomic_store(&self->should_stop, true);

	return 0;
}


static int destroy(struct venc_encoder *base)
{
	int err;
	struct venc_ffmpeg *self;

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

	if (self->avcodec != NULL)
		avcodec_free_context(&self->avcodec);
	if (self->dummy_packet != NULL)
		av_packet_free(&self->dummy_packet);
	if (self->avframe != NULL)
		av_frame_free(&self->avframe);

	err = pomp_loop_idle_remove_by_cookie(base->loop, self);
	if (err < 0)
		VENC_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	/* Delete nalus */
	if (self->nalus != NULL)
		free(self->nalus);

	if (self->dummy_uv_plane.buf != NULL)
		free(self->dummy_uv_plane.buf);

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
	struct venc_ffmpeg *self = userdata;

	VENC_LOG_ERRNO_RETURN_ERR_IF(self == NULL, false);
	VENC_LOG_ERRNO_RETURN_ERR_IF(self->backend == NULL, false);

	if (atomic_load(&self->flushing) || atomic_load(&self->should_stop))
		return false;

	ret = mbuf_raw_video_frame_get_frame_info(frame, &info);
	if (ret != 0)
		return false;

	/* Pass default filters first */
	if (!venc_default_input_filter_internal(
		    self->base,
		    frame,
		    &info,
		    self->backend->supported_formats,
		    self->backend->nb_supported_formats))
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


static int configure_attributes_h264(struct venc_ffmpeg *self,
				     const struct venc_config *config)
{
	float framerate;

	VENC_LOG_ERRNO_RETURN_ERR_IF(config == NULL, EINVAL);
	VENC_LOG_ERRNO_RETURN_ERR_IF(
		vdef_frac_is_null(&config->input.info.framerate), EINVAL);
	VENC_LOG_ERRNO_RETURN_ERR_IF(config->h264.decimation == 0, EINVAL);

	framerate = (float)config->input.info.framerate.num /
		    (float)config->input.info.framerate.den /
		    (float)config->h264.decimation;

	self->attrs.profile_str = profile_to_av(
		config->encoding, self->backend->type, config->h264.profile, 8);
	self->attrs.level_str = level_to_av(
		config->encoding, self->backend->type, config->h264.level);

	self->attrs.rc = config->h264.rate_control;
	(void)rate_control_to_av(self->backend->type,
				 self->attrs.rc,
				 false,
				 false,
				 &self->attrs.rc_key_str,
				 &self->attrs.rc_val_str);

	self->attrs.coder_str = entroy_coding_to_av(
		self->backend->type, config->h264.entropy_coding);

	self->attrs.rc = config->h264.rate_control;

	self->attrs.use_intra_refresh = (config->h264.intra_refresh ==
					 VENC_INTRA_REFRESH_VERTICAL_SCAN);
	self->attrs.gop =
		(self->attrs.use_intra_refresh)
			? (config->h264.intra_refresh_period)
			: (roundf(framerate * config->h264.gop_length_sec) - 1);

	self->attrs.max_bitrate = config->h264.max_bitrate;
	self->attrs.target_bitrate = config->h264.target_bitrate;
	self->attrs.decimation = config->h264.decimation;

	self->attrs.min_qp = config->h264.min_qp;
	self->attrs.max_qp = config->h264.max_qp;

	return 0;
}


static int configure_attributes_h265(struct venc_ffmpeg *self,
				     const struct venc_config *config)
{
	float framerate;

	VENC_LOG_ERRNO_RETURN_ERR_IF(config == NULL, EINVAL);
	VENC_LOG_ERRNO_RETURN_ERR_IF(
		vdef_frac_is_null(&config->input.info.framerate), EINVAL);
	VENC_LOG_ERRNO_RETURN_ERR_IF(config->h265.decimation == 0, EINVAL);

	framerate = (float)config->input.info.framerate.num /
		    (float)config->input.info.framerate.den /
		    (float)config->h265.decimation;

	self->attrs.profile_str = profile_to_av(
		config->encoding, self->backend->type, config->h265.profile, 8);
	self->attrs.level_str = level_to_av(
		config->encoding, self->backend->type, config->h265.level);

	self->attrs.rc = config->h265.rate_control;
	(void)rate_control_to_av(self->backend->type,
				 self->attrs.rc,
				 false,
				 false,
				 &self->attrs.rc_key_str,
				 &self->attrs.rc_val_str);

	self->attrs.gop = roundf(framerate * config->h265.gop_length_sec) - 1;

	self->attrs.max_bitrate = config->h265.max_bitrate;
	self->attrs.target_bitrate = config->h265.target_bitrate;
	self->attrs.decimation = config->h265.decimation;

	self->attrs.min_qp = config->h265.min_qp;
	self->attrs.max_qp = config->h265.max_qp;

	return 0;
}


static void
venc_set_av_opt_int(struct venc_ffmpeg *self, const char *key, int value)
{
	int ret;
	if (key == NULL)
		return;
	VENC_LOGD("%s: %s = %d", __func__, key, value);
	ret = av_opt_set_int(self->avcodec->priv_data, key, value, 0);
	if (ret < 0)
		VENC_LOG_ERRNO("av_set_opt", -ret);
}


static void
venc_set_av_opt(struct venc_ffmpeg *self, const char *key, const char *value)
{
	int ret;
	if (key == NULL || value == NULL)
		return;
	VENC_LOGD("%s: %s = '%s'", __func__, key, value);
	ret = av_opt_set(self->avcodec->priv_data, key, value, 0);
	if (ret < 0)
		VENC_LOG_ERRNO("av_set_opt", -ret);
}


static int create(struct venc_encoder *base)
{
	int ret = 0;
	struct venc_ffmpeg *self = NULL;
	unsigned int ver = avcodec_version();
	struct venc_config_ffmpeg *specific = NULL;
	const char *preset = NULL;
	const char *tune = NULL;
	enum AVPixelFormat pix_fmt = AV_PIX_FMT_NONE;
	bool found = false;
	struct mbuf_raw_video_frame_queue_args queue_args = {
		.filter = input_filter,
	};
	unsigned int min_qp = FUTILS_MAX(VENC_FFMPEG_MIN_QP, 1);
	unsigned int max_qp = FUTILS_MIN(VENC_FFMPEG_MAX_QP, 51);

	VENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	/* Check the configuration */
	for (size_t i = 0; i < nb_supported_encodings; i++) {
		if (base->config.encoding != supported_encodings[i])
			continue;
		found = true;
	}
	if (!found) {
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

	specific = (struct venc_config_ffmpeg *)venc_config_get_specific(
		&base->config, VENC_ENCODER_IMPLEM_FFMPEG);

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->base = base;
	base->derived = self;

	queue_args.filter_userdata = self;

	/* Find the encoder */
	av_log_set_level(AV_LOG_VERBOSE);
	av_log_set_callback(&av_log_cb);

	self->backend =
		get_preferred_backend_by_encoding(base->config.encoding);
	if ((self->backend == NULL) || (self->backend->codec == NULL)) {
		ret = -ENOENT;
		VENC_LOG_ERRNO("codec not found", -ret);
		goto error;
	}

	/* Initialize the encoder */
	self->avcodec = avcodec_alloc_context3(self->backend->codec);
	if (self->avcodec == NULL) {
		ret = -ENOMEM;
		VENC_LOG_ERRNO("avcodec_alloc_context3", -ret);
		goto error;
	}

	VENC_LOGI(
		"ffmpeg implementation - "
		"libavcodec version=%u.%u.%u - using '%s' %s %s encoding",
		(ver >> 16) & 0xFF,
		(ver >> 8) & 0xFF,
		ver & 0xFF,
		self->backend->name,
		vdef_encoding_to_str(base->config.encoding),
		(self->backend->is_hw) ? "HW" : "CPU");

	/* Configure encoding attributes */
	switch (base->config.encoding) {
	case VDEF_ENCODING_H264:
		ret = configure_attributes_h264(self, &base->config);
		if (ret != 0)
			goto error;
		break;
	case VDEF_ENCODING_H265:
		ret = configure_attributes_h265(self, &base->config);
		if (ret != 0)
			goto error;
		break;
	default:
		break;
	}

	if (vdef_raw_format_cmp(&base->config.input.format, &vdef_gray)) {
		/* Override vdef_gray format (unsupported) */
		self->input_format = self->backend->supported_formats[1];

		if (vdef_raw_format_cmp(&self->input_format, &vdef_i420)) {
			self->dummy_uv_plane.count = 2;
		} else if (vdef_raw_format_cmp(&self->input_format,
					       &vdef_nv12)) {
			self->dummy_uv_plane.count = 1;
		} else {
			ret = -EINVAL;
			VENC_LOG_ERRNO("unsupported format", -ret);
			goto error;
		}

		/* FFMPEG does not support grayscale input, create a dummy
		 * UV plane and send it with every frame */
		self->dummy_uv_plane.stride =
			base->config.input.info.resolution.width;
		self->dummy_uv_plane.len =
			base->config.input.info.resolution.width *
			base->config.input.info.resolution.height / 2;
		self->dummy_uv_plane.buf = malloc(self->dummy_uv_plane.len);
		if (self->dummy_uv_plane.buf == NULL) {
			ret = -ENOMEM;
			VENC_LOG_ERRNO("malloc", -ret);
			goto error;
		}
		memset(self->dummy_uv_plane.buf,
		       0x80,
		       self->dummy_uv_plane.len);
	} else {
		self->input_format = base->config.input.format;
	}

	pix_fmt = format_to_av_pixel_format(&self->input_format);

	preset = (specific && specific->preset) ? specific->preset : "p4";
	tune = (specific && specific->tune) ? specific->tune : "ll";

	self->avcodec->pix_fmt = pix_fmt;
	/* Place global headers in extradata instead of every keyframe. */
	self->avcodec->flags = AV_CODEC_FLAG_GLOBAL_HEADER;
	self->avcodec->codec_type = AVMEDIA_TYPE_VIDEO;
	self->avcodec->thread_type = FF_THREAD_SLICE;
	switch (base->config.preferred_thread_count) {
	case 1:
		self->avcodec->thread_count = 1;
		self->avcodec->thread_type &= ~FF_THREAD_FRAME;
		break;
	case 0:
		self->avcodec->thread_count = VENC_FFMPEG_DEFAULT_THREAD_COUNT;
		break;
	default:
		self->avcodec->thread_count =
			base->config.preferred_thread_count;
		break;
	}

	venc_set_av_opt(self, "profile", self->attrs.profile_str);
	venc_set_av_opt(self, "level", self->attrs.level_str);

	switch (self->backend->type) {
	case VENC_FFMPEG_BACKEND_TYPE_OPENH264: {
		/* OpenH264-specific params */
		venc_set_av_opt_int(self, "allow_skip_frames", 0);
		break;
	}
	case VENC_FFMPEG_BACKEND_TYPE_H264_NVENC:
	case VENC_FFMPEG_BACKEND_TYPE_HEVC_NVENC: {
		/* TODO: NVENC-specific params */
		venc_set_av_opt(self, "preset", preset);
		venc_set_av_opt(self, "tune", tune);
		/* If forcing keyframes, force them as IDR frames (needed for
		 * request IDR API) */
		venc_set_av_opt_int(self, "forced-idr", 1);
		venc_set_av_opt_int(self, "nonref_p", 1);
		venc_set_av_opt(self, "b_ref_mode", "disabled");
		venc_set_av_opt_int(
			self, "intra-refresh", !!self->attrs.use_intra_refresh);
		break;
	}
	default:
		break;
	}

	self->avcodec->width = base->config.input.info.resolution.width;
	self->avcodec->height = base->config.input.info.resolution.height;
	self->avcodec->time_base.num = self->avcodec->framerate.num =
		base->config.input.info.framerate.num;
	self->avcodec->time_base.den = self->avcodec->framerate.den =
		base->config.input.info.framerate.den;
	self->avcodec->max_b_frames = 0;
	self->avcodec->gop_size = self->avcodec->keyint_min = self->attrs.gop;

	switch (base->config.encoding) {
	case VDEF_ENCODING_H264: {
		self->avcodec->slices =
			base->config.h264.slice_size_mbrows > 0
				? (base->mb_height /
				   base->config.h264.slice_size_mbrows)
				: 0;
		venc_set_av_opt(self, "coder", self->attrs.coder_str);
		break;
	}
	case VDEF_ENCODING_H265:
	default:
		break;
	}

	venc_set_av_opt(self, self->attrs.rc_key_str, self->attrs.rc_val_str);

	switch (self->attrs.rc) {
	default:
	case VENC_RATE_CONTROL_CBR:
	case VENC_RATE_CONTROL_VBR:
		self->avcodec->rc_max_rate = self->attrs.max_bitrate;
		self->avcodec->bit_rate = self->attrs.target_bitrate;
		break;
	case VENC_RATE_CONTROL_CQ:
		venc_set_av_opt_int(self, "qp", self->attrs.qp);
		break;
	}

	if (self->attrs.min_qp != 0 || self->attrs.max_qp != 0) {
		min_qp = FUTILS_MAX(min_qp, self->attrs.min_qp);
		max_qp = FUTILS_MIN(max_qp, self->attrs.max_qp);
	}

	self->avcodec->qmin = min_qp;
	self->avcodec->qmax = max_qp;

	self->dynconf = (struct venc_dyn_config){
		.decimation = self->attrs.decimation,
		.target_bitrate = self->attrs.target_bitrate,
		.qp = self->attrs.qp,
	};

	self->avframe = av_frame_alloc();
	if (self->avframe == NULL) {
		ret = -ENOMEM;
		VENC_LOG_ERRNO("av_frame_alloc", -ret);
		goto error;
	}

	self->dummy_packet = av_packet_alloc();
	if (self->dummy_packet == NULL) {
		ret = -ENOMEM;
		VENC_LOG_ERRNO("av_packet_alloc", -ret);
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

	/* Initialize the encoder */
	ret = avcodec_open2(self->avcodec, NULL, NULL);
	if (ret < 0) {
		VENC_LOG_ERRNO("avcodec_open2", -ret);
		goto error;
	}

	/* Save PS from extradata */
	ret = save_ps(
		self, self->avcodec->extradata, self->avcodec->extradata_size);
	if (ret < 0) {
		VENC_LOG_ERRNO("save_ps", -ret);
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
	struct venc_ffmpeg *self;
	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);
	self = (struct venc_ffmpeg *)base->derived;

	return self->in_queue;
}


static int get_dyn_config(struct venc_encoder *base,
			  struct venc_dyn_config *config)
{
	struct venc_ffmpeg *self;
	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);
	self = (struct venc_ffmpeg *)base->derived;

	*config = self->dynconf;

	return 0;
}


static int set_dyn_config(struct venc_encoder *base,
			  const struct venc_dyn_config *config)
{
	struct venc_ffmpeg *self;
	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);
	self = (struct venc_ffmpeg *)base->derived;

	/* QP, unsupported */
	if ((config->qp != 0) && (config->qp != self->dynconf.qp)) {
		VENC_LOGE("QP is not supported");
		return -ENOSYS;
	}

	/* Target bitrate */
	if ((config->target_bitrate != 0) &&
	    (config->target_bitrate != self->dynconf.target_bitrate)) {
		self->dynconf.target_bitrate = config->target_bitrate;
		atomic_store(&self->update_bitrate, true);
	}

	/* Decimation */
	if ((config->decimation != 0) &&
	    (config->decimation != self->dynconf.decimation)) {
		self->dynconf.decimation = config->decimation;
	}

	return 0;
}


static int request_idr(struct venc_encoder *base)
{
	struct venc_ffmpeg *self;
	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);
	self = (struct venc_ffmpeg *)base->derived;

	atomic_store(&self->insert_idr, true);

	return 0;
}


const struct venc_ops venc_ffmpeg_ops = {
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
