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

#define ULOG_TAG venc_videotoolbox
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);


#include "venc_videotoolbox_priv.h"


#define MAX_SUPPORTED_ENCODINGS 2
#define NB_SUPPORTED_FORMATS 3
static struct vdef_raw_format supported_formats[NB_SUPPORTED_FORMATS];
static enum vdef_encoding supported_encodings[MAX_SUPPORTED_ENCODINGS];
static int nb_supported_encodings;
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;
static void initialize_supported_formats(void)
{
	supported_formats[0] = vdef_nv12;
	supported_formats[1] = vdef_i420;

	CFMutableDictionaryRef buffer_attr = NULL;
	VTCompressionSessionRef compress_ref;
	OSStatus osstatus;
	size_t j = 0;
	enum vdef_encoding enc[] = {
		VDEF_ENCODING_H264,
		VDEF_ENCODING_H265,
		VDEF_ENCODING_UNKNOWN,
	};
	CMVideoCodecType codec[] = {
		kCMVideoCodecType_H264,
		kCMVideoCodecType_HEVC,
	};

	buffer_attr =
		CFDictionaryCreateMutable(kCFAllocatorDefault,
					  0,
					  &kCFTypeDictionaryKeyCallBacks,
					  &kCFTypeDictionaryValueCallBacks);

	for (size_t i = 0; enc[i] != VDEF_ENCODING_UNKNOWN; i++) {
		osstatus = VTCompressionSessionCreate(kCFAllocatorDefault,
						      1920,
						      1080,
						      codec[i],
						      NULL,
						      buffer_attr,
						      kCFAllocatorDefault,
						      NULL,
						      NULL,
						      &compress_ref);
		if (osstatus == noErr) {
			supported_encodings[j] = enc[i];
			j++;
			VTCompressionSessionInvalidate(compress_ref);
			CFRelease(compress_ref);
		}
	}

	nb_supported_encodings = j;
}


#define GET_SYM(prefix, fallback)                                              \
	do {                                                                   \
		CFStringRef *handle =                                          \
			(CFStringRef *)dlsym(RTLD_DEFAULT, #prefix #fallback); \
		if (!handle)                                                   \
			s_compat_keys.prefix##fallback = CFSTR(#fallback);     \
		else                                                           \
			s_compat_keys.prefix##fallback = *handle;              \
	} while (0)


static pthread_once_t compat_keys_is_init = PTHREAD_ONCE_INIT;
static void initialize_compat_keys(void)
{
	GET_SYM(kVTProfileLevel_, H264_Baseline_4_0);
	GET_SYM(kVTProfileLevel_, H264_Baseline_4_2);
	GET_SYM(kVTProfileLevel_, H264_Baseline_5_0);
	GET_SYM(kVTProfileLevel_, H264_Baseline_5_1);
	GET_SYM(kVTProfileLevel_, H264_Baseline_5_2);
	GET_SYM(kVTProfileLevel_, H264_Main_AutoLevel);
	GET_SYM(kVTProfileLevel_, H264_Main_4_2);
	GET_SYM(kVTProfileLevel_, H264_Main_5_1);
	GET_SYM(kVTProfileLevel_, H264_Main_5_2);
	GET_SYM(kVTProfileLevel_, H264_High_AutoLevel);
	GET_SYM(kVTProfileLevel_, H264_High_3_0);
	GET_SYM(kVTProfileLevel_, H264_High_3_1);
	GET_SYM(kVTProfileLevel_, H264_High_3_2);
	GET_SYM(kVTProfileLevel_, H264_High_4_0);
	GET_SYM(kVTProfileLevel_, H264_High_4_1);
	GET_SYM(kVTProfileLevel_, H264_High_4_2);
	GET_SYM(kVTProfileLevel_, H264_High_5_1);
	GET_SYM(kVTProfileLevel_, H264_High_5_2);
	GET_SYM(kVTProfileLevel_, H264_Extended_AutoLevel);
	GET_SYM(kVTProfileLevel_, H264_Extended_5_0);

	GET_SYM(kVTProfileLevel_, HEVC_Main_AutoLevel);
	GET_SYM(kVTProfileLevel_, HEVC_Main10_AutoLevel);

	GET_SYM(kVTH264EntropyMode_, CAVLC);
	GET_SYM(kVTH264EntropyMode_, CABAC);

	GET_SYM(kCVImageBufferTransferFunction_, SMPTE_ST_2084_PQ);
	GET_SYM(kCVImageBufferTransferFunction_, ITU_R_2100_HLG);
	GET_SYM(kCVImageBufferTransferFunction_, sRGB);
}


static void call_flush_done(void *userdata)
{
	struct venc_videotoolbox *self = userdata;

	venc_call_flush_cb(self->base);
}


static void call_stop_done(void *userdata)
{
	struct venc_videotoolbox *self = userdata;

	atomic_store(&self->is_stopped, true);

	venc_call_stop_cb(self->base);
}


static void encoder_error(struct venc_videotoolbox *self, int error)
{
	venc_call_frame_output_cb(self->base, error, NULL);
}


static void
copy_ps(uint8_t **dst, size_t *dst_size, const uint8_t *src, size_t src_size)
{
	ULOG_ERRNO_RETURN_IF(dst == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(dst_size == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(src == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(src_size == 0, EINVAL);

	free(*dst);
	*dst_size = src_size;
	*dst = calloc(1, src_size);
	if (*dst == NULL) {
		ULOG_ERRNO("calloc", -ENOMEM);
		return;
	}
	memcpy(*dst, src, src_size);
}


static void mbox_cb(int fd, uint32_t revents, void *userdata)
{
	struct venc_videotoolbox *self = userdata;
	int ret, err;
	struct venc_videotoolbox_message message;

	while (true) {
		/* Read from the mailbox */
		ret = mbox_peek(self->mbox, &message);
		if (ret < 0) {
			if (ret != -EAGAIN)
				VENC_LOG_ERRNO("mbox_peek", -ret);
			break;
		}

		switch (message.type) {
		case VENC_VIDEOTOOLBOX_MESSAGE_TYPE_FLUSH:
			err = pomp_loop_idle_add_with_cookie(
				self->base->loop, call_flush_done, self, self);
			if (err < 0)
				VENC_LOG_ERRNO("pomp_loop_idle_add_with_cookie",
					       -err);
			break;
		case VENC_VIDEOTOOLBOX_MESSAGE_TYPE_STOP:
			err = pomp_loop_idle_add_with_cookie(
				self->base->loop, call_stop_done, self, self);
			if (err < 0)
				VENC_LOG_ERRNO("pomp_loop_idle_add_with_cookie",
					       -err);
			break;
		case VENC_VIDEOTOOLBOX_MESSAGE_TYPE_ERROR:
			encoder_error(self, message.error);
			break;
		default:
			VENC_LOGE("unknown message type: %d", message.type);
			break;
		}
	}
}


static CFMutableDictionaryRef buffer_attr_create(struct venc_videotoolbox *self)
{
	int err = 0;
	CFMutableDictionaryRef buffer_attr = NULL, io_surface_properties = NULL;
	CFNumberRef pix_fmt = NULL;

	buffer_attr =
		CFDictionaryCreateMutable(kCFAllocatorDefault,
					  4,
					  &kCFTypeDictionaryKeyCallBacks,
					  &kCFTypeDictionaryValueCallBacks);
	if (buffer_attr == NULL) {
		err = -ENOMEM;
		VENC_LOG_ERRNO("CFDictionaryCreateMutable", -err);
		goto out;
	}

	/* Empty means use default */
	io_surface_properties =
		CFDictionaryCreateMutable(kCFAllocatorDefault,
					  0,
					  &kCFTypeDictionaryKeyCallBacks,
					  &kCFTypeDictionaryValueCallBacks);
	if (io_surface_properties == NULL) {
		err = -ENOMEM;
		VENC_LOG_ERRNO("CFDictionaryCreateMutable", -err);
		goto out;
	}

	pix_fmt = CFNumberCreate(kCFAllocatorDefault,
				 kCFNumberSInt32Type,
				 &self->vt_pixel_format);
	if (pix_fmt == NULL) {
		err = -ENOMEM;
		VENC_LOG_ERRNO("CFNumberCreate", -err);
		goto out;
	}

	CFDictionarySetValue(buffer_attr,
			     kCVPixelBufferIOSurfacePropertiesKey,
			     io_surface_properties);
	CFDictionarySetValue(
		buffer_attr, kCVPixelBufferPixelFormatTypeKey, pix_fmt);

out:
	if (pix_fmt != NULL)
		CFRelease(pix_fmt);
	if (io_surface_properties != NULL)
		CFRelease(io_surface_properties);
	if ((err != 0) && (buffer_attr != NULL))
		CFRelease(buffer_attr);
	return (err == 0) ? buffer_attr : NULL;
}


static void out_queue_evt_cb(struct pomp_evt *evt, void *userdata)
{
	int err;
	struct venc_videotoolbox *self = userdata;
	struct mbuf_coded_video_frame *out_frame = NULL;

	if (!atomic_load(&self->ps_stored))
		return;

	while (true) {
		err = mbuf_coded_video_frame_queue_pop(self->out_queue,
						       &out_frame);
		if (err == -EAGAIN) {
			return;
		} else if (err < 0) {
			VENC_LOG_ERRNO(
				"mbuf_coded_video_frame_queue_pop:output",
				-err);
			return;
		}
		struct vdef_coded_frame out_info = {};
		err = mbuf_coded_video_frame_get_frame_info(out_frame,
							    &out_info);
		if (err < 0) {
			VENC_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info",
				       -err);
		}
		if (!atomic_load(&self->flush_discard))
			venc_call_frame_output_cb(self->base, 0, out_frame);
		else
			VENC_LOGD("discarding frame: %d", out_info.info.index);
		err = mbuf_coded_video_frame_unref(out_frame);
		if (err < 0)
			VENC_LOG_ERRNO("mbuf_coded_video_frame_unref", -err);
	}
}


static int do_flush(struct venc_videotoolbox *self)
{
	int ret;

	if (atomic_load(&self->flush_discard)) {
		/* Flush the queues */
		ret = mbuf_raw_video_frame_queue_flush(self->in_queue);
		if (ret < 0) {
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_flush:input",
				       -ret);
			return ret;
		}
		ret = mbuf_coded_video_frame_queue_flush(self->out_queue);
		if (ret < 0) {
			VENC_LOG_ERRNO(
				"mbuf_coded_video_frame_queue_flush:output",
				-ret);
			return ret;
		}
	}

	atomic_store(&self->flushing, true);
	if (self->compress_ref) {
		ret = VTCompressionSessionCompleteFrames(self->compress_ref,
							 kCMTimeIndefinite);
		if (ret)
			VENC_LOG_ERRNO("VTCompressionSessionCompleteFrames",
				       -ret);
	}
	atomic_store(&self->flushing, false);

	/* Call the flush callback on the loop */
	struct venc_videotoolbox_message message = {
		.type = VENC_VIDEOTOOLBOX_MESSAGE_TYPE_FLUSH,
	};
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		VENC_LOG_ERRNO("mbox_push", -ret);

	atomic_store(&self->flush, false);

	return 0;
}


static void cmbr_mbuf_release(void *data, size_t len, void *userdata)
{
	CMBlockBufferRef ref = userdata;
	CFRelease(ref);
}


static void frame_release(struct mbuf_coded_video_frame *frame, void *userdata)
{
	struct venc_videotoolbox *self = userdata;

	venc_call_pre_release_cb(self->base, frame);
}


static int set_frame_metadata(struct venc_videotoolbox *self,
			      struct mbuf_raw_video_frame *in_frame,
			      struct mbuf_coded_video_frame **out_frame,
			      CMBlockBufferRef ref)
{
	int ret, err;
	struct vdef_raw_frame in_info;
	struct vdef_coded_frame out_info = {};
	uint8_t *data, *start;
	size_t len, offset = 0, nalu_len;
	uint8_t start_code[] = {0, 0, 0, 1};
	enum vdef_coded_data_format format =
		self->base->config.output.preferred_format;
	struct mbuf_mem *mem = NULL;
	void *ptr;
	OSStatus status;
	struct vmeta_frame *metadata = NULL;
	struct vdef_nalu out_nalu;

	struct mbuf_coded_video_frame_cbs frame_cbs = {
		.pre_release = frame_release,
		.pre_release_userdata = (void *)self,
	};

	/* Mem creation */
	ref = (CMBlockBufferRef)CFRetain(ref);

	status = CMBlockBufferGetDataPointer(ref, 0, NULL, NULL, (char **)&ptr);
	if (status != noErr) {
		VENC_LOG_ERRNO("CMBlockBufferGetDataPointer status=%d",
			       EPROTO,
			       (int)status);
	}
	start = data = ptr;

	len = CMBlockBufferGetDataLength(ref);

	ret = mbuf_mem_generic_wrap(ptr, len, cmbr_mbuf_release, ref, &mem);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_mem_generic_wrap", -ret);
		goto out;
	}

	/* Frame creation */
	ret = mbuf_raw_video_frame_get_frame_info(in_frame, &in_info);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		goto out;
	}

	out_info.info = in_info.info;
	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264:
		if (format == VDEF_CODED_DATA_FORMAT_BYTE_STREAM)
			out_info.format = vdef_h264_byte_stream;
		else
			out_info.format = vdef_h264_avcc;
		break;
	case VDEF_ENCODING_H265:
		if (format == VDEF_CODED_DATA_FORMAT_BYTE_STREAM)
			out_info.format = vdef_h265_byte_stream;
		else
			out_info.format = vdef_h265_hvcc;
		break;
	default:
		break;
	}

	out_info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;
	out_info.layer = 0;

	while (offset < len) {
		memcpy(&nalu_len, data, sizeof(uint32_t));
		nalu_len = ntohl(nalu_len);

		if (self->base->config.encoding == VDEF_ENCODING_H264) {
			if ((*(data + 4) & 0x1F) == H264_NALU_TYPE_SLICE_IDR) {
				out_info.type = VDEF_CODED_FRAME_TYPE_IDR;
				break;
			}
		} else if (self->base->config.encoding == VDEF_ENCODING_H265)
			if ((((*(data + 4) & 0x7E) >> 1) ==
			     H265_NALU_TYPE_IDR_W_RADL) ||
			    (((*(data + 4) & 0x7E) >> 1) ==
			     H265_NALU_TYPE_IDR_N_LP)) {
				out_info.type = VDEF_CODED_FRAME_TYPE_IDR;
				break;
			}

		data += 4 + nalu_len;
		offset += 4 + nalu_len;
	}

	ret = mbuf_coded_video_frame_new(&out_info, out_frame);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_new", -ret);
		goto out;
	}
	ret = mbuf_coded_video_frame_set_callbacks(*out_frame, &frame_cbs);
	if (ret < 0)
		VENC_LOG_ERRNO("mbuf_coded_video_frame_set_callbacks", -ret);

	/* Frame metadata */
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
			VENC_LOG_ERRNO("mbuf_coded_video_frame_set_metadata",
				       -ret);
			goto out;
		}
	} else if ((ret < 0) && (ret != -ENOENT)) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_get_metadata", -ret);
		goto out;
	}

	if (self->base->config.encoding == VDEF_ENCODING_H264) {
		/* Add generated NAL units */
		/* TODO: Set SPS and PPS in base before first frame */
		ret = venc_h264_generate_nalus(
			self->base, *out_frame, &out_info);
		if (ret < 0) {
			VENC_LOG_ERRNO("venc_h264_generate_nalus", -ret);
			goto out;
		}
	} else if (self->base->config.encoding == VDEF_ENCODING_H265) {
		/* Add generated NAL units */
		/* TODO: Set VPS, SPS and PPS in base before first frame */
		ret = venc_h265_generate_nalus(
			self->base, *out_frame, &out_info);
		if (ret < 0) {
			VENC_LOG_ERRNO("venc_h265_generate_nalus", -ret);
			goto out;
		}
	}

	data = start;
	offset = 0;
	while (offset < len) {
		memcpy(&nalu_len, data, sizeof(uint32_t));
		nalu_len = ntohl(nalu_len);

		if (format == VDEF_CODED_DATA_FORMAT_BYTE_STREAM)
			memcpy(data, start_code, sizeof(start_code));

		if (self->base->config.encoding == VDEF_ENCODING_H264) {
			out_nalu.h264.type = (*(data + 4) & 0x1F);
			if ((out_nalu.h264.type == H264_NALU_TYPE_AUD) ||
			    (out_nalu.h264.type == H264_NALU_TYPE_SPS) ||
			    (out_nalu.h264.type == H264_NALU_TYPE_PPS) ||
			    (out_nalu.h264.type == H264_NALU_TYPE_SEI)) {
				data += 4 + nalu_len;
				offset += 4 + nalu_len;
				continue;
			}
		} else if (self->base->config.encoding == VDEF_ENCODING_H265) {
			out_nalu.h265.type = ((*(data + 4) & 0x7E) >> 1);
			if ((out_nalu.h265.type == H265_NALU_TYPE_AUD_NUT) ||
			    (out_nalu.h265.type == H265_NALU_TYPE_VPS_NUT) ||
			    (out_nalu.h265.type == H265_NALU_TYPE_SPS_NUT) ||
			    (out_nalu.h265.type == H265_NALU_TYPE_PPS_NUT) ||
			    (out_nalu.h265.type ==
			     H265_NALU_TYPE_PREFIX_SEI_NUT)) {
				data += 4 + nalu_len;
				offset += 4 + nalu_len;
				continue;
			}
		}

		out_nalu.size = nalu_len + 4;
		out_nalu.importance = 0;
		ret = mbuf_coded_video_frame_add_nalu(
			*out_frame, mem, offset, &out_nalu);
		if (ret < 0) {
			VENC_LOG_ERRNO("mbuf_coded_video_frame_add_nalu", -ret);
			goto out;
		}

		data += 4 + nalu_len;
		offset += 4 + nalu_len;
	}

out:
	err = mbuf_mem_unref(mem);
	if (err != 0)
		VENC_LOG_ERRNO("mbuf_mem_unref", -err);

	return ret;
}


static int set_h264_ps(struct venc_videotoolbox *self,
		       CMVideoFormatDescriptionRef format)
{
	int ret;
	OSStatus status;
	size_t ps_count;
	uint8_t **ps_bufs[] = {
		&self->base->h264.sps,
		&self->base->h264.pps,
	};
	size_t *ps_sizes[] = {
		&self->base->h264.sps_size,
		&self->base->h264.pps_size,
	};
	size_t expected_ps_count = SIZEOF_ARRAY(ps_bufs);

	VENC_LOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	pthread_mutex_lock(&self->ps_lock);

	status = CMVideoFormatDescriptionGetH264ParameterSetAtIndex(
		format, 0, NULL, NULL, &ps_count, NULL);
	if (status != noErr) {
		ret = -EAGAIN;
		VENC_LOGE("no PS available");
		goto end;
	}
	if (ps_count != expected_ps_count) {
		ret = -ENOSYS;
		VENC_LOG_ERRNO("invalid PS count (expected %zu, got %zu)",
			       -ret,
			       expected_ps_count,
			       ps_count);
		goto end;
	}

	for (size_t i = 0; i < expected_ps_count; i++) {
		const uint8_t *ps = NULL;
		size_t ps_size;
		status = CMVideoFormatDescriptionGetH264ParameterSetAtIndex(
			format, i, &ps, &ps_size, NULL, NULL);
		if (status != noErr) {
			ret = -EPROTO;
			VENC_LOG_ERRNO(
				"CMVideoFormatDescriptionGetH264ParameterSetAtIndex",
				-ret);
			goto end;
		}
		copy_ps(ps_bufs[i], ps_sizes[i], ps, ps_size);
	}

	/* Initialize the H.264 writer */
	ret = venc_h264_writer_new(self->base->h264.sps,
				   self->base->h264.sps_size,
				   self->base->h264.pps,
				   self->base->h264.pps_size,
				   &self->base->h264.ctx);
	if (ret < 0) {
		VENC_LOG_ERRNO("venc_h264_writer_new", -ret);
		goto end;
	}
	ret = venc_h264_patch_ps(self->base);
	if (ret < 0) {
		VENC_LOG_ERRNO("venc_h264_patch_ps", -ret);
		goto end;
	}

end:
	pthread_mutex_unlock(&self->ps_lock);

	return ret;
}


static int set_h265_ps(struct venc_videotoolbox *self,
		       CMVideoFormatDescriptionRef format)
{
	int ret = 0;
	OSStatus status;

	VENC_LOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	pthread_mutex_lock(&self->ps_lock);

	if (__builtin_available(iOS 11.0, macOS 10.13, tvos 11.0, *)) {
		size_t ps_count;
		uint8_t **ps_bufs[] = {
			&self->base->h265.vps,
			&self->base->h265.sps,
			&self->base->h265.pps,
		};
		size_t *ps_sizes[] = {
			&self->base->h265.vps_size,
			&self->base->h265.sps_size,
			&self->base->h265.pps_size,
		};
		size_t expected_ps_count = SIZEOF_ARRAY(ps_bufs);

		status = CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(
			format, 0, NULL, NULL, &ps_count, NULL);
		if (status != noErr) {
			ret = -EAGAIN;
			VENC_LOGE("no PS available");
			goto end;
		}
		if (ps_count != expected_ps_count) {
			ret = -ENOSYS;
			VENC_LOG_ERRNO(
				"invalid PS count (expected %zu, got %zu)",
				-ret,
				expected_ps_count,
				ps_count);
			goto end;
		}

		for (size_t i = 0; i < expected_ps_count; i++) {
			const uint8_t *ps = NULL;
			size_t ps_size;
			status =
				/* codecheck_ignore[LONG_LINE] */
				CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(
					format, i, &ps, &ps_size, NULL, NULL);
			if (status != noErr) {
				ret = -EPROTO;
				VENC_LOG_ERRNO(
					"CMVideoFormatDescriptionGetHEVCParameterSetAtIndex",
					-ret);
				goto end;
			}
			copy_ps(ps_bufs[i], ps_sizes[i], ps, ps_size);
		}

		/* Initialize the H.265 writer */
		ret = venc_h265_writer_new(self->base->h265.vps,
					   self->base->h265.vps_size,
					   self->base->h265.sps,
					   self->base->h265.sps_size,
					   self->base->h265.pps,
					   self->base->h265.pps_size,
					   &self->base->h265.ctx);
		if (ret < 0) {
			VENC_LOG_ERRNO("venc_h265_writer_new", -ret);
			goto end;
		}
		ret = venc_h265_patch_ps(self->base);
		if (ret < 0) {
			VENC_LOG_ERRNO("venc_h265_patch_ps", -ret);
			goto end;
		}
	}

end:
	pthread_mutex_unlock(&self->ps_lock);

	return ret;
}


static int set_ps(struct venc_videotoolbox *self,
		  CMSampleBufferRef sample_buffer)
{
	int ret = 0;
	CMVideoFormatDescriptionRef format;

	format = CMSampleBufferGetFormatDescription(sample_buffer);
	if (!format) {
		VENC_LOG_ERRNO("CMSampleBufferGetFormatDescription", EPROTO);
		return -EPROTO;
	}

	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264:
		ret = set_h264_ps(self, format);
		if (ret) {
			VENC_LOG_ERRNO("set_h264_ps", -ret);
			goto out;
		}
		atomic_store(&self->ps_stored, true);
		break;
	case VDEF_ENCODING_H265:
		ret = set_h265_ps(self, format);
		if (ret) {
			VENC_LOG_ERRNO("set_h265_ps", -ret);
			goto out;
		}
		atomic_store(&self->ps_stored, true);
		break;
	default:
		break;
	}

out:
	return ret;
}


static void frame_output_cb(void *outputCallbackRefCon,
			    void *sourceFrameRefCon,
			    OSStatus status,
			    VTEncodeInfoFlags infoFlags,
			    CMSampleBufferRef sampleBuffer)
{
	int ret, err;
	struct venc_videotoolbox *self = outputCallbackRefCon;
	struct mbuf_raw_video_frame *in_frame = sourceFrameRefCon;
	struct mbuf_coded_video_frame *out_frame = NULL;
	CMBlockBufferRef bbuf = NULL;
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;

	VENC_LOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	if (status != noErr) {
		VENC_LOG_ERRNO("encoder error %d", EPROTO, (int)status);
		goto out;
	}

	VENC_LOG_ERRNO_RETURN_IF(sampleBuffer == NULL, EINVAL);
	VENC_LOG_ERRNO_RETURN_IF(in_frame == NULL, EINVAL);

	self->base->counters.pulled++;

	/* Discard the buffer when flushing with frames discarding */
	if ((atomic_load(&self->flushing)) &&
	    (atomic_load(&self->flush_discard))) {
		VENC_LOGI("frame discarded (flushing)");
		goto out;
	}

	bbuf = CMSampleBufferGetDataBuffer(sampleBuffer);
	if (bbuf == NULL) {
		ret = -ENOSYS;
		VENC_LOG_ERRNO("CMSampleBufferGetDataBuffer", -ret);
		goto out;
	}

	if (!atomic_load(&self->ps_stored)) {
		ret = set_ps(self, sampleBuffer);
		if (ret < 0) {
			VENC_LOG_ERRNO("set_ps", -ret);
			goto out;
		}
	}

	/* Set the metadata */
	ret = set_frame_metadata(self, in_frame, &out_frame, bbuf);
	if (ret < 0) {
		VENC_LOG_ERRNO("set_frame_metadata", -ret);
		goto out;
	}

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

	ret = mbuf_coded_video_frame_queue_push(self->out_queue, out_frame);
	if (ret < 0)
		VENC_LOG_ERRNO("mbuf_coded_video_frame_queue_push", -ret);

out:
	/* Unref the buffers */
	if (in_frame) {
		err = mbuf_raw_video_frame_unref(in_frame);
		if (err < 0)
			VENC_LOG_ERRNO("mbuf_raw_video_frame_unref", -ret);
	}
	if (out_frame) {
		err = mbuf_coded_video_frame_unref(out_frame);
		if (err < 0)
			VENC_LOG_ERRNO("mbuf_coded_video_frame_unref", -ret);
	}
}


static int set_profile_level(struct venc_videotoolbox *self,
			     unsigned int p,
			     unsigned int l)
{
	int ret;
	OSStatus osstatus;
	CFStringRef profile_level = NULL;

	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264:
		ret = h264_profile_level_to_videotoolbox(p, l, &profile_level);
		break;
	case VDEF_ENCODING_H265:
		/* Note: videotoolbox does not support HEVC level */
		ret = h265_profile_level_to_videotoolbox(p, &profile_level);
		break;
	default:
		ret = -ENOSYS;
		break;
	}
	if (ret < 0)
		goto out;

	osstatus = VTSessionSetProperty(self->compress_ref,
					kVTCompressionPropertyKey_ProfileLevel,
					profile_level);
	if (osstatus != noErr) {
		ret = -ENOSYS;
		VENC_LOGE("unable to set profile/level");
		goto out;
	}
	VENC_LOGD("set profile/level to '%s'",
		  profile_level ? CFStringGetCStringPtr(profile_level,
							kCFStringEncodingASCII)
				: "AUTO");

	ret = 0;

out:
	return ret;
}


static int set_entropy_coding(struct venc_videotoolbox *self)
{
	int ret;
	OSStatus osstatus;
	CFStringRef ec_key = NULL;

	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264:
		ret = entropy_coding_to_videotoolbox(
			self->base->config.h264.entropy_coding, &ec_key);
		break;
	case VDEF_ENCODING_H265:
		/* N/A */
		return 0;
	default:
		ret = -ENOSYS;
		break;
	}
	if (ret < 0)
		goto out;

	osstatus =
		VTSessionSetProperty(self->compress_ref,
				     kVTCompressionPropertyKey_H264EntropyMode,
				     ec_key);
	if (osstatus != noErr) {
		ret = -ENOSYS;
		VENC_LOGE("unable to set profile/level");
		goto out;
	}
	VENC_LOGD("set entropy coding to '%s'",
		  ec_key ? CFStringGetCStringPtr(ec_key, kCFStringEncodingASCII)
			 : "AUTO");

	ret = 0;

out:
	return ret;
}


static int set_color_transfer_matrix(struct venc_videotoolbox *self,
				     enum vdef_color_primaries cp,
				     enum vdef_transfer_function tf,
				     enum vdef_matrix_coefs mc)
{
	int ret;
	OSStatus osstatus;
	CFStringRef cp_key = NULL;
	CFStringRef tf_key = NULL;
	CFStringRef mc_key = NULL;

	ret = color_primaries_to_videotoolbox(cp, &cp_key);
	if (ret < 0)
		goto out;

	osstatus =
		VTSessionSetProperty(self->compress_ref,
				     kVTCompressionPropertyKey_ColorPrimaries,
				     cp_key);
	if (osstatus != noErr) {
		ret = -ENOSYS;
		VENC_LOGE("unable to set color primaries");
		goto out;
	}
	VENC_LOGD("set color primaries to '%s'",
		  cp_key ? CFStringGetCStringPtr(cp_key, kCFStringEncodingASCII)
			 : "AUTO");

	ret = transfer_function_to_videotoolbox(tf, &tf_key);
	if (ret < 0)
		goto out;

	osstatus =
		VTSessionSetProperty(self->compress_ref,
				     kVTCompressionPropertyKey_TransferFunction,
				     tf_key);
	if (osstatus != noErr) {
		ret = -ENOSYS;
		VENC_LOGE("unable to set transfer function");
		goto out;
	}
	VENC_LOGD("set transfer function to '%s'",
		  tf_key ? CFStringGetCStringPtr(tf_key, kCFStringEncodingASCII)
			 : "AUTO");

	ret = matrix_coefs_to_videotoolbox(mc, &mc_key);
	if (ret < 0)
		goto out;

	osstatus = VTSessionSetProperty(self->compress_ref,
					kVTCompressionPropertyKey_YCbCrMatrix,
					mc_key);
	if (osstatus != noErr) {
		ret = -ENOSYS;
		VENC_LOGE("unable to set matrix coefs");
		goto out;
	}
	VENC_LOGD("set matrix coefs to '%s'",
		  mc_key ? CFStringGetCStringPtr(mc_key, kCFStringEncodingASCII)
			 : "AUTO");

	ret = 0;

out:
	return ret;
}


static int set_gop(struct venc_videotoolbox *self, float gop_length_sec)
{
	int ret;
	OSStatus osstatus;
	int gop_size;
	CFNumberRef gop_size_num = NULL;

	gop_size = (int)((gop_length_sec *
			  (float)self->base->config.input.info.framerate.num) /
			 self->base->config.input.info.framerate.den);

	gop_size_num = CFNumberCreate(
		kCFAllocatorDefault, kCFNumberSInt32Type, &gop_size);
	if (gop_size_num == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	osstatus = VTSessionSetProperty(
		self->compress_ref,
		kVTCompressionPropertyKey_MaxKeyFrameInterval,
		gop_size_num);
	if (osstatus != noErr) {
		ret = -ENOSYS;
		VENC_LOGE("unable to set gop_length_sec to %.2f",
			  gop_length_sec);
		goto out;
	}

	ret = 0;

out:
	return ret;
}


static int set_bitrates(struct venc_videotoolbox *self,
			unsigned int target_bitrate,
			unsigned int max_bitrate)
{
	int ret;
	OSStatus osstatus;
	int64_t bytes_per_second = (1.3 * max_bitrate / 8);
	int64_t one_second = 1;
	CFNumberRef target_bitrate_num = NULL;
	CFNumberRef bytes_per_second_num = NULL;
	CFNumberRef one_second_num = NULL;
	CFArrayRef data_rate_limits = NULL;
	void *nums[2];
	enum venc_rate_control rate_control;

	target_bitrate_num = CFNumberCreate(
		kCFAllocatorDefault, kCFNumberSInt32Type, &target_bitrate);
	if (target_bitrate_num == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	bytes_per_second_num = CFNumberCreate(
		kCFAllocatorDefault, kCFNumberSInt64Type, &bytes_per_second);
	if (bytes_per_second_num == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	one_second_num = CFNumberCreate(
		kCFAllocatorDefault, kCFNumberSInt64Type, &one_second);
	if (one_second_num == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	nums[0] = (void *)bytes_per_second_num;
	nums[1] = (void *)one_second_num;
	data_rate_limits = CFArrayCreate(kCFAllocatorDefault,
					 (const void **)nums,
					 2,
					 &kCFTypeArrayCallBacks);
	if (data_rate_limits == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264:
		rate_control = self->base->config.h264.rate_control;
		break;
	case VDEF_ENCODING_H265:
		rate_control = self->base->config.h265.rate_control;
		break;
	default:
		ret = -ENOSYS;
		goto out;
	}

	switch (rate_control) {
	case VENC_RATE_CONTROL_CBR:
	/* Note: kVTCompressionPropertyKey_ConstantBitRate is not
	 * recommended as encoder will pad stream with empty data */
	case VENC_RATE_CONTROL_VBR:
		VENC_LOGD("%s: %u (max: %u)",
			  __func__,
			  target_bitrate,
			  max_bitrate);
		osstatus = VTSessionSetProperty(
			self->compress_ref,
			kVTCompressionPropertyKey_AverageBitRate,
			target_bitrate_num);
		if (osstatus != noErr) {
			ret = -ENOSYS;
			VENC_LOGE("CBR/VBR is not supported");
			goto out;
		}
		osstatus = VTSessionSetProperty(
			self->compress_ref,
			kVTCompressionPropertyKey_DataRateLimits,
			data_rate_limits);
		if (osstatus != noErr &&
		    self->base->config.encoding != VDEF_ENCODING_H265) {
			ret = -ENOSYS;
			VENC_LOGE("unable to set max bitrate");
			goto out;
		}
		break;
	case VENC_RATE_CONTROL_CQ:
		ret = -ENOSYS;
		VENC_LOGE("CQ is not supported");
		goto out;
	default:
		ret = -EPROTO;
		VENC_LOGE("invalid rate_control: %d",
			  self->base->config.h264.rate_control);
		goto out;
	}

	ret = 0;

out:
	if (target_bitrate_num != NULL)
		CFRelease(target_bitrate_num);
	if (bytes_per_second_num != NULL)
		CFRelease(bytes_per_second_num);
	if (one_second_num != NULL)
		CFRelease(one_second_num);
	if (data_rate_limits != NULL)
		CFRelease(data_rate_limits);
	return ret;
}


static void compression_method_destroy(struct venc_videotoolbox *self)
{
	if (self == NULL || self->compress_ref == NULL)
		return;

	VTCompressionSessionInvalidate(self->compress_ref);
	CFRelease(self->compress_ref);
	self->compress_ref = NULL;
}


static int compression_session_create(struct venc_videotoolbox *self,
				      int creation)
{
	int ret = 0;
	OSStatus osstatus;
	CFMutableDictionaryRef buffer_attr = NULL;
	CMVideoCodecType codec_type;
	CFDictionaryRef properties = NULL;
	unsigned int max_bitrate;
	unsigned int target_bitrate;
	unsigned int profile;
	unsigned int level;
	float gop_length_sec;
#if !TARGET_OS_IPHONE
	const void *keys[] = {
		kVTCompressionPropertyKey_RealTime,
		/* codecheck_ignore[LONG_LINE] */
		kVTVideoEncoderSpecification_EnableHardwareAcceleratedVideoEncoder,
		kVTCompressionPropertyKey_AllowFrameReordering,
	};
	const void *values[] = {
		kCFBooleanTrue,
		kCFBooleanTrue,
		kCFBooleanFalse,
	};
#else
	const void *keys[] = {
		kVTCompressionPropertyKey_RealTime,
		kVTCompressionPropertyKey_AllowFrameReordering,
	};
	const void *values[] = {
		kCFBooleanTrue,
		kCFBooleanFalse,
	};
#endif /* !TARGET_OS_IPHONE */
	int properties_size = SIZEOF_ARRAY(keys);

	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264:
		codec_type = kCMVideoCodecType_H264;
		profile = self->base->config.h264.profile;
		level = self->base->config.h264.level;
		max_bitrate = self->base->config.h264.max_bitrate;
		target_bitrate = self->base->config.h264.target_bitrate;
		gop_length_sec = self->base->config.h264.gop_length_sec;
		if (creation) {
			self->dynconf = (struct venc_dyn_config){
				.qp = self->base->config.h264.qp,
				.target_bitrate = target_bitrate,
				.decimation =
					self->base->config.h264.decimation,
			};
		}
		break;
	case VDEF_ENCODING_H265:
		codec_type = kCMVideoCodecType_HEVC;
		profile = self->base->config.h265.profile;
		level = self->base->config.h265.level;
		max_bitrate = self->base->config.h265.max_bitrate;
		target_bitrate = self->base->config.h265.target_bitrate;
		gop_length_sec = self->base->config.h265.gop_length_sec;
		if (creation) {
			self->dynconf = (struct venc_dyn_config){
				.qp = self->base->config.h265.qp,
				.target_bitrate = target_bitrate,
				.decimation =
					self->base->config.h265.decimation,
			};
		}
		break;
	default:
		ret = -EPROTO;
		VENC_LOG_ERRNO("invalid codec type", -ret);
		goto out;
	}
	if (!creation && self->dynconf.target_bitrate != 0) {
		max_bitrate = self->dynconf.target_bitrate;
		target_bitrate = self->dynconf.target_bitrate;
	}

	buffer_attr = buffer_attr_create(self);
	if (buffer_attr == NULL) {
		ret = -ENOMEM;
		VENC_LOG_ERRNO("buffer_attr_create", -ret);
		goto out;
	}

	osstatus = VTCompressionSessionCreate(
		kCFAllocatorDefault,
		self->base->config.input.info.resolution.width,
		self->base->config.input.info.resolution.height,
		codec_type,
		NULL,
		buffer_attr,
		kCFAllocatorDefault,
		frame_output_cb,
		self,
		&self->compress_ref);
	if (osstatus == kVTCouldNotFindVideoEncoderErr) {
		ret = -EINVAL;
		VENC_LOG_ERRNO(
			"VTCompressionSessionCreate error: "
			"Encoding not supported status=%d",
			-ret,
			(int)osstatus);
		goto out;
	} else if (osstatus != noErr) {
		ret = -EPROTO;
		VENC_LOG_ERRNO("VTCompressionSessionCreate status=%d",
			       -ret,
			       (int)osstatus);
		goto out;
	}

	if (creation)
		VENC_LOGI("videotoolbox implementation");

	properties = CFDictionaryCreate(
		kCFAllocatorDefault, keys, values, properties_size, NULL, NULL);
	if (properties == NULL) {
		ret = -ENOSYS;
		VENC_LOG_ERRNO("CFDictionaryCreate", -ret);
		goto out;
	}

	osstatus = VTSessionSetProperties(self->compress_ref, properties);
	if (osstatus != noErr) {
		ret = -EPROTO;
		VENC_LOG_ERRNO("VTSessionSetProperties status=%d",
			       EPROTO,
			       (int)osstatus);
		goto out;
	}

	/* Set profile and level */
	ret = set_profile_level(self, profile, level);
	if (ret < 0) {
		VENC_LOG_ERRNO("set_profile_level", -ret);
		goto out;
	}

	/* Set entropy coding */
	ret = set_entropy_coding(self);
	if (ret < 0) {
		VENC_LOG_ERRNO("set_entropy_coding", -ret);
		goto out;
	}

	/* Set color, transfer, matrix */
	ret = set_color_transfer_matrix(
		self,
		self->base->config.input.info.color_primaries,
		self->base->config.input.info.transfer_function,
		self->base->config.input.info.matrix_coefs);
	if (ret < 0) {
		VENC_LOG_ERRNO("set_color_transfer_matrix", -ret);
		goto out;
	}

	/* Set RC mode and bitrates */
	ret = set_bitrates(self, target_bitrate, max_bitrate);
	if (ret < 0) {
		VENC_LOG_ERRNO("set_bitrates", -ret);
		goto out;
	}

	/* Set GOP */
	ret = set_gop(self, gop_length_sec);
	if (ret < 0) {
		VENC_LOG_ERRNO("set_gop", -ret);
		goto out;
	}

	osstatus =
		VTCompressionSessionPrepareToEncodeFrames(self->compress_ref);
	if (osstatus != noErr) {
		ret = -EPROTO;
		VENC_LOG_ERRNO(
			"VTCompressionSessionPrepareToEncodeFrames status=%d",
			-ret,
			(int)osstatus);
		goto out;
	}

out:
	if (properties)
		CFRelease(properties);

	if (buffer_attr)
		CFRelease(buffer_attr);

	return ret;
}


static int compression_session_renew(struct venc_videotoolbox *self)
{
	compression_method_destroy(self);
	return compression_session_create(self, 0);
}


static int buffer_push_one(struct venc_videotoolbox *self,
			   struct mbuf_raw_video_frame *in_frame)
{
	int ret = 0, err;
	OSStatus osstatus;
	struct vdef_raw_frame info = {};
	const uint8_t *data_addr[VDEF_RAW_MAX_PLANE_COUNT] = {};
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;
	size_t plane_size[VDEF_RAW_MAX_PLANE_COUNT] = {};
	unsigned int plane_count;
	CMTime presentationTimeStamp;
	bool idr_requested = false;
	CFDictionaryRef encode_props = NULL;
	CVPixelBufferPoolRef pool = NULL;
	CVPixelBufferRef pix_buf = NULL;
	bool pix_buf_locked = false;
	int ref_count = 1;

	VENC_LOG_ERRNO_RETURN_ERR_IF(in_frame == NULL, EINVAL);

	/* Frame skipping in case of decimation */
	if (self->input_frame_cnt % self->dynconf.decimation != 0) {
		self->input_frame_cnt++;
		return 0;
	}
	self->input_frame_cnt++;

	ret = mbuf_raw_video_frame_get_frame_info(in_frame, &info);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		goto out;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	err = mbuf_raw_video_frame_add_ancillary_buffer(
		in_frame,
		VENC_ANCILLARY_KEY_DEQUEUE_TIME,
		&ts_us,
		sizeof(ts_us));
	if (err < 0)
		VENC_LOGW_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
				-err);

	plane_count = vdef_get_raw_frame_plane_count(&info.format);
	if (plane_count == 0) {
		ret = -EPROTO;
		VENC_LOG_ERRNO("vdef_get_raw_frame_plane_count", -ret);
		goto out;
	}

	for (unsigned int i = 0; i < plane_count; i++) {
		ret = mbuf_raw_video_frame_get_plane(
			in_frame,
			i,
			(const void **)&data_addr[i],
			&plane_size[i]);
		if (ret < 0) {
			VENC_LOG_ERRNO(
				"mbuf_raw_video_frame_get_plane(%u)", -ret, i);
			goto out;
		}
	}

	pool = VTCompressionSessionGetPixelBufferPool(self->compress_ref);
	if (pool == NULL) {
		osstatus = VTCompressionSessionPrepareToEncodeFrames(
			self->compress_ref);
		if (osstatus == kVTInvalidSessionErr) {
			ret = compression_session_renew(self);
			if (ret < 0) {
				ret = -EIO;
				VENC_LOG_ERRNO("compression_session_renew",
					       -ret);
				goto out;
			}
			pool = VTCompressionSessionGetPixelBufferPool(
				self->compress_ref);
			if (pool == NULL) {
				ret = -EPROTO;
				VENC_LOG_ERRNO(
					"VTCompressionSessionGetPixelBufferPool",
					-ret);
				goto out;
			} else {
				VENC_LOGI(
					"VT session restarted because of a "
					"kVTInvalidSessionErr error");
			}
		} else {
			ret = -EPROTO;
			VENC_LOG_ERRNO("VTCompressionSessionGetPixelBufferPool",
				       -ret);
			goto out;
		}
	}

	ret = CVPixelBufferPoolCreatePixelBuffer(NULL, pool, &pix_buf);
	if (ret != kCVReturnSuccess) {
		VENC_LOG_ERRNO("CVPixelBufferPoolCreatePixelBuffer", -ret);
		goto out;
	}

	/* Fill buffer */

	ret = CVPixelBufferLockBaseAddress(pix_buf, 0);
	if (ret != kCVReturnSuccess) {
		VENC_LOG_ERRNO("CVPixelBufferLockBaseAddress", -ret);
		goto out;
	}
	pix_buf_locked = true;

	if (CVPixelBufferIsPlanar(pix_buf)) {
		unsigned int buf_plane_count =
			CVPixelBufferGetPlaneCount(pix_buf);
		if (buf_plane_count != plane_count) {
			VENC_LOGE("plane count mismatch (expecting %d, got %d)",
				  plane_count,
				  buf_plane_count);
			ret = -EPROTO;
			goto out;
		}
		for (unsigned int i = 0; i < plane_count; i++) {
			uint8_t *plane_dst_addr =
				(uint8_t *)CVPixelBufferGetBaseAddressOfPlane(
					pix_buf, i);
			if (plane_dst_addr == NULL) {
				ret = -EPROTO;
				VENC_LOG_ERRNO(
					"CVPixelBufferGetBaseAddress"
					"OfPlane(%u)",
					-ret,
					i);
				goto out;
			}
			size_t dst_stride =
				CVPixelBufferGetBytesPerRowOfPlane(pix_buf, i);
			if (dst_stride == 0) {
				ret = -EPROTO;
				VENC_LOG_ERRNO(
					"CVPixelBufferGetBytesPerRow"
					"OfPlane(%u)",
					-ret,
					i);
				goto out;
			}
			/* Copy plane with stride */
			if (info.plane_stride[i] == 0) {
				ret = -EPROTO;
				VENC_LOG_ERRNO("invalid stride", -ret);
				goto out;
			}
			unsigned int nlines =
				plane_size[i] / info.plane_stride[i];
			for (unsigned int j = 0; j < nlines; j++) {
				memcpy(plane_dst_addr + (j * dst_stride),
				       data_addr[i] +
					       (j * info.plane_stride[i]),
				       MIN(dst_stride, info.plane_stride[i]));
			}
		}
	} else {
		uint8_t *dst_addr = CVPixelBufferGetBaseAddress(pix_buf);
		if (dst_addr == NULL) {
			ret = -EPROTO;
			VENC_LOG_ERRNO("CVPixelBufferGetBaseAddress", -ret);
			goto out;
		}
		size_t dst_stride = CVPixelBufferGetBytesPerRow(pix_buf);
		if (dst_stride == 0) {
			ret = -EPROTO;
			VENC_LOG_ERRNO("CVPixelBufferGetBytesPerRow", -ret);
			goto out;
		}
		for (unsigned int i = 0; i < plane_count; i++) {
			/* Copy plane with stride */
			if (info.plane_stride[i] == 0) {
				ret = -EPROTO;
				VENC_LOG_ERRNO("invalid stride", -ret);
				goto out;
			}
			unsigned int nlines =
				plane_size[i] / info.plane_stride[i];
			for (unsigned int j = 0; j < nlines; j++) {
				memcpy(dst_addr,
				       data_addr[i] +
					       (j * info.plane_stride[i]),
				       MIN(dst_stride, info.plane_stride[i]));
				dst_addr += dst_stride;
			}
		}
	}

	ret = CVPixelBufferUnlockBaseAddress(pix_buf, 0);
	if (ret != kCVReturnSuccess) {
		VENC_LOG_ERRNO("CVPixelBufferUnlockBaseAddress", -ret);
		goto out;
	}
	pix_buf_locked = false;

	if (atomic_load(&self->idr_requested)) {
		const void *keys[] = {kVTEncodeFrameOptionKey_ForceKeyFrame};
		const void *vals[] = {kCFBooleanTrue};
		encode_props = CFDictionaryCreate(kCFAllocatorDefault,
						  keys,
						  vals,
						  SIZEOF_ARRAY(keys),
						  NULL,
						  NULL);
		if (encode_props == NULL) {
			ret = -ENOMEM;
			VENC_LOG_ERRNO("CFDictionaryCreate", -ret);
			goto out;
		}
		idr_requested = true;
	}

	/* Push the frame */
	presentationTimeStamp =
		CMTimeMake(info.info.timestamp, info.info.timescale);

	/* Note: ref-count is incremented here as frame is passed to the
	 * VTCompressionSession as userdata and unref'ed when output */
	ret = mbuf_raw_video_frame_ref(in_frame);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_ref", -ret);
		goto out;
	}
	ref_count++;

	osstatus = VTCompressionSessionEncodeFrame(self->compress_ref,
						   pix_buf,
						   presentationTimeStamp,
						   kCMTimeIndefinite,
						   encode_props,
						   in_frame,
						   NULL);
	if (osstatus != noErr) {
		ret = -EPROTO;
		VENC_LOG_ERRNO("VTCompressionSessionEncodeFrame status=%d",
			       -ret,
			       (int)osstatus);
		goto out;
	}

	self->base->counters.pushed++;
	if (idr_requested)
		atomic_store(&self->idr_requested, false);

	ret = 0;

out:
	if (pix_buf) {
		if (pix_buf_locked) {
			err = CVPixelBufferUnlockBaseAddress(pix_buf, 0);
			if (err != kCVReturnSuccess) {
				VENC_LOG_ERRNO("CVPixelBufferUnlockBaseAddress",
					       -err);
			}
		}
		CFRelease(pix_buf);
	}
	if (encode_props)
		CFRelease(encode_props);
	if (in_frame) {
		for (unsigned int i = 0; i < SIZEOF_ARRAY(data_addr); i++) {
			if (data_addr[i] == NULL)
				continue;
			err = mbuf_raw_video_frame_release_plane(
				in_frame, i, data_addr[i]);
			if (err < 0) {
				VENC_LOG_ERRNO(
					"mbuf_raw_video_frame"
					"_release_plane(%u)",
					-err,
					i);
			}
		}
		if (ret < 0) {
			while (ref_count > 0) {
				err = mbuf_raw_video_frame_unref(in_frame);
				if (err < 0)
					VENC_LOG_ERRNO(
						"mbuf_raw_video_frame_unref",
						-err);
				ref_count--;
			}
		}
	}
	return ret;
}


static void check_input_queue(struct venc_videotoolbox *self)
{
	int ret, err;
	struct mbuf_raw_video_frame *in_frame;

	ret = mbuf_raw_video_frame_queue_peek(self->in_queue, &in_frame);
	while (ret >= 0) {
		/* Push the input frame; frame is unref on error only */
		/* Encode the frame */
		ret = buffer_push_one(self, in_frame);
		if (ret < 0) {
			if (ret != -EAGAIN)
				VENC_LOG_ERRNO("buffer_push_one", -ret);
			ret = -ENOSPC;
			break;
		}
		if (in_frame) {
			err = mbuf_raw_video_frame_unref(in_frame);
			if (err < 0) {
				VENC_LOG_ERRNO("mbuf_raw_video_frame_unref",
					       -ret);
			}
			/* Pop the frame for real */
			ret = mbuf_raw_video_frame_queue_pop(self->in_queue,
							     &in_frame);
			if (ret < 0) {
				VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_pop",
					       -ret);
				break;
			}
			err = mbuf_raw_video_frame_unref(in_frame);
			if (err < 0) {
				VENC_LOG_ERRNO("mbuf_raw_video_frame_unref",
					       -ret);
			}
		}
		/* Peek the next frame */
		ret = mbuf_raw_video_frame_queue_peek(self->in_queue,
						      &in_frame);
		if (ret < 0 && ret != -EAGAIN && ret != -ENOSPC)
			VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_peek", -ret);
		if (self->flush && ret == -EAGAIN) {
			in_frame = NULL;
			ret = do_flush(self);
			if (ret < 0)
				VENC_LOG_ERRNO("do_flush", -ret);
			break;
		}
	}

	if (ret == -EAGAIN && atomic_load(&self->flush) &&
	    !atomic_load(&self->flush_discard)) {
		ret = do_flush(self);
		if (ret < 0)
			VENC_LOG_ERRNO("do_flush", -ret);
	}
}


static void input_event_cb(struct pomp_evt *evt, void *userdata)
{
	struct venc_videotoolbox *self = userdata;
	check_input_queue(self);
}


static void *encoder_thread(void *ptr)
{
	int ret, timeout;
	struct venc_videotoolbox *self = ptr;
	struct pomp_loop *loop = NULL;
	struct pomp_evt *in_queue_evt = NULL;
	bool flush = false;

#if !TARGET_OS_IPHONE
	ret = pthread_setname_np("venc_vtoolbox");
	if (ret != 0)
		VENC_LOG_ERRNO("pthread_setname_np", ret);
#endif

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

	while ((!atomic_load(&self->should_stop)) || (flush)) {
		flush = atomic_load(&self->flush);
		/* Flush discarding all frames */
		if ((flush) && (atomic_load(&self->flush_discard))) {
			ret = do_flush(self);
			if (ret < 0)
				VENC_LOG_ERRNO("do_flush", -ret);
			continue;
		}

		/* Get an input buffer (with timeout) */
		timeout = ((flush) && (!atomic_load(&self->flush_discard))) ? 0
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
	struct venc_videotoolbox_message message = {
		.type = VENC_VIDEOTOOLBOX_MESSAGE_TYPE_STOP,
	};
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
	*formats = supported_formats;
	return NB_SUPPORTED_FORMATS;
}


static int flush(struct venc_encoder *base, int discard_)
{
	struct venc_videotoolbox *self = NULL;
	bool discard = discard_;

	VENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);
	self = base->derived;

	atomic_store(&self->flush, true);
	atomic_store(&self->flush_discard, discard);

	return 0;
}


static int stop(struct venc_encoder *base)
{
	struct venc_videotoolbox *self = NULL;

	VENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);
	self = base->derived;

	/* Stop the encoding thread */
	atomic_store(&self->should_stop, true);

	return 0;
}


static int destroy(struct venc_encoder *base)
{
	int err;
	struct venc_videotoolbox *self = NULL;

	VENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;
	if (self == NULL)
		return 0;

	err = stop(base);
	if (err != 0)
		VENC_LOG_ERRNO("stop", -err);

	compression_method_destroy(self);

	if (self->ps_lock_created) {
		err = pthread_mutex_destroy(&self->ps_lock);
		if (err != 0)
			VENC_LOG_ERRNO("pthread_mutex_destroy", err);
	}

	if (self->thread_launched) {
		err = pthread_join(self->thread, NULL);
		if (err != 0)
			VENC_LOG_ERRNO("pthread_join", err);
	}

	/* Free the resources */
	if (self->out_queue_evt != NULL &&
	    pomp_evt_is_attached(self->out_queue_evt, base->loop)) {
		err = pomp_evt_detach_from_loop(self->out_queue_evt,
						base->loop);
		if (err < 0)
			VENC_LOG_ERRNO("pomp_evt_detach_from_loop", -err);
	}
	if (self->out_queue != NULL) {
		err = mbuf_coded_video_frame_queue_destroy(self->out_queue);
		if (err < 0)
			VENC_LOG_ERRNO(
				"mbuf_coded_video_frame_queue_destroy:output",
				-err);
	}
	if (self->in_queue != NULL) {
		err = mbuf_raw_video_frame_queue_destroy(self->in_queue);
		if (err < 0)
			VENC_LOG_ERRNO(
				"mbuf_raw_video_frame_queue_destroy:input",
				-err);
	}
	if (self->mbox != NULL) {
		err = pomp_loop_remove(base->loop,
				       mbox_get_read_fd(self->mbox));
		if (err < 0)
			VENC_LOG_ERRNO("pomp_loop_remove", -err);
		mbox_destroy(self->mbox);
	}

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
	struct venc_videotoolbox *self = userdata;

	VENC_LOG_ERRNO_RETURN_ERR_IF(self == NULL, false);

	if (atomic_load(&self->flush) || atomic_load(&self->should_stop))
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


static int create(struct venc_encoder *base)
{
	int ret = 0;
	struct venc_videotoolbox *self = NULL;
	struct mbuf_raw_video_frame_queue_args queue_args = {
		.filter = input_filter,
	};

	(void)pthread_once(&compat_keys_is_init, initialize_compat_keys);
	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	VENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	switch (base->config.encoding) {
	case VDEF_ENCODING_H264:
	case VDEF_ENCODING_H265:
		break;
	default:
		ret = -EINVAL;
		VENC_LOG_ERRNO("unsupported encoding", -ret);
		return ret;
	}

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	base->derived = self;
	self->base = base;
	atomic_init(&self->idr_requested, false);
	queue_args.filter_userdata = self;

	self->mbox = mbox_new(sizeof(struct venc_videotoolbox_message));
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

	ret = mbuf_coded_video_frame_queue_new(&self->out_queue);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_queue_new:output", -ret);
		goto error;
	}
	ret = mbuf_coded_video_frame_queue_get_event(self->out_queue,
						     &self->out_queue_evt);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_coded_video_frame_queue_get_event", -ret);
		goto error;
	}
	ret = pomp_evt_attach_to_loop(
		self->out_queue_evt, base->loop, &out_queue_evt_cb, self);
	if (ret < 0) {
		VENC_LOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto error;
	}

	ret = mbuf_raw_video_frame_queue_new_with_args(&queue_args,
						       &self->in_queue);
	if (ret < 0) {
		VENC_LOG_ERRNO("mbuf_raw_video_frame_queue_new_with_args:input",
			       -ret);
		goto error;
	}

	ret = pthread_mutex_init(&self->ps_lock, NULL);
	if (ret) {
		VENC_LOG_ERRNO("pthread_mutex_init", ret);
		goto error;
	};
	self->ps_lock_created = true;

	ret = pthread_create(&self->thread, NULL, encoder_thread, self);
	if (ret != 0) {
		ret = -ret;
		VENC_LOG_ERRNO("pthread_create", -ret);
		goto error;
	}
	self->thread_launched = true;

	ret = vdef_raw_format_to_videotoolbox(
		&self->base->config.input.format,
		self->base->config.input.info.full_range,
		&self->vt_pixel_format);
	if (ret < 0) {
		VENC_LOG_ERRNO("vdef_raw_format_to_videotoolbox", -ret);
		goto error;
	}

	ret = compression_session_create(self, 1);
	if (ret < 0) {
		VENC_LOG_ERRNO("compression_session_create", -ret);
		goto error;
	}

	return 0;

error:
	destroy(base);
	return ret;
}


static struct mbuf_pool *get_input_buffer_pool(struct venc_encoder *base)
{
	struct venc_videotoolbox *self = NULL;

	VENC_LOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);
	self = base->derived;

	/* No input buffer pool allocated: use the application's */
	return NULL;
}


static struct mbuf_raw_video_frame_queue *
get_input_buffer_queue(struct venc_encoder *base)
{
	struct venc_videotoolbox *self = NULL;

	VENC_LOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);
	self = base->derived;

	return self->in_queue;
}


static int get_dyn_config(struct venc_encoder *base,
			  struct venc_dyn_config *config)
{
	struct venc_videotoolbox *self = NULL;

	VENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);
	self = base->derived;

	*config = self->dynconf;

	return 0;
}


static int set_dyn_config(struct venc_encoder *base,
			  const struct venc_dyn_config *config)
{
	int ret;
	struct venc_videotoolbox *self = NULL;

	VENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);
	self = base->derived;

	/* QP */
	if ((config->qp != 0) && (config->qp != self->dynconf.qp)) {
		VENC_LOGE("QP is not supported");
		return -ENOSYS;
	}

	/* Target bitrate */
	if ((config->target_bitrate != 0) &&
	    (config->target_bitrate != self->dynconf.target_bitrate)) {
		ret = set_bitrates(
			self, config->target_bitrate, config->target_bitrate);
		if (ret < 0) {
			VENC_LOG_ERRNO("set_bitrates", -ret);
			return ret;
		}
		self->dynconf.target_bitrate = config->target_bitrate;
	}

	/* Decimation */
	if ((config->decimation != 0) &&
	    (config->decimation != self->dynconf.decimation))
		self->dynconf.decimation = config->decimation;

	return 0;
}


static int request_idr(struct venc_encoder *base)
{
	struct venc_videotoolbox *self = base->derived;

	atomic_store(&self->idr_requested, true);

	return 0;
}


const struct venc_ops venc_videotoolbox_ops = {
	.get_supported_encodings = get_supported_encodings,
	.get_supported_input_formats = get_supported_input_formats,
	.flush = flush,
	.stop = stop,
	.destroy = destroy,
	.create = create,
	.get_input_buffer_pool = get_input_buffer_pool,
	.get_input_buffer_queue = get_input_buffer_queue,
	.get_dyn_config = get_dyn_config,
	.set_dyn_config = set_dyn_config,
	.request_idr = request_idr,
};
