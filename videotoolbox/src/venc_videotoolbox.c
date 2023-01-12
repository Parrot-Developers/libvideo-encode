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
	supported_formats[0] = vdef_i420;
	supported_formats[1] = vdef_nv12;

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


static void flush_complete(struct venc_videotoolbox *self)
{
	/* Call the flush callback if defined */
	if (self->base->cbs.flush)
		self->base->cbs.flush(self->base, self->base->userdata);
}


static void stop_complete(struct venc_videotoolbox *self)
{
	atomic_store(&self->is_stopped, true);
	/* Call the stop callback if defined */
	if (self->base->cbs.stop)
		self->base->cbs.stop(self->base, self->base->userdata);
}


static void encoder_error(struct venc_videotoolbox *self, int error)
{
	self->base->cbs.frame_output(
		self->base, error, NULL, self->base->userdata);
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
	int ret;
	struct venc_videotoolbox_message message;

	while (true) {
		/* Read from the mailbox */
		ret = mbox_peek(self->mbox, &message);
		if (ret < 0) {
			if (ret != -EAGAIN)
				ULOG_ERRNO("mbox_peek", -ret);
			break;
		}

		switch (message.type) {
		case VENC_VIDEOTOOLBOX_MESSAGE_TYPE_FLUSH:
			flush_complete(self);
			break;
		case VENC_VIDEOTOOLBOX_MESSAGE_TYPE_STOP:
			stop_complete(self);
			break;
		case VENC_VIDEOTOOLBOX_MESSAGE_TYPE_ERROR:
			encoder_error(self, message.error);
			break;
		default:
			ULOGE("unknown message type: %d", message.type);
			break;
		}
	}
}


static CFMutableDictionaryRef buffer_attr_create(int full_range)
{
	int err = 0;
	CFMutableDictionaryRef buffer_attr = NULL, io_surface_properties = NULL;
	CFNumberRef pix_fmt = NULL;
	int fmt = full_range ? kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
			     : kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange;

	buffer_attr =
		CFDictionaryCreateMutable(kCFAllocatorDefault,
					  4,
					  &kCFTypeDictionaryKeyCallBacks,
					  &kCFTypeDictionaryValueCallBacks);
	if (buffer_attr == NULL) {
		err = -ENOMEM;
		ULOG_ERRNO("CFDictionaryCreateMutable", -err);
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
		ULOG_ERRNO("CFDictionaryCreateMutable", -err);
		goto out;
	}

	pix_fmt =
		CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &fmt);
	if (pix_fmt == NULL) {
		err = -ENOMEM;
		ULOG_ERRNO("CFNumberCreate", -err);
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
	struct venc_videotoolbox *self = userdata;
	struct mbuf_coded_video_frame *out_frame = NULL;
	int ret;

	if (atomic_load(&self->ps_stored) == false)
		return;

	while (true) {
		ret = mbuf_coded_video_frame_queue_pop(self->out_queue,
						       &out_frame);
		if (ret == -EAGAIN) {
			return;
		} else if (ret < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_queue_pop:output",
				   -ret);
			return;
		}
		self->base->cbs.frame_output(
			self->base, 0, out_frame, self->base->userdata);
		mbuf_coded_video_frame_unref(out_frame);
	}
}


static int do_flush(struct venc_videotoolbox *self)
{
	int ret;

	if (atomic_load(&self->flush_discard)) {
		/* Flush the queues */
		ret = mbuf_raw_video_frame_queue_flush(self->in_queue);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_flush:input",
				   -ret);
			return ret;
		}
		ret = mbuf_coded_video_frame_queue_flush(self->out_queue);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_queue_flush:output",
				   -ret);
			return ret;
		}
	}

	atomic_store(&self->flushing, true);
	if (self->compress_ref) {
		ret = VTCompressionSessionCompleteFrames(self->compress_ref,
							 kCMTimeIndefinite);
		if (ret)
			ULOG_ERRNO("VTCompressionSessionCompleteFrames", -ret);
	}
	atomic_store(&self->flushing, false);

	/* Call the flush callback on the loop */
	struct venc_videotoolbox_message message = {
		.type = VENC_VIDEOTOOLBOX_MESSAGE_TYPE_FLUSH,
	};
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		ULOG_ERRNO("mbox_push", -ret);

	atomic_store(&self->flush, false);

	return 0;
}


static void cmbr_mbuf_release(void *data, size_t len, void *userdata)
{
	CMBlockBufferRef ref = userdata;
	CFRelease(ref);
}


static int set_frame_metadata(struct venc_videotoolbox *self,
			      struct mbuf_raw_video_frame *in_frame,
			      struct mbuf_coded_video_frame **out_frame,
			      CMBlockBufferRef ref)
{
	int ret, err;
	struct vdef_raw_frame in_info;
	struct vdef_coded_frame out_info;
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
		.pre_release = self->base->cbs.pre_release,
		.pre_release_userdata = self->base->userdata,
	};

	/* Mem creation */
	ref = (CMBlockBufferRef)CFRetain(ref);

	status = CMBlockBufferGetDataPointer(ref, 0, NULL, NULL, (char **)&ptr);
	if (status != noErr) {
		ULOG_ERRNO("CMBlockBufferGetDataPointer status=%d",
			   EPROTO,
			   (int)status);
	}
	start = data = ptr;

	len = CMBlockBufferGetDataLength(ref);

	ret = mbuf_mem_generic_wrap(ptr, len, cmbr_mbuf_release, ref, &mem);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_mem_generic_wrap", -ret);
		goto out;
	}

	/* Frame creation */
	ret = mbuf_raw_video_frame_get_frame_info(in_frame, &in_info);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
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
		ULOG_ERRNO("mbuf_coded_video_frame_new", -ret);
		goto out;
	}
	ret = mbuf_coded_video_frame_set_callbacks(*out_frame, &frame_cbs);
	if (ret < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_set_callbacks", -ret);

	/* Frame metadata */
	ret = mbuf_raw_video_frame_foreach_ancillary_data(
		in_frame,
		mbuf_coded_video_frame_ancillary_data_copier,
		*out_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_foreach_ancillary_data", -ret);
		goto out;
	}
	ret = mbuf_raw_video_frame_get_metadata(in_frame, &metadata);
	if (ret == 0) {
		ret = mbuf_coded_video_frame_set_metadata(*out_frame, metadata);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_set_metadata", -ret);
			goto out;
		}
	} else if ((ret < 0) && (ret != -ENOENT)) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_metadata", -ret);
		goto out;
	}

	if (self->base->config.encoding == VDEF_ENCODING_H264) {
		/* Add generated NAL units */
		/* TODO: Set SPS and PPS in base before first frame */
		ret = venc_h264_generate_nalus(
			self->base, *out_frame, &out_info);
		if (ret < 0) {
			ULOG_ERRNO("venc_h264_generate_nalus", -ret);
			goto out;
		}
	} else if (self->base->config.encoding == VDEF_ENCODING_H265) {
		/* Add generated NAL units */
		/* TODO: Set VPS, SPS and PPS in base before first frame */
		ret = venc_h265_generate_nalus(
			self->base, *out_frame, &out_info);
		if (ret < 0) {
			ULOG_ERRNO("venc_h265_generate_nalus", -ret);
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
			ULOG_ERRNO("mbuf_coded_video_frame_add_nalu", -ret);
			goto out;
		}

		data += 4 + nalu_len;
		offset += 4 + nalu_len;
	}
out:
	err = mbuf_mem_unref(mem);
	if (err != 0)
		ULOG_ERRNO("mbuf_mem_unref", -err);

	return ret;
}


static int set_h264_ps(struct venc_videotoolbox *self,
		       CMVideoFormatDescriptionRef format)
{
	int ret;
	size_t ps_count, ps_size;
	const uint8_t *ps;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	pthread_mutex_lock(&self->ps_lock);

	ret = CMVideoFormatDescriptionGetH264ParameterSetAtIndex(
		format, 0, NULL, NULL, &ps_count, NULL);
	if (ret) {
		ret = -EAGAIN;
		ULOGE("no PS available");
		goto end;
	}
	if (ps_count < 2) {
		ret = -ENOSYS;
		ULOG_ERRNO("not enough PS", -ret);
		goto end;
	} else if (ps_count > 2) {
		ret = -ENOSYS;
		ULOG_ERRNO("too many PS", -ret);
		goto end;
	}

	ret = CMVideoFormatDescriptionGetH264ParameterSetAtIndex(
		format, 0, &ps, &ps_size, NULL, NULL);
	if (ret) {
		ULOG_ERRNO("CMVideoFormatDescriptionGetH264ParameterSetAtIndex",
			   -ret);
		goto end;
	}
	copy_ps(&self->base->h264.sps, &self->base->h264.sps_size, ps, ps_size);

	ret = CMVideoFormatDescriptionGetH264ParameterSetAtIndex(
		format, 1, &ps, &ps_size, NULL, NULL);
	if (ret) {
		ULOG_ERRNO("CMVideoFormatDescriptionGetH264ParameterSetAtIndex",
			   -ret);
		goto end;
	}
	copy_ps(&self->base->h264.pps, &self->base->h264.pps_size, ps, ps_size);

	/* Initialize the H.264 writer */
	venc_h264_writer_new(self->base->h264.sps,
			     self->base->h264.sps_size,
			     self->base->h264.pps,
			     self->base->h264.pps_size,
			     &self->base->h264.ctx);
end:
	pthread_mutex_unlock(&self->ps_lock);

	return ret;
}


static int set_h265_ps(struct venc_videotoolbox *self,
		       CMVideoFormatDescriptionRef format)
{
	int ret = 0;
	size_t ps_count, ps_size;
	const uint8_t *ps;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	pthread_mutex_lock(&self->ps_lock);

	if (__builtin_available(iOS 11.0, macOS 10.13, tvos 11.0, *)) {

		ret = CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(
			format, 0, NULL, NULL, &ps_count, NULL);
		if (ret) {
			ret = -EAGAIN;
			ULOGE("no PS available");
			goto end;
		}
		if (ps_count < 3) {
			ret = -ENOSYS;
			ULOG_ERRNO("not enough PS", -ret);
			goto end;
		} else if (ps_count > 3) {
			ret = -ENOSYS;
			ULOG_ERRNO("too many PS", -ret);
			goto end;
		}

		ret = CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(
			format, 0, &ps, &ps_size, NULL, NULL);
		if (ret) {
			ULOG_ERRNO(
				"CMVideoFormatDescriptionGetHEVCParameterSetAtIndex",
				-ret);
			goto end;
		}
		copy_ps(&self->base->h265.vps,
			&self->base->h265.vps_size,
			ps,
			ps_size);

		ret = CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(
			format, 1, &ps, &ps_size, NULL, NULL);
		if (ret) {
			ULOG_ERRNO(
				"CMVideoFormatDescriptionGetHEVCParameterSetAtIndex",
				-ret);
			goto end;
		}
		copy_ps(&self->base->h265.sps,
			&self->base->h265.sps_size,
			ps,
			ps_size);

		ret = CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(
			format, 2, &ps, &ps_size, NULL, NULL);
		if (ret) {
			ULOG_ERRNO(
				"CMVideoFormatDescriptionGetHEVCParameterSetAtIndex",
				-ret);
			goto end;
		}
		copy_ps(&self->base->h265.pps,
			&self->base->h265.pps_size,
			ps,
			ps_size);

		/* Initialize the H.265 writer */
		venc_h265_writer_new(self->base->h265.vps,
				     self->base->h265.vps_size,
				     self->base->h265.sps,
				     self->base->h265.sps_size,
				     self->base->h265.pps,
				     self->base->h265.pps_size,
				     &self->base->h265.ctx);
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
		ULOG_ERRNO("CMSampleBufferGetFormatDescription", EPROTO);
		return -EPROTO;
	}

	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264:
		ret = set_h264_ps(self, format);
		if (ret) {
			ULOG_ERRNO("set_h264_ps", -ret);
			goto out;
		}
		atomic_store(&self->ps_stored, true);
		break;
	case VDEF_ENCODING_H265:
		ret = set_h265_ps(self, format);
		if (ret) {
			ULOG_ERRNO("set_h265_ps", -ret);
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
	int ret;
	struct venc_videotoolbox *self = outputCallbackRefCon;
	struct mbuf_raw_video_frame *in_frame = sourceFrameRefCon;
	struct mbuf_coded_video_frame *out_frame = NULL;
	CMBlockBufferRef bbuf = NULL;
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	if (status != noErr) {
		ULOG_ERRNO("encoder error %d", EPROTO, (int)status);
		goto out;
	}

	ULOG_ERRNO_RETURN_IF(sampleBuffer == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(in_frame == NULL, EINVAL);

	/* Discard the buffer when flushing with frames discarding */
	if ((atomic_load(&self->flushing)) &&
	    (atomic_load(&self->flush_discard))) {
		ULOGI("frame discarded (flushing)");
		goto out;
	}

	bbuf = CMSampleBufferGetDataBuffer(sampleBuffer);
	if (bbuf == NULL) {
		ret = -ENOSYS;
		ULOG_ERRNO("CMSampleBufferGetDataBuffer", -ret);
		goto out;
	}

	if (atomic_load(&self->ps_stored) == false) {
		ret = set_ps(self, sampleBuffer);
		if (ret < 0) {
			ULOG_ERRNO("set_ps", -ret);
			goto out;
		}
	}

	/* Set the metadata */
	ret = set_frame_metadata(self, in_frame, &out_frame, bbuf);
	if (ret < 0) {
		ULOG_ERRNO("set_frame_metadata", -ret);
		goto out;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		out_frame,
		VENC_ANCILLARY_KEY_OUTPUT_TIME,
		&ts_us,
		sizeof(ts_us));
	if (ret < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer", -ret);

	/* Output the frame */
	ret = mbuf_coded_video_frame_finalize(out_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_finalize", -ret);
		goto out;
	}

	ret = mbuf_coded_video_frame_queue_push(self->out_queue, out_frame);
	if (ret < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_queue_push", -ret);
out:
	/* Unref the buffers */
	if (in_frame)
		mbuf_raw_video_frame_unref(in_frame);
	if (out_frame)
		mbuf_coded_video_frame_unref(out_frame);
}


static void frame_data_release(void *release_ctx,
			       const void *data,
			       size_t size,
			       size_t plane_count,
			       const void *plane_adresses[])
{
	struct frame_data *in_frame_data = release_ctx;
	mbuf_raw_video_frame_release_packed_buffer(in_frame_data->frame,
						   in_frame_data->data);
	mbuf_raw_video_frame_unref(in_frame_data->frame);
	free(in_frame_data);
}


static int buffer_push_one(struct venc_videotoolbox *self,
			   struct mbuf_raw_video_frame *in_frame)
{
	int ret = 0;
	OSStatus osstatus;
	struct vdef_raw_frame info;
	const void *void_data;
	const uint8_t *in_data, *data_addr[3];
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;
	size_t len, buf_size, widths[3], heights[3];
	int plane_count, fmt;
	CVPixelBufferRef pixelBufferOut = NULL;
	CMTime presentationTimeStamp;
	struct frame_data *in_frame_data = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(in_frame == NULL, EINVAL);

	ret = mbuf_raw_video_frame_get_frame_info(in_frame, &info);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		return ret;
	}

	/* Frame skipping in case of decimation */
	if (self->input_frame_cnt % self->base->config.h264.decimation != 0) {
		self->input_frame_cnt++;
		return ret;
	}
	self->input_frame_cnt++;

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	ret = mbuf_raw_video_frame_add_ancillary_buffer(
		in_frame,
		VENC_ANCILLARY_KEY_DEQUEUE_TIME,
		&ts_us,
		sizeof(ts_us));
	if (ret < 0)
		ULOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer", -ret);

	if (!vdef_raw_format_intersect(
		    &info.format, supported_formats, NB_SUPPORTED_FORMATS)) {
		ret = -ENOSYS;
		ULOG_ERRNO(
			"unsupported format:"
			" " VDEF_RAW_FORMAT_TO_STR_FMT,
			-ret,
			VDEF_RAW_FORMAT_TO_STR_ARG(&info.format));
		return ret;
	}

	ret = mbuf_raw_video_frame_get_plane(in_frame, 0, &void_data, &len);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_plane_0", -ret);
		return ret;
	}

	in_data = void_data;
	widths[0] = info.info.resolution.width;
	heights[0] = info.info.resolution.height;
	data_addr[0] = (uint8_t *)in_data;

	ret = mbuf_raw_video_frame_release_plane(in_frame, 0, void_data);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_release_plane_0", -ret);
		return ret;
	}

	ret = mbuf_raw_video_frame_get_plane(in_frame, 1, &void_data, &len);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_plane_1", -ret);
		return ret;
	}

	in_data = void_data;
	widths[1] = (info.info.resolution.width + 1) / 2;
	heights[1] = (info.info.resolution.height + 1) / 2;
	data_addr[1] = (uint8_t *)in_data;

	ret = mbuf_raw_video_frame_release_plane(in_frame, 1, void_data);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_release_plane_1", -ret);
		return ret;
	}

	if (vdef_raw_format_cmp(&info.format, &vdef_nv12)) {
		plane_count = 2;
		if (self->base->config.input.info.full_range)
			fmt = kCVPixelFormatType_420YpCbCr8BiPlanarFullRange;
		else
			fmt = kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange;
	} else if (vdef_raw_format_cmp(&info.format, &vdef_i420)) {
		plane_count = 3;

		ret = mbuf_raw_video_frame_get_plane(
			in_frame, 2, &void_data, &len);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_get_plane_2", -ret);
			return ret;
		}

		in_data = void_data;
		widths[2] = (info.info.resolution.width + 1) / 2;
		heights[2] = (info.info.resolution.height + 1) / 2;
		data_addr[2] = (uint8_t *)in_data;

		ret = mbuf_raw_video_frame_release_plane(
			in_frame, 2, void_data);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_release_plane_2",
				   -ret);
			return ret;
		}

		if (self->base->config.input.info.full_range)
			fmt = kCVPixelFormatType_420YpCbCr8PlanarFullRange;
		else
			fmt = kCVPixelFormatType_420YpCbCr8Planar;
	}

	/* Create the pixel buffer */
	ret = mbuf_raw_video_frame_get_packed_buffer(
		in_frame, &void_data, &buf_size);
	if (ret != 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_packed_buffer", -ret);
		return ret;
	}
	in_data = void_data;
	in_frame_data = calloc(1, sizeof(struct frame_data));
	if (!in_frame_data) {
		ret = -ENOMEM;
		ULOG_ERRNO("calloc", -ret);
		goto out;
	}
	in_frame_data->frame = in_frame;
	in_frame_data->data = void_data;
	ret = mbuf_raw_video_frame_ref(in_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_ref", -ret);
		free(in_frame_data);
		goto out;
	}

	ret = CVPixelBufferCreateWithPlanarBytes(kCFAllocatorDefault,
						 info.info.resolution.width,
						 info.info.resolution.height,
						 fmt,
						 (uint8_t *)in_data,
						 buf_size,
						 plane_count,
						 (void **)data_addr,
						 widths,
						 heights,
						 info.plane_stride,
						 frame_data_release,
						 in_frame_data,
						 NULL,
						 &pixelBufferOut);
	if (ret < 0) {
		ULOG_ERRNO("CVPixelBufferCreateWithPlanarBytes", -ret);
		goto out;
	}

	/* Push the frame */
	presentationTimeStamp =
		CMTimeMake(info.info.timestamp, info.info.timescale);

	ret = mbuf_raw_video_frame_ref(in_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_ref", -ret);
		goto out;
	}

	osstatus = VTCompressionSessionEncodeFrame(self->compress_ref,
						   pixelBufferOut,
						   presentationTimeStamp,
						   kCMTimeIndefinite,
						   NULL,
						   in_frame,
						   NULL);
	if (osstatus != noErr) {
		ret = -EPROTO;
		ULOG_ERRNO("VTCompressionSessionEncodeFrame status=%d",
			   -ret,
			   (int)osstatus);
		goto out;
	}

out:
	if (pixelBufferOut)
		CFRelease(pixelBufferOut);
	if (void_data)
		mbuf_raw_video_frame_release_packed_buffer(in_frame, void_data);
	else if (in_frame_data)
		free(in_frame_data);
	return ret;
}


static void check_input_queue(struct venc_videotoolbox *self)
{
	int ret;
	struct mbuf_raw_video_frame *in_frame;

	ret = mbuf_raw_video_frame_queue_peek(self->in_queue, &in_frame);
	while (ret >= 0) {
		/* Push the input frame */
		/* Encode the frame */
		ret = buffer_push_one(self, in_frame);
		if (ret < 0) {
			if (ret != -EAGAIN)
				ULOG_ERRNO("encode_frame", -ret);
			ret = -ENOSPC;
			break;
		}
		if (in_frame) {
			mbuf_raw_video_frame_unref(in_frame);
			/* Pop the frame for real */
			ret = mbuf_raw_video_frame_queue_pop(self->in_queue,
							     &in_frame);
			if (ret < 0) {
				ULOG_ERRNO("mbuf_raw_video_frame_queue_pop",
					   -ret);
				break;
			}
			mbuf_raw_video_frame_unref(in_frame);
		}
		/* Peek the next frame */
		ret = mbuf_raw_video_frame_queue_peek(self->in_queue,
						      &in_frame);
		if (ret < 0 && ret != -EAGAIN && ret != -ENOSPC)
			ULOG_ERRNO("mbuf_raw_video_frame_queue_peek", -ret);
		if (self->flush && ret == -EAGAIN) {
			in_frame = NULL;
			ret = do_flush(self);
			if (ret < 0)
				ULOG_ERRNO("do_flush", -ret);
			break;
		}
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

	loop = pomp_loop_new();
	if (!loop) {
		ULOG_ERRNO("pomp_loop_new", ENOMEM);
		goto exit;
	}
	ret = mbuf_raw_video_frame_queue_get_event(self->in_queue,
						   &in_queue_evt);
	if (ret != 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_get_event", -ret);
		goto exit;
	}
	ret = pomp_evt_attach_to_loop(in_queue_evt, loop, input_event_cb, self);
	if (ret != 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto exit;
	}

	while ((!atomic_load(&self->should_stop)) || (flush)) {
		flush = atomic_load(&self->flush);
		/* Flush discarding all frames */
		if ((flush) && (atomic_load(&self->flush_discard))) {
			ret = do_flush(self);
			if (ret < 0)
				ULOG_ERRNO("do_flush", -ret);
			continue;
		}

		/* Get an input buffer (with timeout) */
		timeout = ((flush) && (!atomic_load(&self->flush_discard))) ? 0
									    : 5;
		ret = pomp_loop_wait_and_process(loop, timeout);
		if (ret < 0 && ret != -ETIMEDOUT) {
			ULOG_ERRNO("pomp_loop_wait_and_process", -ret);
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
		ULOG_ERRNO("mbox_push", -ret);

exit:
	if (in_queue_evt != NULL) {
		ret = pomp_evt_detach_from_loop(in_queue_evt, loop);
		if (ret != 0)
			ULOG_ERRNO("pomp_evt_detach_from_loop", -ret);
	}
	if (loop != NULL) {
		ret = pomp_loop_destroy(loop);
		if (ret != 0)
			ULOG_ERRNO("pomp_loop_destroy", -ret);
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


static int get_supported_input_formats(const struct vdef_raw_format **formats)
{
	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);
	*formats = supported_formats;
	return NB_SUPPORTED_FORMATS;
}


static int flush(struct venc_encoder *base, int discard_)
{
	struct venc_videotoolbox *self;
	bool discard = discard_;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	atomic_store(&self->flush, true);
	atomic_store(&self->flush_discard, discard);

	return 0;
}


static int stop(struct venc_encoder *base)
{
	struct venc_videotoolbox *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	/* Stop the encoding thread */
	atomic_store(&self->should_stop, true);

	return 0;
}


static int destroy(struct venc_encoder *base)
{
	int err;
	struct venc_videotoolbox *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;
	if (self == NULL)
		return 0;

	err = stop(base);
	if (err != 0)
		ULOG_ERRNO("stop", -err);
	if (self->compress_ref) {
		VTCompressionSessionInvalidate(self->compress_ref);
		CFRelease(self->compress_ref);
	}

	if (self->ps_lock_created) {
		err = pthread_mutex_destroy(&self->ps_lock);
		if (err != 0)
			ULOG_ERRNO("pthread_mutex_destroy", err);
	}

	if (self->thread_launched) {
		err = pthread_join(self->thread, NULL);
		if (err != 0)
			ULOG_ERRNO("pthread_join", err);
	}

	/* Free the resources */
	if (self->out_queue_evt != NULL &&
	    pomp_evt_is_attached(self->out_queue_evt, base->loop)) {
		err = pomp_evt_detach_from_loop(self->out_queue_evt,
						base->loop);
		if (err < 0)
			ULOG_ERRNO("pomp_evt_detach_from_loop", -err);
	}
	if (self->out_queue != NULL) {
		err = mbuf_coded_video_frame_queue_destroy(self->out_queue);
		if (err < 0)
			ULOG_ERRNO(
				"mbuf_coded_video_frame_queue_destroy:output",
				-err);
	}
	if (self->in_queue != NULL) {
		err = mbuf_raw_video_frame_queue_destroy(self->in_queue);
		if (err < 0)
			ULOG_ERRNO("mbuf_raw_video_frame_queue_destroy:input",
				   -err);
	}
	if (self->mbox != NULL) {
		err = pomp_loop_remove(base->loop,
				       mbox_get_read_fd(self->mbox));
		if (err < 0)
			ULOG_ERRNO("pomp_loop_remove", -err);
		mbox_destroy(self->mbox);
	}
	free(self);

	return 0;
}


static bool input_filter(struct mbuf_raw_video_frame *frame, void *userdata)
{
	int ret;
	const void *tmp;
	size_t tmplen;
	struct vdef_raw_frame info;
	struct venc_videotoolbox *self = userdata;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, false);

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
		ULOG_ERRNO("mbuf_raw_video_frame_get_packed_buffer", -ret);
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
	OSStatus osstatus;
	struct mbuf_raw_video_frame_queue_args queue_args = {
		.filter = input_filter,
	};
	CMVideoCodecType codec_type;
	CFMutableDictionaryRef buffer_attr = NULL;
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
	CFDictionaryRef properties = NULL;

	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	if (base->config.encoding != VDEF_ENCODING_H264 &&
	    base->config.encoding != VDEF_ENCODING_H265) {
		ret = -EINVAL;
		ULOG_ERRNO("unsupported encoding", -ret);
		return ret;
	}

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	base->derived = self;
	self->base = base;
	queue_args.filter_userdata = self;

	self->mbox = mbox_new(sizeof(struct venc_videotoolbox_message));
	if (self->mbox == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("mbox_new", -ret);
		goto error;
	}
	ret = pomp_loop_add(base->loop,
			    mbox_get_read_fd(self->mbox),
			    POMP_FD_EVENT_IN,
			    &mbox_cb,
			    self);
	if (ret < 0) {
		ULOG_ERRNO("pomp_loop_add", -ret);
		goto error;
	}

	ret = mbuf_coded_video_frame_queue_new(&self->out_queue);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_new:output", -ret);
		goto error;
	}
	ret = mbuf_coded_video_frame_queue_get_event(self->out_queue,
						     &self->out_queue_evt);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_get_event", -ret);
		goto error;
	}
	ret = pomp_evt_attach_to_loop(
		self->out_queue_evt, base->loop, &out_queue_evt_cb, self);
	if (ret < 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto error;
	}

	ret = mbuf_raw_video_frame_queue_new_with_args(&queue_args,
						       &self->in_queue);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_new_with_args:input",
			   -ret);
		goto error;
	}

	ret = pthread_mutex_init(&self->ps_lock, NULL);
	if (ret) {
		ULOG_ERRNO("pthread_mutex_init", ret);
		goto error;
	};
	self->ps_lock_created = true;

	ret = pthread_create(&self->thread, NULL, encoder_thread, self);
	if (ret != 0) {
		ret = -ret;
		ULOG_ERRNO("pthread_create", -ret);
		goto error;
	}
	self->thread_launched = true;

	switch (base->config.encoding) {
	case VDEF_ENCODING_H264:
		codec_type = kCMVideoCodecType_H264;
		break;
	case VDEF_ENCODING_H265:
		codec_type = kCMVideoCodecType_HEVC;
		break;
	default:
		break;
	}

	buffer_attr = buffer_attr_create(base->config.input.info.full_range);
	if (buffer_attr == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("buffer_attr_create", -ret);
		goto error;
	}

	osstatus = VTCompressionSessionCreate(
		kCFAllocatorDefault,
		base->config.input.info.resolution.width,
		base->config.input.info.resolution.height,
		codec_type,
		NULL,
		buffer_attr,
		kCFAllocatorDefault,
		frame_output_cb,
		self,
		&self->compress_ref);
	if (osstatus == kVTCouldNotFindVideoEncoderErr) {
		ret = -EINVAL;
		ULOG_ERRNO(
			"VTCompressionSessionCreate error: "
			"Encoding not supported status=%d",
			-ret,
			(int)osstatus);
		goto error;
	} else if (osstatus != noErr) {
		ret = -EPROTO;
		ULOG_ERRNO("VTCompressionSessionCreate status=%d",
			   -ret,
			   (int)osstatus);
		goto error;
	}

	properties = CFDictionaryCreate(
		kCFAllocatorDefault, keys, values, properties_size, NULL, NULL);
	if (properties == NULL) {
		ret = -ENOSYS;
		ULOG_ERRNO("CFDictionaryCreate", -ret);
		goto error;
	}

	osstatus = VTSessionSetProperties(self->compress_ref, properties);
	if (osstatus != noErr) {
		ret = -EPROTO;
		ULOG_ERRNO("VTSessionSetProperties status=%d",
			   EPROTO,
			   (int)osstatus);
		goto error;
	}

	if (buffer_attr)
		CFRelease(buffer_attr);
	if (properties)
		CFRelease(properties);
	return 0;

error:
	/* Cleanup on error */
	if (self->compress_ref != NULL) {
		VTCompressionSessionInvalidate(self->compress_ref);
		CFRelease(self->compress_ref);
	}
	if (buffer_attr != NULL)
		CFRelease(buffer_attr);
	if (properties)
		CFRelease(properties);
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
	struct venc_videotoolbox *self;

	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	self = base->derived;

	return self->in_queue;
}


static int get_dyn_config(struct venc_encoder *base,
			  struct venc_dyn_config *config)
{
	struct venc_videotoolbox *self = base->derived;

	*config = self->dynconf;

	return 0;
}


static int set_dyn_config(struct venc_encoder *base,
			  const struct venc_dyn_config *config)
{
	struct venc_videotoolbox *self = base->derived;
	OSStatus osstatus;
	CFNumberRef CFBitrate;

	if (config->qp != 0)
		return -ENOSYS;

	if (config->target_bitrate != 0) {
		unsigned int decimation = config->decimation == 0
						  ? self->dynconf.decimation
						  : config->decimation;
		uint32_t bitrate = config->target_bitrate * decimation;
		CFBitrate = CFNumberCreate(
			kCFAllocatorDefault, kCFNumberSInt32Type, &bitrate);
		if (CFBitrate == NULL) {
			ULOG_ERRNO("CFNumberCreate", ENOMEM);
			return -ENOMEM;
		}
		osstatus = VTSessionSetProperty(
			self->compress_ref,
			kVTCompressionPropertyKey_AverageBitRate,
			CFBitrate);
		if (osstatus != noErr) {
			ULOG_ERRNO("VTSessionSetProperty status=%d",
				   EPROTO,
				   (int)osstatus);
			return -EPROTO;
		}
		self->dynconf.target_bitrate = config->target_bitrate;
	}

	if (config->decimation != 0)
		self->dynconf.decimation = config->decimation;

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
};
