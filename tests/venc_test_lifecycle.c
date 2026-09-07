/**
 * Copyright (c) 2026 Parrot Drones SAS
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

#include "venc_test.h"

/*
 * Lifecycle and API-consistency tests. Tests are named <feature>-<implem> so
 * that all implementations of a given feature appear together in CUnit output,
 * making cross-implem consistency gaps immediately visible.
 *
 * Three generic lifecycle behaviors are verified for every backend:
 *   frame-output-and-drain   push N frames, drain, all N come out
 *   flush-discard            push N frames, discard flush, clean stop
 *   flush-then-reencode      drain, then push and drain a second batch
 *
 * Three API functions are verified for every backend:
 *   dyn-config     venc_get_dyn_config / venc_set_dyn_config
 *   request-idr    venc_request_idr
 *   get-h264-ps    venc_get_h264_ps
 *
 * ffmpeg and x265 are skipped gracefully at runtime when unavailable.
 */

#define LIFECYCLE_WIDTH 64
#define LIFECYCLE_HEIGHT 64
#define LIFECYCLE_Y_SIZE (LIFECYCLE_WIDTH * LIFECYCLE_HEIGHT)
#define LIFECYCLE_UV_SIZE ((LIFECYCLE_WIDTH / 2) * (LIFECYCLE_HEIGHT / 2))
#define LIFECYCLE_FRAME_SIZE (LIFECYCLE_Y_SIZE + 2 * LIFECYCLE_UV_SIZE)
#define PNG_FRAME_SIZE (LIFECYCLE_WIDTH * LIFECYCLE_HEIGHT)
#define LIFECYCLE_RGB_FRAME_SIZE (LIFECYCLE_WIDTH * LIFECYCLE_HEIGHT * 3)

/*
 * ffmpeg alone: venc_ffmpeg.c's get_preferred_backend_by_encoding() always
 * prefers HW encoders (h264_nvenc/hevc_nvenc) over software ones when the
 * probe at a fixed 1280x720 succeeds, and fill_lifecycle_ffmpeg_config() has
 * no way to steer it back to a software backend. NVENC enforces its own
 * hardware minimum encode dimensions (empirically ~148 width / ~50 height on
 * a Blackwell RTX 5080; varies by GPU/driver generation), well above
 * LIFECYCLE_WIDTH/HEIGHT. Keep those small for the CPU backends (x264/x265/
 * turbojpeg/png) and give ffmpeg its own larger resolution instead of
 * bumping everyone, since CPU encode time scales with pixel count.
 */
#define FFMPEG_LIFECYCLE_WIDTH 320
#define FFMPEG_LIFECYCLE_HEIGHT 240
#define FFMPEG_LIFECYCLE_Y_SIZE                                                \
	(FFMPEG_LIFECYCLE_WIDTH * FFMPEG_LIFECYCLE_HEIGHT)
#define FFMPEG_LIFECYCLE_UV_SIZE                                               \
	((FFMPEG_LIFECYCLE_WIDTH / 2) * (FFMPEG_LIFECYCLE_HEIGHT / 2))
#define FFMPEG_LIFECYCLE_FRAME_SIZE                                            \
	(FFMPEG_LIFECYCLE_Y_SIZE + 2 * FFMPEG_LIFECYCLE_UV_SIZE)

#define PUMP_TIMEOUT_MS 100
#define PUMP_MAX_ITER 150


/* ------------------------------------------------------------------ */
/* Config fill functions — one per implem                              */
/* ------------------------------------------------------------------ */

static void fill_lifecycle_x264_config(struct venc_config *config)
{
	memset(config, 0, sizeof(*config));
	config->implem = VENC_ENCODER_IMPLEM_X264;
	config->encoding = VDEF_ENCODING_H264;
	config->input.format = vdef_i420;
	config->input.info.resolution.width = LIFECYCLE_WIDTH;
	config->input.info.resolution.height = LIFECYCLE_HEIGHT;
	config->input.info.framerate.num = 30;
	config->input.info.framerate.den = 1;
	config->h264.rate_control = VENC_RATE_CONTROL_CBR;
	config->h264.max_bitrate = 500000;
	config->output.preferred_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;
}

static void fill_lifecycle_x265_config(struct venc_config *config)
{
	memset(config, 0, sizeof(*config));
	config->implem = VENC_ENCODER_IMPLEM_X265;
	config->encoding = VDEF_ENCODING_H265;
	config->input.format = vdef_i420;
	config->input.info.resolution.width = LIFECYCLE_WIDTH;
	config->input.info.resolution.height = LIFECYCLE_HEIGHT;
	config->input.info.framerate.num = 30;
	config->input.info.framerate.den = 1;
	config->h265.rate_control = VENC_RATE_CONTROL_CBR;
	config->h265.max_bitrate = 500000;
	/* Exercise all optional-NALU paths: insert_aud fires unconditionally;
	 * the others fire on IDR/I/P_IR_START (first frame of a fresh encoder).
	 */
	config->h265.insert_aud = 1;
	config->h265.insert_ps = 1;
	config->h265.insert_time_code_sei = 1;
	config->h265.insert_mdcv_sei = 1;
	config->h265.insert_cll_sei = 1;
	config->input.info.mdcv.display_primaries = VDEF_COLOR_PRIMARIES_BT709;
	config->input.info.mdcv.max_display_mastering_luminance = 1000.f;
	config->input.info.mdcv.min_display_mastering_luminance = 0.01f;
	config->input.info.cll.max_cll = 1000;
	config->input.info.cll.max_fall = 400;
}

static void fill_lifecycle_turbojpeg_config(struct venc_config *config)
{
	memset(config, 0, sizeof(*config));
	config->implem = VENC_ENCODER_IMPLEM_TURBOJPEG;
	config->encoding = VDEF_ENCODING_MJPEG;
	config->input.format = vdef_i420;
	config->input.info.resolution.width = LIFECYCLE_WIDTH;
	config->input.info.resolution.height = LIFECYCLE_HEIGHT;
	config->input.info.framerate.num = 30;
	config->input.info.framerate.den = 1;
	config->mjpeg.rate_control = VENC_RATE_CONTROL_CQ;
	config->mjpeg.quality = 80;
}

static void fill_lifecycle_png_config(struct venc_config *config)
{
	memset(config, 0, sizeof(*config));
	config->implem = VENC_ENCODER_IMPLEM_PNG;
	config->encoding = VDEF_ENCODING_PNG;
	config->input.format = vdef_gray;
	config->input.info.resolution.width = LIFECYCLE_WIDTH;
	config->input.info.resolution.height = LIFECYCLE_HEIGHT;
	config->input.info.framerate.num = 30;
	config->input.info.framerate.den = 1;
	/* libpng rejects bit_depth == 0 (the memset default). */
	config->input.info.bit_depth = 8;
	config->png.compression_level = 6;
}

static void fill_lifecycle_ffmpeg_config(struct venc_config *config)
{
	memset(config, 0, sizeof(*config));
	config->implem = VENC_ENCODER_IMPLEM_FFMPEG;
	config->encoding = VDEF_ENCODING_H264;
	config->input.format = vdef_i420;
	config->input.info.resolution.width = FFMPEG_LIFECYCLE_WIDTH;
	config->input.info.resolution.height = FFMPEG_LIFECYCLE_HEIGHT;
	config->input.info.framerate.num = 30;
	config->input.info.framerate.den = 1;
	config->h264.rate_control = VENC_RATE_CONTROL_CBR;
	config->h264.max_bitrate = 500000;
	/* ffmpeg does NOT default UNKNOWN to BYTE_STREAM (unlike x264/x265). */
	config->output.preferred_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;
}

/*
 * ffmpeg's H.265 support (hevc_nvenc) is HW-only (see s_backend_map in
 * venc_ffmpeg.c: no software H.265 backend is listed there) — only usable
 * when an NVIDIA GPU/driver is present. Tests using this config must guard
 * with encoding_supported(VENC_ENCODER_IMPLEM_FFMPEG, VDEF_ENCODING_H265).
 */
static void fill_lifecycle_ffmpeg_h265_config(struct venc_config *config)
{
	memset(config, 0, sizeof(*config));
	config->implem = VENC_ENCODER_IMPLEM_FFMPEG;
	config->encoding = VDEF_ENCODING_H265;
	config->input.format = vdef_i420;
	config->input.info.resolution.width = FFMPEG_LIFECYCLE_WIDTH;
	config->input.info.resolution.height = FFMPEG_LIFECYCLE_HEIGHT;
	config->input.info.framerate.num = 30;
	config->input.info.framerate.den = 1;
	config->h265.rate_control = VENC_RATE_CONTROL_CBR;
	config->h265.max_bitrate = 500000;
	config->output.preferred_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;
}


/* ------------------------------------------------------------------ */
/* Frame push helpers                                                  */
/* ------------------------------------------------------------------ */

/*
 * Attach a minimal vmeta_frame (type V3) to an input raw frame so that
 * venc_call_frame_output_cb (venc_core.c) copies it to the coded output frame
 * via mbuf_coded_video_frame_set_metadata (lines 169-173 of venc_core.c).
 * Errors are intentionally ignored: coverage is the goal, not strict
 * metadata correctness.
 */
static void attach_test_vmeta(struct mbuf_raw_video_frame *frame)
{
	struct vmeta_frame *meta = NULL;
	if (vmeta_frame_new(VMETA_FRAME_TYPE_V3, &meta) < 0)
		return;
	mbuf_raw_video_frame_set_metadata(frame, meta);
	vmeta_frame_unref(meta);
}


static int push_frame(struct mbuf_raw_video_frame_queue *queue,
		      unsigned int index)
{
	int res;
	struct vdef_raw_frame frame_info;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame = NULL;
	void *data = NULL;
	size_t capacity = 0;

	res = mbuf_mem_generic_new(LIFECYCLE_FRAME_SIZE, &mem);
	if (res < 0)
		return res;

	res = mbuf_mem_get_data(mem, &data, &capacity);
	if (res < 0)
		goto out;
	memset(data, 0x10 + (index % 16), capacity);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_i420;
	frame_info.info.resolution.width = LIFECYCLE_WIDTH;
	frame_info.info.resolution.height = LIFECYCLE_HEIGHT;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = (uint64_t)index * 1000000 / 30;
	frame_info.info.capture_timestamp = frame_info.info.timestamp;
	frame_info.info.index = index;
	frame_info.plane_stride[0] = LIFECYCLE_WIDTH;
	frame_info.plane_stride[1] = LIFECYCLE_WIDTH / 2;
	frame_info.plane_stride[2] = LIFECYCLE_WIDTH / 2;

	res = mbuf_raw_video_frame_new(&frame_info, &frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_set_plane(
		frame, 0, mem, 0, LIFECYCLE_Y_SIZE);
	if (res < 0)
		goto out;
	res = mbuf_raw_video_frame_set_plane(
		frame, 1, mem, LIFECYCLE_Y_SIZE, LIFECYCLE_UV_SIZE);
	if (res < 0)
		goto out;
	res = mbuf_raw_video_frame_set_plane(frame,
					     2,
					     mem,
					     LIFECYCLE_Y_SIZE +
						     LIFECYCLE_UV_SIZE,
					     LIFECYCLE_UV_SIZE);
	if (res < 0)
		goto out;

	attach_test_vmeta(frame);

	res = mbuf_raw_video_frame_finalize(frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_queue_push(queue, frame);

out:
	if (frame != NULL)
		mbuf_raw_video_frame_unref(frame);
	if (mem != NULL)
		mbuf_mem_unref(mem);
	return res;
}


/*
 * Same as push_frame(), sized for FFMPEG_LIFECYCLE_WIDTH/HEIGHT instead of
 * LIFECYCLE_WIDTH/HEIGHT (see the comment on those macros above).
 */
static int push_ffmpeg_frame(struct mbuf_raw_video_frame_queue *queue,
			     unsigned int index)
{
	int res;
	struct vdef_raw_frame frame_info;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame = NULL;
	void *data = NULL;
	size_t capacity = 0;

	res = mbuf_mem_generic_new(FFMPEG_LIFECYCLE_FRAME_SIZE, &mem);
	if (res < 0)
		return res;

	res = mbuf_mem_get_data(mem, &data, &capacity);
	if (res < 0)
		goto out;
	memset(data, 0x10 + (index % 16), capacity);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_i420;
	frame_info.info.resolution.width = FFMPEG_LIFECYCLE_WIDTH;
	frame_info.info.resolution.height = FFMPEG_LIFECYCLE_HEIGHT;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = (uint64_t)index * 1000000 / 30;
	frame_info.info.capture_timestamp = frame_info.info.timestamp;
	frame_info.info.index = index;
	frame_info.plane_stride[0] = FFMPEG_LIFECYCLE_WIDTH;
	frame_info.plane_stride[1] = FFMPEG_LIFECYCLE_WIDTH / 2;
	frame_info.plane_stride[2] = FFMPEG_LIFECYCLE_WIDTH / 2;

	res = mbuf_raw_video_frame_new(&frame_info, &frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_set_plane(
		frame, 0, mem, 0, FFMPEG_LIFECYCLE_Y_SIZE);
	if (res < 0)
		goto out;
	res = mbuf_raw_video_frame_set_plane(frame,
					     1,
					     mem,
					     FFMPEG_LIFECYCLE_Y_SIZE,
					     FFMPEG_LIFECYCLE_UV_SIZE);
	if (res < 0)
		goto out;
	res = mbuf_raw_video_frame_set_plane(frame,
					     2,
					     mem,
					     FFMPEG_LIFECYCLE_Y_SIZE +
						     FFMPEG_LIFECYCLE_UV_SIZE,
					     FFMPEG_LIFECYCLE_UV_SIZE);
	if (res < 0)
		goto out;

	attach_test_vmeta(frame);

	res = mbuf_raw_video_frame_finalize(frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_queue_push(queue, frame);

out:
	if (frame != NULL)
		mbuf_raw_video_frame_unref(frame);
	if (mem != NULL)
		mbuf_mem_unref(mem);
	return res;
}


static int push_gray_frame(struct mbuf_raw_video_frame_queue *queue,
			   unsigned int index)
{
	int res;
	struct vdef_raw_frame frame_info;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame = NULL;
	void *data = NULL;
	size_t capacity = 0;

	res = mbuf_mem_generic_new(PNG_FRAME_SIZE, &mem);
	if (res < 0)
		return res;

	res = mbuf_mem_get_data(mem, &data, &capacity);
	if (res < 0)
		goto out;
	memset(data, 0x10 + (index % 16), capacity);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_gray;
	frame_info.info.resolution.width = LIFECYCLE_WIDTH;
	frame_info.info.resolution.height = LIFECYCLE_HEIGHT;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = (uint64_t)index * 1000000 / 30;
	frame_info.info.capture_timestamp = frame_info.info.timestamp;
	frame_info.info.index = index;
	frame_info.plane_stride[0] = LIFECYCLE_WIDTH;

	res = mbuf_raw_video_frame_new(&frame_info, &frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_set_plane(frame, 0, mem, 0, PNG_FRAME_SIZE);
	if (res < 0)
		goto out;

	attach_test_vmeta(frame);

	res = mbuf_raw_video_frame_finalize(frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_queue_push(queue, frame);

out:
	if (frame != NULL)
		mbuf_raw_video_frame_unref(frame);
	if (mem != NULL)
		mbuf_mem_unref(mem);
	return res;
}


/*
 * Same as push_gray_frame(), sized for FFMPEG_LIFECYCLE_WIDTH/HEIGHT (see
 * test_frame_output_and_drain_ffmpeg_gray, the only ffmpeg test exercising
 * this format). Gray has no chroma plane, so its frame size is just
 * width*height, same value as FFMPEG_LIFECYCLE_Y_SIZE.
 */
static int push_ffmpeg_gray_frame(struct mbuf_raw_video_frame_queue *queue,
				  unsigned int index)
{
	int res;
	struct vdef_raw_frame frame_info;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame = NULL;
	void *data = NULL;
	size_t capacity = 0;

	res = mbuf_mem_generic_new(FFMPEG_LIFECYCLE_Y_SIZE, &mem);
	if (res < 0)
		return res;

	res = mbuf_mem_get_data(mem, &data, &capacity);
	if (res < 0)
		goto out;
	memset(data, 0x10 + (index % 16), capacity);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_gray;
	frame_info.info.resolution.width = FFMPEG_LIFECYCLE_WIDTH;
	frame_info.info.resolution.height = FFMPEG_LIFECYCLE_HEIGHT;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = (uint64_t)index * 1000000 / 30;
	frame_info.info.capture_timestamp = frame_info.info.timestamp;
	frame_info.info.index = index;
	frame_info.plane_stride[0] = FFMPEG_LIFECYCLE_WIDTH;

	res = mbuf_raw_video_frame_new(&frame_info, &frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_set_plane(
		frame, 0, mem, 0, FFMPEG_LIFECYCLE_Y_SIZE);
	if (res < 0)
		goto out;

	attach_test_vmeta(frame);

	res = mbuf_raw_video_frame_finalize(frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_queue_push(queue, frame);

out:
	if (frame != NULL)
		mbuf_raw_video_frame_unref(frame);
	if (mem != NULL)
		mbuf_mem_unref(mem);
	return res;
}


/*
 * RGB variant: 1 packed plane, stride = W*3, size = W*H*3.
 */
static int push_rgb_frame(struct mbuf_raw_video_frame_queue *queue,
			  unsigned int index)
{
	int res;
	struct vdef_raw_frame frame_info;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame = NULL;
	void *data = NULL;
	size_t capacity = 0;

	res = mbuf_mem_generic_new(LIFECYCLE_RGB_FRAME_SIZE, &mem);
	if (res < 0)
		return res;

	res = mbuf_mem_get_data(mem, &data, &capacity);
	if (res < 0)
		goto out;
	memset(data, 0x80 + (index % 64), capacity);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_rgb;
	frame_info.info.resolution.width = LIFECYCLE_WIDTH;
	frame_info.info.resolution.height = LIFECYCLE_HEIGHT;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = (uint64_t)index * 1000000 / 30;
	frame_info.info.capture_timestamp = frame_info.info.timestamp;
	frame_info.info.index = index;
	frame_info.plane_stride[0] = LIFECYCLE_WIDTH * 3;

	res = mbuf_raw_video_frame_new(&frame_info, &frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_set_plane(
		frame, 0, mem, 0, LIFECYCLE_RGB_FRAME_SIZE);
	if (res < 0)
		goto out;

	attach_test_vmeta(frame);

	res = mbuf_raw_video_frame_finalize(frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_queue_push(queue, frame);

out:
	if (frame != NULL)
		mbuf_raw_video_frame_unref(frame);
	if (mem != NULL)
		mbuf_mem_unref(mem);
	return res;
}


/*
 * NV12 variant: 2 planes (Y full-resolution, UV interleaved half-height).
 * UV plane stride = W (both components packed), UV size = W*(H/2).
 * Total allocation is the same as I420 (LIFECYCLE_FRAME_SIZE).
 */
static int push_nv12_frame(struct mbuf_raw_video_frame_queue *queue,
			   unsigned int index)
{
	int res;
	struct vdef_raw_frame frame_info;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame = NULL;
	void *data = NULL;
	size_t capacity = 0;

	res = mbuf_mem_generic_new(LIFECYCLE_FRAME_SIZE, &mem);
	if (res < 0)
		return res;

	res = mbuf_mem_get_data(mem, &data, &capacity);
	if (res < 0)
		goto out;
	memset(data, 0x10 + (index % 16), capacity);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_nv12;
	frame_info.info.resolution.width = LIFECYCLE_WIDTH;
	frame_info.info.resolution.height = LIFECYCLE_HEIGHT;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = (uint64_t)index * 1000000 / 30;
	frame_info.info.capture_timestamp = frame_info.info.timestamp;
	frame_info.info.index = index;
	frame_info.plane_stride[0] = LIFECYCLE_WIDTH;
	frame_info.plane_stride[1] = LIFECYCLE_WIDTH; /* UV interleaved */

	res = mbuf_raw_video_frame_new(&frame_info, &frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_set_plane(
		frame, 0, mem, 0, LIFECYCLE_Y_SIZE);
	if (res < 0)
		goto out;
	/* UV plane: W*(H/2) = 2 * LIFECYCLE_UV_SIZE */
	res = mbuf_raw_video_frame_set_plane(
		frame, 1, mem, LIFECYCLE_Y_SIZE, 2 * LIFECYCLE_UV_SIZE);
	if (res < 0)
		goto out;

	attach_test_vmeta(frame);

	res = mbuf_raw_video_frame_finalize(frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_queue_push(queue, frame);

out:
	if (frame != NULL)
		mbuf_raw_video_frame_unref(frame);
	if (mem != NULL)
		mbuf_mem_unref(mem);
	return res;
}


/*
 * NV21 variant: identical plane layout to NV12 (Y + interleaved half-height
 * plane, same strides and sizes) but with vdef_nv21, which makes the x264
 * backend set X264_CSP_NV21 instead of X264_CSP_NV12 (distinct code path).
 */
static int push_nv21_frame(struct mbuf_raw_video_frame_queue *queue,
			   unsigned int index)
{
	int res;
	struct vdef_raw_frame frame_info;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame = NULL;
	void *data = NULL;
	size_t capacity = 0;

	res = mbuf_mem_generic_new(LIFECYCLE_FRAME_SIZE, &mem);
	if (res < 0)
		return res;

	res = mbuf_mem_get_data(mem, &data, &capacity);
	if (res < 0)
		goto out;
	memset(data, 0x10 + (index % 16), capacity);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_nv21;
	frame_info.info.resolution.width = LIFECYCLE_WIDTH;
	frame_info.info.resolution.height = LIFECYCLE_HEIGHT;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = (uint64_t)index * 1000000 / 30;
	frame_info.info.capture_timestamp = frame_info.info.timestamp;
	frame_info.info.index = index;
	frame_info.plane_stride[0] = LIFECYCLE_WIDTH;
	frame_info.plane_stride[1] = LIFECYCLE_WIDTH; /* VU interleaved */

	res = mbuf_raw_video_frame_new(&frame_info, &frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_set_plane(
		frame, 0, mem, 0, LIFECYCLE_Y_SIZE);
	if (res < 0)
		goto out;
	res = mbuf_raw_video_frame_set_plane(
		frame, 1, mem, LIFECYCLE_Y_SIZE, 2 * LIFECYCLE_UV_SIZE);
	if (res < 0)
		goto out;

	attach_test_vmeta(frame);

	res = mbuf_raw_video_frame_finalize(frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_queue_push(queue, frame);

out:
	if (frame != NULL)
		mbuf_raw_video_frame_unref(frame);
	if (mem != NULL)
		mbuf_mem_unref(mem);
	return res;
}


/*
 * Like push_frame but also attaches 16 bytes of dummy USERDATA_SEI ancillary
 * data to the raw frame before finalization. Required when the encoder config
 * has serialize_user_data = 1: in that path venc_h264/h265_generate_nalus
 * calls mbuf_coded_video_frame_get_ancillary_data and hard-fails if the key
 * is absent (H264) or silently skips it (H265). The backend copies ancillary
 * data from the input raw frame to the output coded frame automatically, so
 * attaching it here is sufficient.
 */
static int
push_frame_with_userdata_sei(struct mbuf_raw_video_frame_queue *queue,
			     unsigned int index)
{
	int res;
	struct vdef_raw_frame frame_info;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame = NULL;
	void *data = NULL;
	size_t capacity = 0;
	static const uint8_t sei_buf[16] = {
		0x00,
		0x11,
		0x22,
		0x33,
		0x44,
		0x55,
		0x66,
		0x77,
		0x88,
		0x99,
		0xaa,
		0xbb,
		0xcc,
		0xdd,
		0xee,
		0xff,
	};

	res = mbuf_mem_generic_new(LIFECYCLE_FRAME_SIZE, &mem);
	if (res < 0)
		return res;

	res = mbuf_mem_get_data(mem, &data, &capacity);
	if (res < 0)
		goto out;
	memset(data, 0x10 + (index % 16), capacity);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_i420;
	frame_info.info.resolution.width = LIFECYCLE_WIDTH;
	frame_info.info.resolution.height = LIFECYCLE_HEIGHT;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = (uint64_t)index * 1000000 / 30;
	frame_info.info.capture_timestamp = frame_info.info.timestamp;
	frame_info.info.index = index;
	frame_info.plane_stride[0] = LIFECYCLE_WIDTH;
	frame_info.plane_stride[1] = LIFECYCLE_WIDTH / 2;
	frame_info.plane_stride[2] = LIFECYCLE_WIDTH / 2;

	res = mbuf_raw_video_frame_new(&frame_info, &frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_set_plane(
		frame, 0, mem, 0, LIFECYCLE_Y_SIZE);
	if (res < 0)
		goto out;
	res = mbuf_raw_video_frame_set_plane(
		frame, 1, mem, LIFECYCLE_Y_SIZE, LIFECYCLE_UV_SIZE);
	if (res < 0)
		goto out;
	res = mbuf_raw_video_frame_set_plane(frame,
					     2,
					     mem,
					     LIFECYCLE_Y_SIZE +
						     LIFECYCLE_UV_SIZE,
					     LIFECYCLE_UV_SIZE);
	if (res < 0)
		goto out;

	/* Attach ancillary data before finalize (finalize is write-locking) */
	res = mbuf_raw_video_frame_add_ancillary_buffer(
		frame,
		MBUF_ANCILLARY_KEY_USERDATA_SEI,
		sei_buf,
		sizeof(sei_buf));
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_finalize(frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_queue_push(queue, frame);

out:
	if (frame != NULL)
		mbuf_raw_video_frame_unref(frame);
	if (mem != NULL)
		mbuf_mem_unref(mem);
	return res;
}


/*
 * I420 frame whose declared resolution differs from the encoder config.
 * The memory layout is valid (LIFECYCLE_FRAME_SIZE) — only the metadata
 * is wrong.  Frame creation and finalize succeed; the rejection happens at
 * push time via the queue filter (venc_default_input_filter_internal,
 * venc_core.c:318, !vdef_dim_cmp branch → -EPROTO).
 */
static int push_frame_wrong_resolution(struct mbuf_raw_video_frame_queue *queue,
				       unsigned int index)
{
	int res;
	struct vdef_raw_frame frame_info;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame = NULL;
	void *data = NULL;
	size_t capacity = 0;

	res = mbuf_mem_generic_new(LIFECYCLE_FRAME_SIZE, &mem);
	if (res < 0)
		return res;

	res = mbuf_mem_get_data(mem, &data, &capacity);
	if (res < 0)
		goto out;
	memset(data, 0, capacity);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_i420;
	/* Declare width*2 so vdef_dim_cmp fails against the encoder config */
	frame_info.info.resolution.width = LIFECYCLE_WIDTH * 2;
	frame_info.info.resolution.height = LIFECYCLE_HEIGHT;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = (uint64_t)index * 1000000 / 30;
	frame_info.info.capture_timestamp = frame_info.info.timestamp;
	frame_info.info.index = index;
	frame_info.plane_stride[0] = LIFECYCLE_WIDTH;
	frame_info.plane_stride[1] = LIFECYCLE_WIDTH / 2;
	frame_info.plane_stride[2] = LIFECYCLE_WIDTH / 2;

	res = mbuf_raw_video_frame_new(&frame_info, &frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_set_plane(
		frame, 0, mem, 0, LIFECYCLE_Y_SIZE);
	if (res < 0)
		goto out;
	res = mbuf_raw_video_frame_set_plane(
		frame, 1, mem, LIFECYCLE_Y_SIZE, LIFECYCLE_UV_SIZE);
	if (res < 0)
		goto out;
	res = mbuf_raw_video_frame_set_plane(frame,
					     2,
					     mem,
					     LIFECYCLE_Y_SIZE +
						     LIFECYCLE_UV_SIZE,
					     LIFECYCLE_UV_SIZE);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_finalize(frame);
	if (res < 0)
		goto out;

	res = mbuf_raw_video_frame_queue_push(queue, frame);

out:
	if (frame != NULL)
		mbuf_raw_video_frame_unref(frame);
	if (mem != NULL)
		mbuf_mem_unref(mem);
	return res;
}


/* ------------------------------------------------------------------ */
/* Loop pump helpers                                                   */
/* ------------------------------------------------------------------ */

static void pump_until(struct pomp_loop *loop, bool *flag, int max_iter)
{
	for (int i = 0; (i < max_iter) && !(*flag); i++)
		pomp_loop_wait_and_process(loop, PUMP_TIMEOUT_MS);
}


static void
pump_until_count(struct pomp_loop *loop, int *count, int target, int max_iter)
{
	for (int i = 0; (i < max_iter) && (*count < target); i++)
		pomp_loop_wait_and_process(loop, PUMP_TIMEOUT_MS);
}


/* ------------------------------------------------------------------ */
/* Push function dispatch                                              */
/* ------------------------------------------------------------------ */

typedef int (*push_frame_fn)(struct mbuf_raw_video_frame_queue *, unsigned int);

static push_frame_fn get_push_fn(const struct venc_config *config)
{
	/*
	 * Dispatch on the configured resolution rather than config->implem:
	 * ffmpeg configs use FFMPEG_LIFECYCLE_WIDTH/HEIGHT to stay clear of
	 * NVENC's hardware minimum (see the comment on those macros), and so
	 * does the VENC_ENCODER_IMPLEM_AUTO test (test_frame_output_and_drain_
	 * implem_auto_h264) since AUTO can itself resolve to ffmpeg — at which
	 * point config->implem is still AUTO, not FFMPEG, so checking implem
	 * here would miss it. Only I420 and gray are exercised at this size.
	 */
	if (config->input.info.resolution.width == FFMPEG_LIFECYCLE_WIDTH) {
		if (vdef_raw_format_cmp(&config->input.format, &vdef_gray))
			return push_ffmpeg_gray_frame;
		return push_ffmpeg_frame;
	}
	if (vdef_raw_format_cmp(&config->input.format, &vdef_gray))
		return push_gray_frame;
	if (vdef_raw_format_cmp(&config->input.format, &vdef_nv12))
		return push_nv12_frame;
	if (vdef_raw_format_cmp(&config->input.format, &vdef_nv21))
		return push_nv21_frame;
	if (vdef_raw_format_cmp(&config->input.format, &vdef_rgb))
		return push_rgb_frame;
	return push_frame;
}


/* ------------------------------------------------------------------ */
/* Runtime availability checks are provided by venc_test.h */


/* ================================================================== */
/* LIFECYCLE RUNNERS                                                   */
/* ================================================================== */

/*
 * Push 3 frames, drain flush, assert all 3 come out, then stop cleanly.
 *
 * venc_stop() completion is gated on venc_count_unreleased_frames() == 0:
 * mock_venc_cbs_ctx_clear() releases all captured output frames before stop.
 */
static void run_lifecycle_frame_output_and_drain(struct venc_config *config)
{
	int res;
	push_frame_fn push = get_push_fn(config);
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;
	const unsigned int frame_count = 3;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(enc);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	for (unsigned int i = 0; i < frame_count; i++) {
		res = push(queue, i);
		CU_ASSERT_EQUAL(res, 0);
	}

	res = venc_flush(enc, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, (int)frame_count);

	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/*
 * Push 3 frames then immediately discard-flush. Frame count is racy so only
 * flush completion and clean stop are asserted.
 */
static void run_lifecycle_flush_discard(struct venc_config *config)
{
	int res;
	push_frame_fn push = get_push_fn(config);
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	for (unsigned int i = 0; i < 3; i++) {
		res = push(queue, i);
		CU_ASSERT_EQUAL(res, 0);
	}

	res = venc_flush(enc, 1 /* discard */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);

	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/*
 * Two push+drain cycles on the same encoder. Verifies that every backend
 * accepts new frames after a drain without needing to be destroyed.
 * x264/x265/ffmpeg achieve this via an internal reopen; stateless backends
 * (turbojpeg, png) satisfy it trivially.
 */
static void run_lifecycle_flush_then_reencode(struct venc_config *config)
{
	int res;
	push_frame_fn push = get_push_fn(config);
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;
	const unsigned int frames_before = 3;
	const unsigned int frames_after = 3;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	for (unsigned int i = 0; i < frames_before; i++) {
		res = push(queue, i);
		CU_ASSERT_EQUAL(res, 0);
	}
	res = venc_flush(enc, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, (int)frames_before);

	mock_venc_cbs_ctx_clear(&ctx);

	for (unsigned int i = 0; i < frames_after; i++) {
		res = push(queue, frames_before + i);
		CU_ASSERT_EQUAL(res, 0);
	}
	res = venc_flush(enc, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, (int)frames_after);

	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/*
 * Discard-flush then re-encode on the same encoder.
 *
 * Differs from run_lifecycle_flush_then_reencode (drain→drain) in that the
 * first flush is a *discard*.  Verifies that every backend correctly resets
 * its internal state after a discard flush and accepts a fresh batch of frames
 * without needing to be destroyed.  The discard output count is deliberately
 * not asserted (racy), but the second batch must produce exactly N frames.
 */
static void
run_lifecycle_flush_discard_then_reencode(struct venc_config *config)
{
	int res;
	push_frame_fn push = get_push_fn(config);
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;
	const unsigned int frames_after = 3;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	for (unsigned int i = 0; i < 3; i++) {
		res = push(queue, i);
		CU_ASSERT_EQUAL(res, 0);
	}

	/* Discard flush: internal state reset, frame count is racy */
	res = venc_flush(enc, 1 /* discard */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);

	mock_venc_cbs_ctx_clear(&ctx);

	/* Second batch: must come out in full after drain */
	for (unsigned int i = 0; i < frames_after; i++) {
		res = push(queue, 3 + i);
		CU_ASSERT_EQUAL(res, 0);
	}
	res = venc_flush(enc, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, (int)frames_after);

	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/*
 * set_dyn_config during active encoding (hot-update path).
 *
 * Push one batch of frames, call set_dyn_config while those frames may still
 * be in the encoder pipeline, then push a second batch and drain.  All frames
 * from both batches must come out.  This exercises the "update applied to the
 * next frame the worker thread processes" path that the pre-encode dyn-config
 * tests never reach.
 */
static void run_lifecycle_dyn_config_mid_encode(struct venc_config *config)
{
	int res;
	push_frame_fn push = get_push_fn(config);
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;
	struct venc_dyn_config dyn;
	const unsigned int frames_before = 3;
	const unsigned int frames_after = 3;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	/* First batch: push without draining first */
	for (unsigned int i = 0; i < frames_before; i++) {
		res = push(queue, i);
		CU_ASSERT_EQUAL(res, 0);
	}

	/* Hot-update: double the target bitrate while frames are in flight */
	res = venc_get_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);
	dyn.target_bitrate *= 2;
	res = venc_set_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);

	/* Second batch */
	for (unsigned int i = 0; i < frames_after; i++) {
		res = push(queue, frames_before + i);
		CU_ASSERT_EQUAL(res, 0);
	}

	/* Drain: all frames from both batches must come out */
	res = venc_flush(enc, 0);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, (int)(frames_before + frames_after));

	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* API RUNNERS                                                         */
/* ================================================================== */

/*
 * venc_get_dyn_config / venc_set_dyn_config
 *
 * Three behavioral families:
 *   cbr  — H264/H265 CBR: target_bitrate > 0, round-trip works
 * (x264/x265/ffmpeg) cq   — MJPEG CQ: target_bitrate == 0, quality in qp field
 * (turbojpeg) none — encoding has no rate control: returns -ENOSYS (png)
 */
static void run_api_dyn_config_cbr(struct venc_config *config)
{
	int res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_encoder *enc = NULL;
	struct venc_dyn_config dyn = {0};
	uint32_t new_bitrate;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_get_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(dyn.target_bitrate > 0);

	new_bitrate = dyn.target_bitrate / 2;
	dyn.target_bitrate = new_bitrate;
	res = venc_set_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);

	memset(&dyn, 0, sizeof(dyn));
	res = venc_get_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(dyn.target_bitrate, new_bitrate);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}

static void run_api_dyn_config_cq(struct venc_config *config,
				  uint32_t expected_qp)
{
	int res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_encoder *enc = NULL;
	struct venc_dyn_config dyn = {0};

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_get_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(dyn.target_bitrate, 0u);
	CU_ASSERT_EQUAL(dyn.qp, expected_qp);

	/* set is accepted (returns 0) even in CQ mode */
	res = venc_set_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}

static void run_api_dyn_config_unsupported(struct venc_config *config)
{
	int res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_encoder *enc = NULL;
	struct venc_dyn_config dyn = {0};

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_get_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	res = venc_set_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}

/*
 * CQ mode: change qp to a new (nonzero, different) value and read it back.
 *
 * Covers x264 venc_x264.c lines 1576-1580 and x265 venc_x265.c lines 1549-1553:
 * the branch `rate_control == CQ && qp != 0 && qp != current_qp` that updates
 * base->config.h264/h265.qp and reconfigures the encoder.
 */
static void run_api_dyn_config_cq_qp_change(struct venc_config *config,
					    uint32_t initial_qp,
					    uint32_t new_qp)
{
	int res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_encoder *enc = NULL;
	struct venc_dyn_config dyn = {0};

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_get_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(dyn.qp, initial_qp);

	dyn.qp = new_qp;
	res = venc_set_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);

	memset(&dyn, 0, sizeof(dyn));
	res = venc_get_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(dyn.qp, new_qp);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}

/*
 * Decimation change: set a nonzero decimation different from the current value
 * and confirm get_dyn_config reflects the update.
 *
 * Covers x264 venc_x264.c lines 1594-1596, x265 venc_x265.c line 1567, and
 * ffmpeg venc_ffmpeg.c line 2394: `decimation != 0 && decimation != current`
 * update branch.  All fill helpers leave decimation at 0 (memset default), so
 * any nonzero value satisfies both conditions.
 */
static void run_api_dyn_config_decimation_change(struct venc_config *config,
						 uint32_t new_decimation)
{
	int res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_encoder *enc = NULL;
	struct venc_dyn_config dyn = {0};

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_get_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);
	/* venc_new defaults h264/h265 decimation to 1 when the config field is
	 * 0 (venc.c lines 625-626, 728-729); ffmpeg mirrors it via dynconf. The
	 * fill helpers leave decimation unset (memset 0), so after venc_new the
	 * effective value is always 1. */
	CU_ASSERT_EQUAL(dyn.decimation, 1u);

	dyn.decimation = new_decimation;
	res = venc_set_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);

	memset(&dyn, 0, sizeof(dyn));
	res = venc_get_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(dyn.decimation, new_decimation);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/*
 * venc_request_idr
 *
 * Two behavioral families:
 *   h264_h265 — encoder supports IDR requests: returns 0 (x264/x265/ffmpeg)
 *   non_h264  — encoding is not H264/H265: venc.c dispatcher returns -EINVAL
 *               before reaching the backend (turbojpeg/png)
 */
static void run_api_request_idr_h264_h265(struct venc_config *config)
{
	int res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_encoder *enc = NULL;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_request_idr(enc);
	CU_ASSERT_EQUAL(res, 0);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}

static void run_api_request_idr_non_h264(struct venc_config *config)
{
	int res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_encoder *enc = NULL;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* venc.c rejects non-H264/H265 encodings before calling the backend */
	res = venc_request_idr(enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}

/*
 * request_idr with actual frame output verification.
 *
 * The existing run_api_request_idr_h264_h265 only confirms the call returns 0
 * (sets the atomic flag) but never pushes frames, so the encoding loop branch
 * that consumes the flag — x264.c:611-613, x265.c:565-567, ffmpeg.c:1227-1229
 * — is never reached.
 *
 * Two-phase approach:
 *   Phase 1 — encode pre_count frames + drain to get past the initial natural
 *              IDR (first frame of a fresh encoder is always IDR).
 *   Phase 2 — call venc_request_idr then encode post_count frames + drain;
 *              the first encoded frame must be IDR because insert_idr==true.
 * Asserts that at least one output frame from phase 2 has
 * type == VDEF_CODED_FRAME_TYPE_IDR.
 */
static void run_api_request_idr_produces_idr_frame(struct venc_config *config)
{
	int res;
	bool found_idr;
	push_frame_fn push = get_push_fn(config);
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;
	const unsigned int pre_count = 3;
	const unsigned int post_count = 3;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	/* Phase 1: encode pre_count frames, drain, discard output */
	for (unsigned int i = 0; i < pre_count; i++) {
		res = push(queue, i);
		CU_ASSERT_EQUAL(res, 0);
	}
	res = venc_flush(enc, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	mock_venc_cbs_ctx_clear(&ctx);

	/* Phase 2: request IDR, encode post_count frames, drain */
	res = venc_request_idr(enc);
	CU_ASSERT_EQUAL(res, 0);

	for (unsigned int i = 0; i < post_count; i++) {
		res = push(queue, pre_count + i);
		CU_ASSERT_EQUAL(res, 0);
	}
	res = venc_flush(enc, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_TRUE(ctx.frame_count > 0);

	/* At least one output frame must be IDR */
	found_idr = false;
	for (int i = 0; i < ctx.frame_count; i++) {
		struct vdef_coded_frame frame_info;
		if (ctx.captured[i] == NULL)
			continue;
		if (mbuf_coded_video_frame_get_frame_info(ctx.captured[i],
							  &frame_info) == 0 &&
		    frame_info.type == VDEF_CODED_FRAME_TYPE_IDR)
			found_idr = true;
	}
	CU_ASSERT_TRUE(found_idr);
	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/*
 * venc_get_h264_ps
 *
 * Two behavioral families:
 *   h264  — H264 encoder populates base->h264.{sps,pps}: returns SPS+PPS
 *            (x264, ffmpeg)
 *   other — non-H264 encoder: h264.sps stays NULL, returns -EAGAIN
 *            (x265, turbojpeg, png)
 */
static void run_api_get_h264_ps_h264(struct venc_config *config)
{
	int res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_encoder *enc = NULL;
	size_t sps_size = 0, pps_size = 0;
	uint8_t *sps = NULL, *pps = NULL;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Size-query: NULL data pointers, non-NULL size pointers */
	res = venc_get_h264_ps(enc, NULL, &sps_size, NULL, &pps_size);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(sps_size > 0);
	CU_ASSERT_TRUE(pps_size > 0);

	sps = malloc(sps_size);
	pps = malloc(pps_size);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sps);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pps);

	res = venc_get_h264_ps(enc, sps, &sps_size, pps, &pps_size);
	CU_ASSERT_EQUAL(res, 0);

	/* Buffer too small -> -ENOBUFS */
	{
		size_t too_small = 1;
		res = venc_get_h264_ps(enc, sps, &too_small, NULL, NULL);
		CU_ASSERT_EQUAL(res, -ENOBUFS);
	}

	free(sps);
	free(pps);
	venc_destroy(enc);
	pomp_loop_destroy(loop);
}

static void run_api_get_h264_ps_non_h264(struct venc_config *config)
{
	int res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_encoder *enc = NULL;
	size_t sps_size = 0;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* h264.sps is never populated for non-H264 encoders */
	res = venc_get_h264_ps(enc, NULL, &sps_size, NULL, NULL);
	CU_ASSERT_EQUAL(res, -EAGAIN);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* PER-IMPLEM WRAPPERS: frame-output-and-drain                         */
/* ================================================================== */

static void test_frame_output_and_drain_x264(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	run_lifecycle_frame_output_and_drain(&config);
}

static void test_frame_output_and_drain_x265(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	run_lifecycle_frame_output_and_drain(&config);
}

static void test_frame_output_and_drain_turbojpeg(void)
{
	struct venc_config config;
	fill_lifecycle_turbojpeg_config(&config);
	run_lifecycle_frame_output_and_drain(&config);
}

static void test_frame_output_and_drain_png(void)
{
	struct venc_config config;
	fill_lifecycle_png_config(&config);
	run_lifecycle_frame_output_and_drain(&config);
}

static void test_frame_output_and_drain_ffmpeg(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	run_lifecycle_frame_output_and_drain(&config);
}


/* ================================================================== */
/* PER-IMPLEM WRAPPERS: flush-discard                                  */
/* ================================================================== */

static void test_flush_discard_x264(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	run_lifecycle_flush_discard(&config);
}

static void test_flush_discard_x265(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	run_lifecycle_flush_discard(&config);
}

static void test_flush_discard_turbojpeg(void)
{
	struct venc_config config;
	fill_lifecycle_turbojpeg_config(&config);
	run_lifecycle_flush_discard(&config);
}

static void test_flush_discard_png(void)
{
	struct venc_config config;
	fill_lifecycle_png_config(&config);
	run_lifecycle_flush_discard(&config);
}

static void test_flush_discard_ffmpeg(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	run_lifecycle_flush_discard(&config);
}


/* ================================================================== */
/* PER-IMPLEM WRAPPERS: flush-then-reencode                            */
/* ================================================================== */

static void test_flush_then_reencode_x264(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	run_lifecycle_flush_then_reencode(&config);
}

static void test_flush_then_reencode_x265(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	run_lifecycle_flush_then_reencode(&config);
}

static void test_flush_then_reencode_turbojpeg(void)
{
	struct venc_config config;
	fill_lifecycle_turbojpeg_config(&config);
	run_lifecycle_flush_then_reencode(&config);
}

static void test_flush_then_reencode_png(void)
{
	struct venc_config config;
	fill_lifecycle_png_config(&config);
	run_lifecycle_flush_then_reencode(&config);
}

static void test_flush_then_reencode_ffmpeg(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	run_lifecycle_flush_then_reencode(&config);
}


/* ================================================================== */
/* PER-IMPLEM WRAPPERS: dyn-config                                     */
/* ================================================================== */

static void test_dyn_config_x264(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);

	/* NULL-arg guards (dispatcher in venc.c, implem-independent) */
	{
		struct pomp_loop *loop = pomp_loop_new();
		struct mock_venc_cbs_ctx ctx;
		struct venc_cbs cbs = {.frame_output =
					       &mock_venc_frame_output_cb};
		struct venc_encoder *enc = NULL;
		struct venc_dyn_config dyn = {0};
		CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
		mock_venc_cbs_ctx_reset(&ctx);
		CU_ASSERT_EQUAL_FATAL(venc_new(loop, &config, &cbs, &ctx, &enc),
				      0);
		CU_ASSERT_EQUAL(venc_get_dyn_config(enc, NULL), -EINVAL);
		CU_ASSERT_EQUAL(venc_get_dyn_config(NULL, &dyn), -EINVAL);
		CU_ASSERT_EQUAL(venc_set_dyn_config(enc, NULL), -EINVAL);
		CU_ASSERT_EQUAL(venc_set_dyn_config(NULL, &dyn), -EINVAL);
		venc_destroy(enc);
		pomp_loop_destroy(loop);
	}

	run_api_dyn_config_cbr(&config);
}

static void test_dyn_config_x265(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	run_api_dyn_config_cbr(&config);
}

static void test_dyn_config_turbojpeg(void)
{
	struct venc_config config;
	fill_lifecycle_turbojpeg_config(&config);
	/* CQ mode: qp = mjpeg.quality = 80, target_bitrate = 0 */
	run_api_dyn_config_cq(&config, 80);
}

static void test_dyn_config_png(void)
{
	struct venc_config config;
	fill_lifecycle_png_config(&config);
	run_api_dyn_config_unsupported(&config);
}

static void test_dyn_config_ffmpeg(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	run_api_dyn_config_cbr(&config);
}

/*
 * x264 CQ mode: qp update path (venc_x264.c lines 1576-1580).
 *
 * The baseline test_dyn_config_x264 only exercises CBR.  Here the encoder is
 * created in CQ mode with qp=28; changing to qp=32 satisfies
 * `rate_control==CQ && qp!=0 && qp!=current_qp` and triggers
 * x264_encoder_reconfig with the new i_qp_constant.
 */
static void test_dyn_config_cq_qp_change_x264(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	config.h264.rate_control = VENC_RATE_CONTROL_CQ;
	config.h264.qp = 28;
	run_api_dyn_config_cq_qp_change(&config, 28, 32);
}

/*
 * x265 CQ mode: NOT testable.
 *
 * venc_new always defaults h265.level to VENC_H265_LEVEL_4_0 (venc.c:673-676),
 * which causes x265_encoder_open to reject CQ mode with:
 *   "Constant QP is inconsistent with specifying a decoder level,
 *    no bitrate guarantee is possible."
 * Consequently the CQ branch in x265's set_dyn_config (lines 1549-1553) is
 * dead code — it can never be reached without first fixing the x265 backend
 * to skip levelIdc when rate_control == CQ.
 */

/*
 * x264 decimation change (venc_x264.c lines 1594-1596).
 *
 * The fill helper leaves decimation at 0.  Setting 2 satisfies
 * `decimation!=0 && decimation!=current` and updates
 * base->config.h264.decimation.
 */
static void test_dyn_config_decimation_change_x264(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	run_api_dyn_config_decimation_change(&config, 2);
}

/*
 * x265 decimation change (venc_x265.c line 1567).
 */
static void test_dyn_config_decimation_change_x265(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	run_api_dyn_config_decimation_change(&config, 2);
}

/*
 * ffmpeg decimation change (venc_ffmpeg.c line 2394).
 */
static void test_dyn_config_decimation_change_ffmpeg(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	run_api_dyn_config_decimation_change(&config, 2);
}

/*
 * turbojpeg: qp (JPEG quality) change (venc_turbojpeg.c line 968).
 *
 * The existing test_dyn_config_turbojpeg reads qp=80 then sets the same
 * value back — the `config->qp != base->config.mjpeg.quality` condition is
 * false, so quality is never updated.  Here we explicitly set a different
 * quality (70) and verify get_dyn_config reflects the change.
 */
static void test_dyn_config_turbojpeg_qp_change(void)
{
	int res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_config config;
	struct venc_encoder *enc = NULL;
	struct venc_dyn_config dyn = {0};

	fill_lifecycle_turbojpeg_config(&config); /* quality=80 */
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_get_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(dyn.qp, 80u);

	dyn.qp = 70;
	res = venc_set_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);

	memset(&dyn, 0, sizeof(dyn));
	res = venc_get_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(dyn.qp, 70u);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}

/*
 * turbojpeg: target_bitrate update with VBR rate control (lines 962-966).
 *
 * turbojpeg's create() does not validate rate_control, so VBR is accepted.
 * With rate_control != CQ, `target_bitrate != 0 && bitrate changed` triggers
 * the update branch.  The baseline turbojpeg test always uses CQ, so this
 * branch was never reached.
 */
static void test_dyn_config_turbojpeg_bitrate_vbr(void)
{
	int res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_config config;
	struct venc_encoder *enc = NULL;
	struct venc_dyn_config dyn = {0};
	const uint32_t initial_bitrate = 1000000;
	const uint32_t new_bitrate = 500000;

	fill_lifecycle_turbojpeg_config(&config);
	config.mjpeg.rate_control = VENC_RATE_CONTROL_VBR;
	/* venc_new requires max_bitrate > 0 for CBR/VBR (venc.c lines 747-755)
	 */
	config.mjpeg.max_bitrate = initial_bitrate;
	config.mjpeg.target_bitrate = initial_bitrate;
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_get_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(dyn.target_bitrate, initial_bitrate);

	dyn.target_bitrate = new_bitrate;
	res = venc_set_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);

	memset(&dyn, 0, sizeof(dyn));
	res = venc_get_dyn_config(enc, &dyn);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(dyn.target_bitrate, new_bitrate);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* PER-IMPLEM WRAPPERS: decimation (frame-skip encoding loop)          */
/* ================================================================== */

/*
 * Push DECIMATION * EXPECTED_OUT frames with decimation=DECIMATION configured
 * at creation.  The encoding loop skips frames where input_frame_cnt % D != 0
 * (x264.c:536-542, x265.c:490-497, ffmpeg.c:1462-1467), so exactly
 * EXPECTED_OUT frames are encoded and returned after a drain flush.
 */
#define DECIMATION_FACTOR 2u
#define DECIMATION_EXPECTED_OUT 3u

static void run_lifecycle_decimation(struct venc_config *config)
{
	int res;
	push_frame_fn push = get_push_fn(config);
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;
	const unsigned int total_pushed =
		DECIMATION_FACTOR * DECIMATION_EXPECTED_OUT;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	for (unsigned int i = 0; i < total_pushed; i++) {
		res = push(queue, i);
		CU_ASSERT_EQUAL(res, 0);
	}

	res = venc_flush(enc, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, (int)DECIMATION_EXPECTED_OUT);

	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}

static void test_decimation_x264(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	config.h264.decimation = DECIMATION_FACTOR;
	run_lifecycle_decimation(&config);
}

static void test_decimation_x265(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	config.h265.decimation = DECIMATION_FACTOR;
	run_lifecycle_decimation(&config);
}

static void test_decimation_ffmpeg(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	config.h264.decimation = DECIMATION_FACTOR;
	run_lifecycle_decimation(&config);
}


/* ================================================================== */
/* PER-IMPLEM WRAPPERS: request-idr                                    */
/* ================================================================== */

static void test_request_idr_x264(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	run_api_request_idr_h264_h265(&config);
}

static void test_request_idr_x265(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	run_api_request_idr_h264_h265(&config);
}

static void test_request_idr_turbojpeg(void)
{
	struct venc_config config;
	fill_lifecycle_turbojpeg_config(&config);
	run_api_request_idr_non_h264(&config);
}

static void test_request_idr_png(void)
{
	struct venc_config config;
	fill_lifecycle_png_config(&config);
	run_api_request_idr_non_h264(&config);
}

static void test_request_idr_ffmpeg(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	run_api_request_idr_h264_h265(&config);
}

static void test_request_idr_x264_idr_produced(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	run_api_request_idr_produces_idr_frame(&config);
}

static void test_request_idr_x265_idr_produced(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	run_api_request_idr_produces_idr_frame(&config);
}

static void test_request_idr_ffmpeg_idr_produced(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	run_api_request_idr_produces_idr_frame(&config);
}


/* ================================================================== */
/* PER-IMPLEM WRAPPERS: get-input-buffer-pool                         */
/* ================================================================== */

/*
 * All in-tree backends (x264, x265, turbojpeg, png, ffmpeg) leave the input
 * buffer pool unallocated and return NULL from get_input_buffer_pool()
 * (documented "use the application's pool" behaviour).  Only the out-of-tree
 * HISI/QCOM backends in libvideo-encode-parrot are expected to return a
 * non-NULL pool.  The tests here confirm the contract for every backend
 * compiled into this product, plus the NULL-self guard in venc.c.
 */
static void run_get_input_buffer_pool(struct venc_config *config)
{
	int res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_encoder *enc = NULL;
	struct mbuf_pool *pool;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	pool = venc_get_input_buffer_pool(enc);
	CU_ASSERT_PTR_NULL(pool);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}

static void test_get_input_buffer_pool_null_self(void)
{
	struct mbuf_pool *pool = venc_get_input_buffer_pool(NULL);
	CU_ASSERT_PTR_NULL(pool);
}

static void test_get_input_buffer_pool_x264(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	run_get_input_buffer_pool(&config);
}

static void test_get_input_buffer_pool_x265(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	run_get_input_buffer_pool(&config);
}

static void test_get_input_buffer_pool_turbojpeg(void)
{
	struct venc_config config;
	fill_lifecycle_turbojpeg_config(&config);
	run_get_input_buffer_pool(&config);
}

static void test_get_input_buffer_pool_png(void)
{
	struct venc_config config;
	fill_lifecycle_png_config(&config);
	run_get_input_buffer_pool(&config);
}

static void test_get_input_buffer_pool_ffmpeg(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	run_get_input_buffer_pool(&config);
}


/* ================================================================== */
/* PER-IMPLEM WRAPPERS: get-h264-ps                                    */
/* ================================================================== */

static void test_get_h264_ps_x264(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	run_api_get_h264_ps_h264(&config);
}

static void test_get_h264_ps_x265(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	/* h264.{sps,pps} and h265.{vps,sps} share memory in the union inside
	 * struct venc_encoder. After x265 initialization, h265.{vps,sps} are
	 * non-NULL, so venc_get_h264_ps returns 0 and yields the H265 VPS+SPS
	 * bytes (not H264 PS). Callers should only call venc_get_h264_ps for
	 * H264 encoders; this test pins the actual behavior. */
	run_api_get_h264_ps_h264(&config);
}

static void test_get_h264_ps_turbojpeg(void)
{
	struct venc_config config;
	fill_lifecycle_turbojpeg_config(&config);
	run_api_get_h264_ps_non_h264(&config);
}

static void test_get_h264_ps_png(void)
{
	struct venc_config config;
	fill_lifecycle_png_config(&config);
	run_api_get_h264_ps_non_h264(&config);
}

static void test_get_h264_ps_ffmpeg(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	run_api_get_h264_ps_h264(&config);
}


/* ================================================================== */
/* x264 input-format variants (NV12, GRAY)                            */
/* ================================================================== */

/*
 * x264 input-format variants: NV12, NV21, GRAY.
 *
 * Each exercises a distinct branch in check_input_queue() (venc_x264.c):
 *   NV12 → X264_CSP_NV12, 2 planes (lines 566-568)
 *   NV21 → X264_CSP_NV21, 2 planes, distinct stride setup (lines ~1400-1406)
 *   GRAY → X264_CSP_I400, 1 real plane + dummy UV plane (lines 560-565)
 * All configs are identical to the baseline x264 config; output stays H264
 * BYTE_STREAM.
 */
static void test_frame_output_and_drain_x264_nv12(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	config.input.format = vdef_nv12;
	run_lifecycle_frame_output_and_drain(&config);
}


static void test_frame_output_and_drain_x264_nv21(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	config.input.format = vdef_nv21;
	run_lifecycle_frame_output_and_drain(&config);
}


static void test_frame_output_and_drain_x264_gray(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	config.input.format = vdef_gray;
	run_lifecycle_frame_output_and_drain(&config);
}


/*
 * x265 GRAY input format — dummy UV plane path.
 *
 * When vdef_gray is configured, venc_x265_create() (lines 1390-1400) computes
 * dummy_uv_plane_stride = width/2, dummy_uv_plane_len = (width/2)*(height/2),
 * malloc's and memset(0x80)'s the dummy UV plane, then injects it alongside
 * every gray Y plane in the encoding loop.  This exercises the same allocation
 * and injection path that the x264 gray test covers, but in the x265 backend.
 * The test is skipped at runtime if x265 is not available.
 */
static void test_frame_output_and_drain_x265_gray(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	config.input.format = vdef_gray;
	run_lifecycle_frame_output_and_drain(&config);
}


/*
 * PNG RGB input format.
 *
 * PNG backend sets PNG_COLOR_TYPE_RGB + planes=1 (venc_png.c lines 886-888)
 * when the input is vdef_rgb. The GRAY path is already covered by the baseline
 * PNG lifecycle test. Config is identical to the PNG baseline with the format
 * overridden; bit_depth stays 8.
 */
static void test_frame_output_and_drain_png_rgb(void)
{
	struct venc_config config;
	fill_lifecycle_png_config(&config);
	config.input.format = vdef_rgb;
	run_lifecycle_frame_output_and_drain(&config);
}


/*
 * turbojpeg input-format variants: NV12, NV21.
 *
 * The turbojpeg backend de-interleaves the semi-planar UV plane into separate
 * U and V buffers before passing them to libjpeg-turbo (venc_turbojpeg.c
 * lines 316-344).  NV21 additionally swaps up_offset/vp_offset (line 335-337).
 * Both branches are uncovered by the default I420 lifecycle tests.
 * Config is identical to the baseline turbojpeg config with format overridden.
 */
static void test_frame_output_and_drain_turbojpeg_nv12(void)
{
	struct venc_config config;
	fill_lifecycle_turbojpeg_config(&config);
	config.input.format = vdef_nv12;
	run_lifecycle_frame_output_and_drain(&config);
}


static void test_frame_output_and_drain_turbojpeg_nv21(void)
{
	struct venc_config config;
	fill_lifecycle_turbojpeg_config(&config);
	config.input.format = vdef_nv21;
	run_lifecycle_frame_output_and_drain(&config);
}


static void test_frame_output_and_drain_turbojpeg_gray(void)
{
	struct venc_config config;
	fill_lifecycle_turbojpeg_config(&config);
	config.input.format = vdef_gray;
	run_lifecycle_frame_output_and_drain(&config);
}


/*
 * ffmpeg input-format variants: GRAY.
 *
 * ffmpeg does not support grayscale input natively. When vdef_gray is
 * configured, venc_ffmpeg_create() (lines 2189-2217) overrides the internal
 * format with supported_formats[1] (the first non-gray native format, typically
 * I420 or NV12 depending on the codec) and allocates a zeroed dummy UV plane
 * that is injected alongside every gray Y plane in check_input_queue()
 * (line 1177-1178: plane_count = 1).
 *
 * The test is skipped if ffmpeg is unavailable (same as all other ffmpeg
 * tests).
 */
static void test_frame_output_and_drain_ffmpeg_gray(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	config.input.format = vdef_gray;
	run_lifecycle_frame_output_and_drain(&config);
}


/* ================================================================== */
/* get-h265-ps (x265 only)                                            */
/* ================================================================== */

/*
 * venc_get_h265_ps returns VPS + SPS + PPS after x265 initialization.
 * For non-H265 encoders, h265.pps is NULL in the union (the h264 struct
 * doesn't have a pps_size beyond the h264.pps_size field), so they return
 * -EAGAIN. Only x265 is tested here since it is the only H265 backend in
 * the Linux product config.
 */
static void test_get_h265_ps_x265(void)
{
	int res;
	struct venc_config config;
	struct pomp_loop *loop;
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_encoder *enc = NULL;
	size_t vps_size = 0, sps_size = 0, pps_size = 0;
	uint8_t *vps = NULL, *sps = NULL, *pps = NULL;

	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	fill_lifecycle_x265_config(&config);

	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* NULL-arg guards (EINVAL) */
	res = venc_get_h265_ps(
		NULL, NULL, &vps_size, NULL, &sps_size, NULL, &pps_size);
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* Size-query: NULL data, non-NULL size pointers */
	res = venc_get_h265_ps(
		enc, NULL, &vps_size, NULL, &sps_size, NULL, &pps_size);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(vps_size > 0);
	CU_ASSERT_TRUE(sps_size > 0);
	CU_ASSERT_TRUE(pps_size > 0);

	vps = malloc(vps_size);
	sps = malloc(sps_size);
	pps = malloc(pps_size);
	CU_ASSERT_PTR_NOT_NULL_FATAL(vps);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sps);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pps);

	/* Full data retrieval */
	res = venc_get_h265_ps(
		enc, vps, &vps_size, sps, &sps_size, pps, &pps_size);
	CU_ASSERT_EQUAL(res, 0);

	/* Buffer too small -> -ENOBUFS */
	{
		size_t too_small = 1;
		res = venc_get_h265_ps(
			enc, vps, &too_small, NULL, NULL, NULL, NULL);
		CU_ASSERT_EQUAL(res, -ENOBUFS);
	}

	free(vps);
	free(sps);
	free(pps);
	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* ffmpeg H.265 (hevc_nvenc) -- only run when the machine actually has  */
/* an NVIDIA GPU/driver exposing hevc_nvenc to libavcodec               */
/* ================================================================== */

static void test_frame_output_and_drain_ffmpeg_h265(void)
{
	struct venc_config config;
	if (!encoding_supported(VENC_ENCODER_IMPLEM_FFMPEG, VDEF_ENCODING_H265))
		return;
	fill_lifecycle_ffmpeg_h265_config(&config);
	run_lifecycle_frame_output_and_drain(&config);
}


/*
 * Regression test for a real bug: venc_ffmpeg.c's create() validates
 * output.preferred_format with the exact same block as venc_x264.c/
 * venc_x265.c (accepts UNKNOWN/BYTE_STREAM/AVCC), but unlike those two
 * backends never applied the corresponding UNKNOWN -> BYTE_STREAM default
 * afterwards -- base->config.output.preferred_format was used as-is
 * everywhere else. Left at UNKNOWN (its zero-value default, e.g. an
 * all-zero-initialized venc_config), every output frame's format.data_format
 * stayed UNKNOWN, which vdef_is_coded_format_valid() rejects for H264/H265,
 * so mbuf_coded_video_frame_new() failed on every single frame -- silently,
 * from the caller's point of view: no error surfaced beyond "no frame ever
 * comes out". Pins down the fix instead of relying on every ffmpeg test in
 * this file setting preferred_format explicitly.
 */
static void test_output_format_defaults_to_byte_stream_ffmpeg(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	config.output.preferred_format = VDEF_CODED_DATA_FORMAT_UNKNOWN;
	run_lifecycle_frame_output_and_drain(&config);
}

static void test_output_format_defaults_to_byte_stream_ffmpeg_h265(void)
{
	struct venc_config config;
	if (!encoding_supported(VENC_ENCODER_IMPLEM_FFMPEG, VDEF_ENCODING_H265))
		return;
	fill_lifecycle_ffmpeg_h265_config(&config);
	config.output.preferred_format = VDEF_CODED_DATA_FORMAT_UNKNOWN;
	run_lifecycle_frame_output_and_drain(&config);
}


/*
 * Mirrors test_get_h265_ps_x265: venc_ffmpeg.c calls the same shared
 * venc_h265_patch_ps() helper (core/src/venc_h265.c) right after opening the
 * encoder, so self->h265.{vps,sps,pps} are populated the same way regardless
 * of backend.
 */
static void test_get_h265_ps_ffmpeg(void)
{
	int res;
	struct venc_config config;
	struct pomp_loop *loop;
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_encoder *enc = NULL;
	size_t vps_size = 0, sps_size = 0, pps_size = 0;
	uint8_t *vps = NULL, *sps = NULL, *pps = NULL;

	if (!encoding_supported(VENC_ENCODER_IMPLEM_FFMPEG, VDEF_ENCODING_H265))
		return;

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	fill_lifecycle_ffmpeg_h265_config(&config);

	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_get_h265_ps(
		enc, NULL, &vps_size, NULL, &sps_size, NULL, &pps_size);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(vps_size > 0);
	CU_ASSERT_TRUE(sps_size > 0);
	CU_ASSERT_TRUE(pps_size > 0);

	vps = malloc(vps_size);
	sps = malloc(sps_size);
	pps = malloc(pps_size);
	CU_ASSERT_PTR_NOT_NULL_FATAL(vps);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sps);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pps);

	res = venc_get_h265_ps(
		enc, vps, &vps_size, sps, &sps_size, pps, &pps_size);
	CU_ASSERT_EQUAL(res, 0);

	free(vps);
	free(sps);
	free(pps);
	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/*
 * A non-standard SAR (not in the H.265 spec Table E-1) makes
 * h265_sar_to_aspect_ratio_idc() return H265_ASPECT_RATIO_EXTENDED_SAR (255),
 * triggering the sar_width/sar_height copy branch in set_h265_vui().
 *
 * The complementary uncovered branch — framerate null + insert_time_code_sei
 * set — is not reachable via the public API: x265 rejects fpsNum=0 before the
 * encoder reaches venc_h265_patch_ps(), so it is documented here as a
 * known untestable defensive-warning path.
 */
static void test_get_h265_ps_extended_sar_x265(void)
{
	int res;
	struct venc_config config;
	struct pomp_loop *loop;
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_encoder *enc = NULL;
	size_t vps_size = 0, sps_size = 0, pps_size = 0;
	uint8_t *vps = NULL, *sps = NULL, *pps = NULL;
	struct h265_sps parsed_sps;

	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	fill_lifecycle_x265_config(&config);
	/* 3:7 is not in the H.265 standard SAR table → aspect_ratio_idc = 255
	 */
	config.input.info.sar.width = 3;
	config.input.info.sar.height = 7;

	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Size query */
	res = venc_get_h265_ps(
		enc, NULL, &vps_size, NULL, &sps_size, NULL, &pps_size);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE(sps_size > 0);

	vps = malloc(vps_size);
	sps = malloc(sps_size);
	pps = malloc(pps_size);
	CU_ASSERT_PTR_NOT_NULL_FATAL(vps);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sps);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pps);

	res = venc_get_h265_ps(
		enc, vps, &vps_size, sps, &sps_size, pps, &pps_size);
	CU_ASSERT_EQUAL(res, 0);

	/* Parse the SPS: verify Extended_SAR fields */
	memset(&parsed_sps, 0, sizeof(parsed_sps));
	res = h265_parse_sps(sps, sps_size, &parsed_sps);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL((int)parsed_sps.vui.aspect_ratio_idc,
			H265_ASPECT_RATIO_EXTENDED_SAR);
	CU_ASSERT_EQUAL(parsed_sps.vui.sar_width, 3u);
	CU_ASSERT_EQUAL(parsed_sps.vui.sar_height, 7u);

	free(vps);
	free(sps);
	free(pps);
	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* CODEC-SPECIFIC TESTS (x264 H264 streaming SEI)                     */
/* ================================================================== */

/*
 * venc_h264_generate_nalus() optional-NALU branches via a real x264 encode.
 * insert_aud fires on every frame; streaming_user_data_sei_version fires on
 * IDR/I/P_IR_START (reliably the first frame of any fresh encoder).
 */
static void run_h264_streaming_sei(int streaming_version)
{
	int res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct venc_config config;
	struct mbuf_raw_video_frame_queue *queue;
	const unsigned int frame_count = 3;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	fill_lifecycle_x264_config(&config);
	config.h264.insert_aud = 1;
	config.h264.insert_ps = 1;
	config.h264.streaming_user_data_sei_version = streaming_version;

	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	for (unsigned int i = 0; i < frame_count; i++) {
		res = push_frame(queue, i);
		CU_ASSERT_EQUAL(res, 0);
	}

	res = venc_flush(enc, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, (int)frame_count);

	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


static void test_h264_streaming_sei_v2(void)
{
	run_h264_streaming_sei(2);
}


static void test_h264_streaming_sei_v4(void)
{
	run_h264_streaming_sei(4);
}


/*
 * H264 picture-timing SEI.
 *
 * Condition in venc_h264_generate_nalus (core/src/venc_h264.c):
 *   if (insert_pic_timing_sei && capture_timestamp != 0)
 *
 * push_frame sets capture_timestamp = timestamp = index * 1000000/30.
 * Frame 0 has timestamp 0 (skipped); frames 1 and 2 have non-zero timestamps
 * and trigger venc_h264_sei_add_picture_timing.
 * insert_pic_timing_sei also sets b_pic_struct in x264 params.
 */
static void test_h264_pic_timing_sei(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	config.h264.insert_pic_timing_sei = 1;
	run_lifecycle_frame_output_and_drain(&config);
}


/*
 * H264 serialize_user_data SEI.
 *
 * With serialize_user_data = 1, venc_h264_generate_nalus reads
 * MBUF_ANCILLARY_KEY_USERDATA_SEI from the output coded frame and injects it
 * as a User Data Unregistered SEI. The x264 backend transfers ancillary data
 * from the input raw frame to the output coded frame automatically, so we
 * attach it on every pushed frame. Note: if the key is absent the H264 path
 * returns an error (goto out), so every frame MUST carry the data.
 */
static void test_h264_serialize_user_data(void)
{
	int res;
	struct venc_config config;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;
	const unsigned int frame_count = 3;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	fill_lifecycle_x264_config(&config);
	config.h264.serialize_user_data = 1;

	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	for (unsigned int i = 0; i < frame_count; i++) {
		res = push_frame_with_userdata_sei(queue, i);
		CU_ASSERT_EQUAL(res, 0);
	}

	res = venc_flush(enc, 0);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, (int)frame_count);

	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* AVCC / HVCC output format                                          */
/* ================================================================== */

/*
 * VDEF_CODED_DATA_FORMAT_AVCC == VDEF_CODED_DATA_FORMAT_HVCC (same enum
 * value per vdefs.h:1054).  The fill helpers default to BYTE_STREAM, so
 * none of the baseline lifecycle tests exercise the AVCC/HVCC path.
 *
 * x264 (venc_x264.c line 378): replaces start code with 4-byte BE NALU
 * length prefix per NALU.
 * x265 (venc_h265.c lines 424/541/595/649): same per-NALU-type branches.
 * ffmpeg (venc_ffmpeg.c line 991): patches start codes in-place for H.264.
 */
static void test_h264_avcc_output_x264(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	config.output.preferred_format = VDEF_CODED_DATA_FORMAT_AVCC;
	run_lifecycle_frame_output_and_drain(&config);
}

static void test_h265_avcc_output(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	config.output.preferred_format = VDEF_CODED_DATA_FORMAT_AVCC;
	run_lifecycle_frame_output_and_drain(&config);
}

static void test_h264_avcc_output_ffmpeg(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	config.output.preferred_format = VDEF_CODED_DATA_FORMAT_AVCC;
	run_lifecycle_frame_output_and_drain(&config);
}


/*
 * H265 serialize_user_data SEI.
 *
 * With serialize_user_data = 1, venc_h265_generate_nalus reads
 * MBUF_ANCILLARY_KEY_USERDATA_SEI from the output coded frame. Unlike the
 * H264 path (which hard-fails on missing data), H265 silently skips if the
 * key is absent. We still attach data on every frame to actually exercise the
 * SEI-injection branch at line 1152.
 */
static void test_h265_serialize_user_data(void)
{
	int res;
	struct venc_config config;
	struct pomp_loop *loop;
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;
	const unsigned int frame_count = 3;

	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	fill_lifecycle_x265_config(&config);
	config.h265.serialize_user_data = 1;

	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	for (unsigned int i = 0; i < frame_count; i++) {
		res = push_frame_with_userdata_sei(queue, i);
		CU_ASSERT_EQUAL(res, 0);
	}

	res = venc_flush(enc, 0);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, (int)frame_count);

	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/*
 * insert_mdcv_sei — venc_h265_generate_nalus() lines 1088-1125.
 *
 * The MDCV SEI is inserted on IDR/I/P_IR_START frames when insert_mdcv_sei is
 * set and MDCV data is valid.  The outer condition has two OR branches:
 *   (A) display_primaries != VDEF_COLOR_PRIMARIES_UNKNOWN  (known enum)
 *   (B) display_primaries == UNKNOWN + all 8 chromaticity coords non-zero
 *
 * run_h265_mdcv_sei is parameterised so we can exercise both branches from
 * separate test functions with a single helper.
 */
static void run_h265_mdcv_sei(enum vdef_color_primaries primaries,
			      const struct vdef_color_primaries_value *val)
{
	int res;
	struct venc_config config;
	struct pomp_loop *loop;
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;

	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	fill_lifecycle_x265_config(&config);
	config.h265.insert_mdcv_sei = 1;
	config.input.info.mdcv.display_primaries = primaries;
	if (val)
		config.input.info.mdcv.display_primaries_val = *val;
	/* Non-zero luminance is always required */
	config.input.info.mdcv.max_display_mastering_luminance = 1000.0f;
	config.input.info.mdcv.min_display_mastering_luminance = 0.01f;

	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	/* One frame is enough — the first output is always IDR */
	res = push_frame(queue, 0);
	CU_ASSERT_EQUAL(res, 0);

	res = venc_flush(enc, 0);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, 1);

	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}

/* Branch A: display_primaries != UNKNOWN → first OR arm */
static void test_h265_mdcv_sei_known_primaries_x265(void)
{
	run_h265_mdcv_sei(VDEF_COLOR_PRIMARIES_BT709, NULL);
}

/* Branch B: UNKNOWN primaries + all 8 chromaticity coordinates non-zero
 * → exercises the display_primaries_val chain (venc_h265.c lines 1093-1114) */
static void test_h265_mdcv_sei_custom_primaries_x265(void)
{
	/* BT.709 values spelled out explicitly so display_primaries stays
	 * UNKNOWN and the custom-value branch is taken */
	static const struct vdef_color_primaries_value bt709 = {
		/* green, blue, red */
		.color_primaries =
			{
				{.x = 0.300f, .y = 0.600f},
				{.x = 0.150f, .y = 0.060f},
				{.x = 0.640f, .y = 0.330f},
			},
		/* D65 white point */
		.white_point = {.x = 0.3127f, .y = 0.3290f},
	};
	run_h265_mdcv_sei(VDEF_COLOR_PRIMARIES_UNKNOWN, &bt709);
}


/*
 * H264 insert_recovery_point_sei — lines 1047-1054 of venc_h264.c.
 *
 * The branch fires when frame type == VDEF_CODED_FRAME_TYPE_P_IR_START.
 * The x264 backend sets P_IR_START when it sees a recovery_point SEI in the
 * x264 bitstream output AND intra_refresh != NONE (venc_x264.c line 704-707).
 * With b_intra_refresh=1 and a short keyint, x264 emits the SEI at the start
 * of each refresh cycle.  gop_length_sec=3/30s → keyint=3; encoding 8 frames
 * spans at least 2 cycles, so frames 3 and 6 are typed P_IR_START.
 *
 * Note: the equivalent H265 path (venc_h265.c line 1068-1075) is currently
 * unreachable: the x265 backend detects recovery_point SEI (its callback sets
 * self->recovery_point=true) but — unlike x264 — never uses that flag to set
 * out_info.type = P_IR_START.  Fixing that omission in the x265 backend would
 * make the H265 branch coverable via the same pattern.
 */
static void test_h264_insert_recovery_point_sei(void)
{
	int res;
	struct venc_config config;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;
	const unsigned int frame_count = 8;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	fill_lifecycle_x264_config(&config);
	config.h264.intra_refresh = VENC_INTRA_REFRESH_VERTICAL_SCAN;
	config.h264.intra_refresh_length = 3;
	/* keyint = round(3/30 * 30/1) = 3: P_IR_START appears at frames 3, 6, …
	 */
	config.h264.gop_length_sec = 3.0f / 30.0f;
	config.h264.insert_recovery_point_sei = 1;

	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	for (unsigned int i = 0; i < frame_count; i++) {
		res = push_frame(queue, i);
		CU_ASSERT_EQUAL(res, 0);
	}

	res = venc_flush(enc, 0);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, (int)frame_count);

	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* IMPLEM_AUTO                                                         */
/* ================================================================== */

/*
 * venc_new() with VENC_ENCODER_IMPLEM_AUTO resolves to the first entry of
 * venc.c's supported_implems[] via venc_get_implem() — ffmpeg is listed
 * ahead of x264 there ("Put preferred implementation first"), so on any
 * product build with ffmpeg enabled AUTO resolves to ffmpeg, not x264. The
 * test verifies that the resolved encoder actually produces output — i.e.
 * that the AUTO dispatch path in venc.c:507 + ops->create() complete
 * successfully. Uses the ffmpeg fixture (FFMPEG_LIFECYCLE_WIDTH/HEIGHT) since
 * ffmpeg may itself resolve to the h264_nvenc HW backend, which enforces its
 * own minimum encode resolution above LIFECYCLE_WIDTH/HEIGHT.
 */
static void test_frame_output_and_drain_implem_auto_h264(void)
{
	struct venc_config config;
	/* AUTO resolves to supported_implems[0] (typically FFMPEG on Linux).
	 * If that implem cannot do H264 at runtime (e.g. no GPU for nvenc),
	 * venc_new() fails. Guard the same way ffmpeg-specific tests do. */
	if (!encoding_supported(VENC_ENCODER_IMPLEM_AUTO, VDEF_ENCODING_H264))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	config.implem = VENC_ENCODER_IMPLEM_AUTO;
	run_lifecycle_frame_output_and_drain(&config);
}


/* ================================================================== */
/* flush-discard-then-reencode per implem                             */
/* ================================================================== */

static void test_flush_discard_then_reencode_x264(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	run_lifecycle_flush_discard_then_reencode(&config);
}

static void test_flush_discard_then_reencode_x265(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	run_lifecycle_flush_discard_then_reencode(&config);
}

static void test_flush_discard_then_reencode_turbojpeg(void)
{
	struct venc_config config;
	fill_lifecycle_turbojpeg_config(&config);
	run_lifecycle_flush_discard_then_reencode(&config);
}

static void test_flush_discard_then_reencode_png(void)
{
	struct venc_config config;
	fill_lifecycle_png_config(&config);
	run_lifecycle_flush_discard_then_reencode(&config);
}

static void test_flush_discard_then_reencode_ffmpeg(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	run_lifecycle_flush_discard_then_reencode(&config);
}


/* ================================================================== */
/* dyn-config mid-encode per implem                                   */
/* ================================================================== */

static void test_dyn_config_mid_encode_x264(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	run_lifecycle_dyn_config_mid_encode(&config);
}

static void test_dyn_config_mid_encode_x265(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	run_lifecycle_dyn_config_mid_encode(&config);
}

static void test_dyn_config_mid_encode_ffmpeg(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	run_lifecycle_dyn_config_mid_encode(&config);
}


/* ================================================================== */
/* Two simultaneous encoders on the same loop                         */
/* ================================================================== */

/*
 * Two independent x264 encoders share the same pomp_loop.  Each gets its own
 * input queue, callback context, and worker thread.  Push 3 frames to each,
 * drain both (sequentially, one flush each), and verify 3 frames come out of
 * each encoder independently.  This checks that no global state (e.g. a
 * static singleton) is accidentally shared between instances.
 */
static void test_two_simultaneous_encoders(void)
{
	int res;
	struct venc_config config;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx1, ctx2;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc1 = NULL, *enc2 = NULL;
	struct mbuf_raw_video_frame_queue *q1, *q2;
	const unsigned int frame_count = 3;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx1);
	mock_venc_cbs_ctx_reset(&ctx2);

	fill_lifecycle_x264_config(&config);

	res = venc_new(loop, &config, &cbs, &ctx1, &enc1);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = venc_new(loop, &config, &cbs, &ctx2, &enc2);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	q1 = venc_get_input_buffer_queue(enc1);
	q2 = venc_get_input_buffer_queue(enc2);
	CU_ASSERT_PTR_NOT_NULL_FATAL(q1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(q2);

	for (unsigned int i = 0; i < frame_count; i++) {
		res = push_frame(q1, i);
		CU_ASSERT_EQUAL(res, 0);
		res = push_frame(q2, i);
		CU_ASSERT_EQUAL(res, 0);
	}

	/* Drain enc1 */
	res = venc_flush(enc1, 0);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx1.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx1.flush_done);
	CU_ASSERT_EQUAL(ctx1.frame_count, (int)frame_count);

	/* Drain enc2 */
	res = venc_flush(enc2, 0);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx2.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx2.flush_done);
	CU_ASSERT_EQUAL(ctx2.frame_count, (int)frame_count);

	mock_venc_cbs_ctx_clear(&ctx1);
	mock_venc_cbs_ctx_clear(&ctx2);

	res = venc_stop(enc1);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx1.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx1.stop_done);
	venc_destroy(enc1);

	res = venc_stop(enc2);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx2.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx2.stop_done);
	venc_destroy(enc2);

	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* DELAYED STOP (frame_release → stop_done path)                      */
/* ================================================================== */

/*
 * venc_stop() when unreleased output frames exist.
 *
 * On VENC_MSG_STOP, the worker thread checks venc_count_unreleased_frames().
 * If non-zero, it sets stopping=true and returns.  The stop_done callback is
 * only scheduled later in frame_release() (the mbuf coded-frame pre_release
 * hook), once unreleased_count reaches 0.
 *
 * This path is never entered in the standard lifecycle tests because those all
 * call mock_venc_cbs_ctx_clear() before venc_stop(), making unreleased==0 at
 * stop time and taking the immediate VENC_MSG_STOP path instead.  Here we
 * deliberately hold the output frame refs across the stop call.
 */
static void run_stop_delayed_until_frames_released(struct venc_config *config)
{
	int res;
	push_frame_fn push = get_push_fn(config);
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;
	const unsigned int frame_count = 3;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	res = venc_new(loop, config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	for (unsigned int i = 0; i < frame_count; i++) {
		res = push(queue, i);
		CU_ASSERT_EQUAL(res, 0);
	}

	/* Drain: all frames encoded, test holds refs via ctx.captured[] */
	res = venc_flush(enc, 0);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, (int)frame_count);

	/* Do NOT release captured frames — stop must be deferred */
	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);

	/* Pump briefly: worker processes VENC_MSG_STOP, sets stopping=true;
	 * stop_done must NOT fire yet (unreleased_count > 0) */
	for (int i = 0; i < 3 && !ctx.stop_done; i++)
		pomp_loop_wait_and_process(loop, PUMP_TIMEOUT_MS);
	CU_ASSERT_FALSE(ctx.stop_done);

	/* Release all held output frames: frame_release() fires for each;
	 * on the last one (unreleased==0 && stopping==true) it schedules
	 * call_stop_done on the loop */
	mock_venc_cbs_ctx_clear(&ctx);

	/* Pump: the pending idle callback fires → stop_done */
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


static void test_stop_delayed_x264(void)
{
	struct venc_config config;
	fill_lifecycle_x264_config(&config);
	run_stop_delayed_until_frames_released(&config);
}

static void test_stop_delayed_x265(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;
	fill_lifecycle_x265_config(&config);
	run_stop_delayed_until_frames_released(&config);
}

static void test_stop_delayed_turbojpeg(void)
{
	struct venc_config config;
	fill_lifecycle_turbojpeg_config(&config);
	run_stop_delayed_until_frames_released(&config);
}

static void test_stop_delayed_png(void)
{
	struct venc_config config;
	fill_lifecycle_png_config(&config);
	run_stop_delayed_until_frames_released(&config);
}

static void test_stop_delayed_ffmpeg(void)
{
	struct venc_config config;
	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;
	fill_lifecycle_ffmpeg_config(&config);
	run_stop_delayed_until_frames_released(&config);
}


/* ================================================================== */
/* INPUT FILTER REJECTION (venc_default_input_filter_internal)        */
/* ================================================================== */

/*
 * The queue filter registered at encoder creation calls
 * venc_default_input_filter_internal (venc_core.c:284) at push time.  On
 * rejection the filter returns false and mbuf_raw_video_frame_queue_push
 * returns -EPROTO (mbuf_raw_video_frame.c:939) — synchronously, with no
 * loop pump needed.  All three branches are exercised below using x264
 * (always available); since the filter is shared code in venc_core.c,
 * testing any one backend covers all backends.
 *
 * Each test also pushes one valid frame after the bad one and drains to
 * confirm the encoder keeps running normally after a rejection.
 */

/* Branch: !vdef_dim_cmp (line 318) */
static void test_input_filter_wrong_resolution(void)
{
	int res;
	struct venc_config config;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	fill_lifecycle_x264_config(&config);
	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	/* Frame with wrong resolution is rejected synchronously at push */
	res = push_frame_wrong_resolution(queue, 0);
	CU_ASSERT_EQUAL(res, -EPROTO);

	/* Valid frame is accepted */
	res = push_frame(queue, 1);
	CU_ASSERT_EQUAL(res, 0);

	/* Drain: only the one valid frame comes out */
	res = venc_flush(enc, 0);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, 1);

	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/* Branch: timestamp <= last_timestamp (line 304) */
static void test_input_filter_nonmonotonic_timestamp(void)
{
	int res;
	struct venc_config config;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	fill_lifecycle_x264_config(&config);
	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	/* index=5 → ts=166666 µs: accepted, last_timestamp updated */
	res = push_frame(queue, 5);
	CU_ASSERT_EQUAL(res, 0);

	/* index=3 → ts=100000 µs: non-monotonic, rejected */
	res = push_frame(queue, 3);
	CU_ASSERT_EQUAL(res, -EPROTO);

	/* index=10 → ts=333333 µs: monotonic again, accepted */
	res = push_frame(queue, 10);
	CU_ASSERT_EQUAL(res, 0);

	/* Drain: the two accepted frames come out, the middle one was rejected
	 */
	res = venc_flush(enc, 0);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, 2);

	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/* Branch: !vdef_raw_format_intersect (line 293) */
static void test_input_filter_unsupported_format(void)
{
	int res;
	struct venc_config config;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_venc_cbs_ctx ctx;
	struct venc_cbs cbs = {
		.frame_output = &mock_venc_frame_output_cb,
		.flush = &mock_venc_flush_cb,
		.stop = &mock_venc_stop_cb,
	};
	struct venc_encoder *enc = NULL;
	struct mbuf_raw_video_frame_queue *queue;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	/* x264 supports i420/yv12/nv12/nv21/gray — RGB is not in the list */
	fill_lifecycle_x264_config(&config);
	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = venc_get_input_buffer_queue(enc);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	/* RGB frame is rejected (unsupported format, venc_core.c:293) */
	res = push_rgb_frame(queue, 0);
	CU_ASSERT_EQUAL(res, -EPROTO);

	/* Valid I420 frame is accepted */
	res = push_frame(queue, 1);
	CU_ASSERT_EQUAL(res, 0);

	res = venc_flush(enc, 0);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, 1);

	mock_venc_cbs_ctx_clear(&ctx);

	res = venc_stop(enc);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	venc_destroy(enc);
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* TEST TABLE                                                          */
/* ================================================================== */

CU_TestInfo g_venc_test_lifecycle[] = {
	/* frame-output-and-drain */
	{FN("frame-output-and-drain-x264"), &test_frame_output_and_drain_x264},
	{FN("frame-output-and-drain-x265"), &test_frame_output_and_drain_x265},
	{FN("frame-output-and-drain-turbojpeg"),
	 &test_frame_output_and_drain_turbojpeg},
	{FN("frame-output-and-drain-png"), &test_frame_output_and_drain_png},
	{FN("frame-output-and-drain-ffmpeg"),
	 &test_frame_output_and_drain_ffmpeg},
	{FN("frame-output-and-drain-ffmpeg-h265"),
	 &test_frame_output_and_drain_ffmpeg_h265},
	{FN("output-format-defaults-to-byte-stream-ffmpeg"),
	 &test_output_format_defaults_to_byte_stream_ffmpeg},
	{FN("output-format-defaults-to-byte-stream-ffmpeg-h265"),
	 &test_output_format_defaults_to_byte_stream_ffmpeg_h265},

	/* frame-output-and-drain: x264 input-format variants */
	{FN("frame-output-and-drain-x264-nv12"),
	 &test_frame_output_and_drain_x264_nv12},
	{FN("frame-output-and-drain-x264-nv21"),
	 &test_frame_output_and_drain_x264_nv21},
	{FN("frame-output-and-drain-x264-gray"),
	 &test_frame_output_and_drain_x264_gray},

	/* frame-output-and-drain: x265 input-format variants */
	{FN("frame-output-and-drain-x265-gray"),
	 &test_frame_output_and_drain_x265_gray},

	/* frame-output-and-drain: turbojpeg input-format variants */
	{FN("frame-output-and-drain-turbojpeg-nv12"),
	 &test_frame_output_and_drain_turbojpeg_nv12},
	{FN("frame-output-and-drain-turbojpeg-nv21"),
	 &test_frame_output_and_drain_turbojpeg_nv21},
	{FN("frame-output-and-drain-turbojpeg-gray"),
	 &test_frame_output_and_drain_turbojpeg_gray},

	/* frame-output-and-drain: png input-format variants */
	{FN("frame-output-and-drain-png-rgb"),
	 &test_frame_output_and_drain_png_rgb},

	/* frame-output-and-drain: ffmpeg input-format variants */
	{FN("frame-output-and-drain-ffmpeg-gray"),
	 &test_frame_output_and_drain_ffmpeg_gray},

	/* flush-discard */
	{FN("flush-discard-x264"), &test_flush_discard_x264},
	{FN("flush-discard-x265"), &test_flush_discard_x265},
	{FN("flush-discard-turbojpeg"), &test_flush_discard_turbojpeg},
	{FN("flush-discard-png"), &test_flush_discard_png},
	{FN("flush-discard-ffmpeg"), &test_flush_discard_ffmpeg},

	/* flush-then-reencode */
	{FN("flush-then-reencode-x264"), &test_flush_then_reencode_x264},
	{FN("flush-then-reencode-x265"), &test_flush_then_reencode_x265},
	{FN("flush-then-reencode-turbojpeg"),
	 &test_flush_then_reencode_turbojpeg},
	{FN("flush-then-reencode-png"), &test_flush_then_reencode_png},
	{FN("flush-then-reencode-ffmpeg"), &test_flush_then_reencode_ffmpeg},

	/* dyn-config */
	{FN("dyn-config-x264"), &test_dyn_config_x264},
	{FN("dyn-config-x265"), &test_dyn_config_x265},
	{FN("dyn-config-turbojpeg"), &test_dyn_config_turbojpeg},
	{FN("dyn-config-png"), &test_dyn_config_png},
	{FN("dyn-config-ffmpeg"), &test_dyn_config_ffmpeg},

	/* dyn-config: CQ qp update (x264 lines 1576-1580; x265 not testable —
	 * see comment above test_dyn_config_cq_qp_change_x265) */
	{FN("dyn-config-cq-qp-change-x264"),
	 &test_dyn_config_cq_qp_change_x264},

	/* dyn-config: decimation change (x264/x265/ffmpeg) */
	{FN("dyn-config-decimation-change-x264"),
	 &test_dyn_config_decimation_change_x264},
	{FN("dyn-config-decimation-change-x265"),
	 &test_dyn_config_decimation_change_x265},
	{FN("dyn-config-decimation-change-ffmpeg"),
	 &test_dyn_config_decimation_change_ffmpeg},

	/* dyn-config: turbojpeg-specific branches */
	{FN("dyn-config-turbojpeg-qp-change"),
	 &test_dyn_config_turbojpeg_qp_change},
	{FN("dyn-config-turbojpeg-bitrate-vbr"),
	 &test_dyn_config_turbojpeg_bitrate_vbr},

	/* decimation: frame-skip encoding loop (x264/x265/ffmpeg) */
	{FN("decimation-x264"), &test_decimation_x264},
	{FN("decimation-x265"), &test_decimation_x265},
	{FN("decimation-ffmpeg"), &test_decimation_ffmpeg},

	/* request-idr */
	{FN("request-idr-x264"), &test_request_idr_x264},
	{FN("request-idr-x265"), &test_request_idr_x265},
	{FN("request-idr-turbojpeg"), &test_request_idr_turbojpeg},
	{FN("request-idr-png"), &test_request_idr_png},
	{FN("request-idr-ffmpeg"), &test_request_idr_ffmpeg},

	/* request-idr: verify the IDR is actually produced in output */
	{FN("request-idr-x264-idr-produced"),
	 &test_request_idr_x264_idr_produced},
	{FN("request-idr-x265-idr-produced"),
	 &test_request_idr_x265_idr_produced},
	{FN("request-idr-ffmpeg-idr-produced"),
	 &test_request_idr_ffmpeg_idr_produced},

	/* get-input-buffer-pool */
	{FN("get-input-buffer-pool-null-self"),
	 &test_get_input_buffer_pool_null_self},
	{FN("get-input-buffer-pool-x264"), &test_get_input_buffer_pool_x264},
	{FN("get-input-buffer-pool-x265"), &test_get_input_buffer_pool_x265},
	{FN("get-input-buffer-pool-turbojpeg"),
	 &test_get_input_buffer_pool_turbojpeg},
	{FN("get-input-buffer-pool-png"), &test_get_input_buffer_pool_png},
	{FN("get-input-buffer-pool-ffmpeg"),
	 &test_get_input_buffer_pool_ffmpeg},

	/* get-h264-ps */
	{FN("get-h264-ps-x264"), &test_get_h264_ps_x264},
	{FN("get-h264-ps-x265"), &test_get_h264_ps_x265},
	{FN("get-h264-ps-turbojpeg"), &test_get_h264_ps_turbojpeg},
	{FN("get-h264-ps-png"), &test_get_h264_ps_png},
	{FN("get-h264-ps-ffmpeg"), &test_get_h264_ps_ffmpeg},

	/* get-h265-ps */
	{FN("get-h265-ps-x265"), &test_get_h265_ps_x265},
	{FN("get-h265-ps-extended-sar-x265"),
	 &test_get_h265_ps_extended_sar_x265},
	{FN("get-h265-ps-ffmpeg"), &test_get_h265_ps_ffmpeg},

	/* x264 H264 streaming SEI (codec-specific) */
	{FN("h264-streaming-sei-v2"), &test_h264_streaming_sei_v2},
	{FN("h264-streaming-sei-v4"), &test_h264_streaming_sei_v4},

	/* x264 H264 SEI/ancillary coverage */
	{FN("h264-pic-timing-sei"), &test_h264_pic_timing_sei},
	{FN("h264-serialize-user-data"), &test_h264_serialize_user_data},

	/* AVCC/HVCC output format (x264, x265, ffmpeg) */
	{FN("h264-avcc-output-x264"), &test_h264_avcc_output_x264},
	{FN("h265-avcc-output"), &test_h265_avcc_output},
	{FN("h264-avcc-output-ffmpeg"), &test_h264_avcc_output_ffmpeg},

	/* x265 H265 ancillary coverage */
	{FN("h265-serialize-user-data"), &test_h265_serialize_user_data},
	/* insert_mdcv_sei: known primaries (first OR arm) */
	{FN("h265-mdcv-sei-known-primaries"),
	 &test_h265_mdcv_sei_known_primaries_x265},
	/* insert_mdcv_sei: UNKNOWN primaries + custom chromaticity (second OR
	 * arm, covers venc_h265.c lines 1093-1114) */
	{FN("h265-mdcv-sei-custom-primaries"),
	 &test_h265_mdcv_sei_custom_primaries_x265},

	/* H264 insert_recovery_point_sei (lines 1047-1054 of venc_h264.c) */
	{FN("h264-insert-recovery-point-sei"),
	 &test_h264_insert_recovery_point_sei},

	/* implem-auto: AUTO resolves to first registered backend and encodes */
	{FN("frame-output-and-drain-implem-auto-h264"),
	 &test_frame_output_and_drain_implem_auto_h264},

	/* flush-discard-then-reencode */
	{FN("flush-discard-then-reencode-x264"),
	 &test_flush_discard_then_reencode_x264},
	{FN("flush-discard-then-reencode-x265"),
	 &test_flush_discard_then_reencode_x265},
	{FN("flush-discard-then-reencode-turbojpeg"),
	 &test_flush_discard_then_reencode_turbojpeg},
	{FN("flush-discard-then-reencode-png"),
	 &test_flush_discard_then_reencode_png},
	{FN("flush-discard-then-reencode-ffmpeg"),
	 &test_flush_discard_then_reencode_ffmpeg},

	/* dyn-config mid-encode (hot-update path) */
	{FN("dyn-config-mid-encode-x264"), &test_dyn_config_mid_encode_x264},
	{FN("dyn-config-mid-encode-x265"), &test_dyn_config_mid_encode_x265},
	{FN("dyn-config-mid-encode-ffmpeg"),
	 &test_dyn_config_mid_encode_ffmpeg},

	/* two simultaneous encoders on the same loop */
	{FN("two-simultaneous-encoders"), &test_two_simultaneous_encoders},

	/* delayed stop: stop_done fires only after all output frames are
	   released */
	{FN("stop-delayed-x264"), &test_stop_delayed_x264},
	{FN("stop-delayed-x265"), &test_stop_delayed_x265},
	{FN("stop-delayed-turbojpeg"), &test_stop_delayed_turbojpeg},
	{FN("stop-delayed-png"), &test_stop_delayed_png},
	{FN("stop-delayed-ffmpeg"), &test_stop_delayed_ffmpeg},

	/* input-filter rejection (venc_default_input_filter_internal branches)
	 */
	{FN("input-filter-wrong-resolution"),
	 &test_input_filter_wrong_resolution},
	{FN("input-filter-nonmonotonic-timestamp"),
	 &test_input_filter_nonmonotonic_timestamp},
	{FN("input-filter-unsupported-format"),
	 &test_input_filter_unsupported_format},

	CU_TEST_INFO_NULL,
};
