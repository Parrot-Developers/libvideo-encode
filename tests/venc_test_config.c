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

/* Each header is only exported transitively when its backend is actually
 * linked into libvideo-encode (see atom.mk's BUILD_LIBVIDEO_ENCODE_<NAME>
 * defines for this test module) - guard every include accordingly so
 * toggling a CONFIG_VENC_<NAME> Kconfig option off doesn't break the build
 * of this file. */
#ifdef BUILD_LIBVIDEO_ENCODE_X264
#	include <video-encode/venc_x264.h>
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_X265
#	include <video-encode/venc_x265.h>
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_FFMPEG
#	include <video-encode/venc_ffmpeg.h>
#endif

/*
 * venc_new()/venc_config_copy()/venc_config_free() validation and defaulting
 * logic (src/venc.c), exercised through real, pure-software, already-in-tree
 * backends (x264/x265/turbojpeg/png) so the format-intersect/dispatch checks
 * that gate every per-encoding branch are exercised for real rather than
 * mocked. Every backend's get_supported_input_formats() ignores its
 * `encoding` argument (verified by reading each backend's source), so a
 * config's `encoding` field can be deliberately made invalid without the
 * dispatch itself rejecting it first, letting the final default-case switch
 * in venc_new() be reached and tested too.
 */

static void dummy_frame_output_cb(struct venc_encoder *enc,
				  int status,
				  struct mbuf_coded_video_frame *frame,
				  void *userdata)
{
	(void)enc;
	(void)status;
	(void)frame;
	(void)userdata;
}


static void fill_base_config(struct venc_config *config,
			     enum venc_encoder_implem implem,
			     enum vdef_encoding encoding,
			     struct vdef_raw_format format)
{
	memset(config, 0, sizeof(*config));
	config->implem = implem;
	config->encoding = encoding;
	config->input.format = format;
	config->input.info.resolution.width = 640;
	config->input.info.resolution.height = 480;
	config->input.info.framerate.num = 30;
	config->input.info.framerate.den = 1;
}


static void fill_valid_h264_config(struct venc_config *config,
				   enum venc_encoder_implem implem)
{
	fill_base_config(config, implem, VDEF_ENCODING_H264, vdef_i420);
	config->h264.rate_control = VENC_RATE_CONTROL_CBR;
	config->h264.max_bitrate = 4000000;
}


static void fill_valid_h265_config(struct venc_config *config,
				   enum venc_encoder_implem implem)
{
	fill_base_config(config, implem, VDEF_ENCODING_H265, vdef_i420);
	config->h265.rate_control = VENC_RATE_CONTROL_CBR;
	config->h265.max_bitrate = 4000000;
}


static void fill_valid_mjpeg_config(struct venc_config *config,
				    enum venc_encoder_implem implem)
{
	fill_base_config(config, implem, VDEF_ENCODING_MJPEG, vdef_i420);
	config->mjpeg.rate_control = VENC_RATE_CONTROL_CBR;
	config->mjpeg.max_bitrate = 4000000;
}


static void fill_valid_png_config(struct venc_config *config,
				  enum venc_encoder_implem implem)
{
	fill_base_config(config, implem, VDEF_ENCODING_PNG, vdef_gray);
	config->png.compression_level = 6;
}


/*
 * Argument guards.
 */

static void test_venc_new_null_args(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);

	res = venc_new(NULL, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_new(loop, NULL, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_new(loop, &config, NULL, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_new(loop, &config, &cbs, NULL, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_venc_new_unbuilt_implem_returns_enosys(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	/* HISI/QCOM/QCOM_JPEG/MEDIACODEC/VIDEOTOOLBOX are not built for the
	 * groundsdk-linux target - venc_get_implem() must reject them. */
	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_HISI);
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -ENOSYS);
	CU_ASSERT_PTR_NULL(enc);

	pomp_loop_destroy(loop);
}


static void test_venc_new_unsupported_input_format(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	/* PNG only supports vdef_gray/vdef_gray16/vdef_raw8: vdef_i420 must
	 * be rejected. */
	fill_valid_png_config(&config, VENC_ENCODER_IMPLEM_PNG);
	config.input.format = vdef_i420;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);
	CU_ASSERT_PTR_NULL(enc);

	pomp_loop_destroy(loop);
}


static void test_venc_new_invalid_dimensions_and_framerate(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.input.info.resolution.width = 0;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.input.info.resolution.height = 0;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.input.info.framerate.num = 0;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.input.info.framerate.den = 0;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_venc_new_unknown_encoding(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	/* x264's get_supported_input_formats() ignores its `encoding`
	 * argument, so a bogus encoding value reaches venc_new()'s final
	 * per-encoding switch and its default case. */
	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.encoding = (enum vdef_encoding)999;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);
	CU_ASSERT_PTR_NULL(enc);

	pomp_loop_destroy(loop);
}


/*
 * H.264-specific validation/defaulting.
 */

static void test_venc_new_h264_cq_invalid_qp(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.h264.rate_control = VENC_RATE_CONTROL_CQ;
	config.h264.qp = 0;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	config.h264.qp = 52;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_venc_new_h264_cbr_vbr_requires_max_bitrate(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.h264.rate_control = VENC_RATE_CONTROL_CBR;
	config.h264.max_bitrate = 0;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	config.h264.rate_control = VENC_RATE_CONTROL_VBR;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_venc_new_h264_min_max_qp_range_and_order(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	/* Out of [1:51] range */
	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.h264.min_qp = 52;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.h264.max_qp = 52;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* min_qp > max_qp */
	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.h264.min_qp = 40;
	config.h264.max_qp = 10;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_venc_new_h264_base_ref_frame_interval_mismatch(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.h264.ref_frame_interval = 2;
	config.h264.base_frame_interval = 3; /* not a multiple of 2 */
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


/*
 * Intra-refresh branch in venc_new() (src/venc.c line ~652):
 *   recovery_frame_cnt = (intra_refresh != NONE && intra_refresh_length > 0)
 *     ? (intra_refresh_length - 1) * base_frame_interval / ref_frame_interval
 *     : 0;
 * The true branch is taken when both conditions hold; both ref/base intervals
 * default safely to 1 so there is no division-by-zero.
 */
static void test_venc_new_h264_intra_refresh(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.h264.intra_refresh = VENC_INTRA_REFRESH_VERTICAL_SCAN;
	config.h264.intra_refresh_length = 3;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, 0);
	venc_destroy(enc);

	pomp_loop_destroy(loop);
}


static void test_venc_new_h264_invalid_streaming_sei_version(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.h264.streaming_user_data_sei_version = 3;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_venc_new_h264_valid_config_succeeds(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;
	enum venc_encoder_implem implem;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(enc);

	if (enc != NULL) {
		/* AUTO is never stored: venc_get_used_implem() must reflect
		 * the concrete implem actually used. */
		implem = venc_get_used_implem(enc);
		CU_ASSERT_EQUAL(implem, VENC_ENCODER_IMPLEM_X264);
		venc_destroy(enc);
	}

	pomp_loop_destroy(loop);
}


/*
 * H.265-specific validation/defaulting (same shape as H.264, minus slicing).
 */

static void test_venc_new_h265_cq_invalid_qp(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_h265_config(&config, VENC_ENCODER_IMPLEM_X265);
	config.h265.rate_control = VENC_RATE_CONTROL_CQ;
	config.h265.qp = 0;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_venc_new_h265_cbr_vbr_requires_max_bitrate(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_h265_config(&config, VENC_ENCODER_IMPLEM_X265);
	config.h265.max_bitrate = 0;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_venc_new_h265_min_max_qp_range(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	/* min_qp > 51: out of [1:51] range */
	fill_valid_h265_config(&config, VENC_ENCODER_IMPLEM_X265);
	config.h265.min_qp = 52;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* max_qp > 51: out of [1:51] range */
	fill_valid_h265_config(&config, VENC_ENCODER_IMPLEM_X265);
	config.h265.max_qp = 52;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_venc_new_h265_min_max_qp_order(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_h265_config(&config, VENC_ENCODER_IMPLEM_X265);
	config.h265.min_qp = 40;
	config.h265.max_qp = 10;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_venc_new_h265_invalid_streaming_sei_version(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_h265_config(&config, VENC_ENCODER_IMPLEM_X265);
	config.h265.streaming_user_data_sei_version = 1;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_venc_new_h265_valid_config_succeeds(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_h265_config(&config, VENC_ENCODER_IMPLEM_X265);
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(enc);

	if (enc != NULL)
		venc_destroy(enc);

	pomp_loop_destroy(loop);
}


/*
 * MJPEG-specific validation/defaulting.
 */

static void test_venc_new_mjpeg_cq_invalid_quality(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_mjpeg_config(&config, VENC_ENCODER_IMPLEM_TURBOJPEG);
	config.mjpeg.rate_control = VENC_RATE_CONTROL_CQ;
	config.mjpeg.quality = 0;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_venc_new_mjpeg_quality_upper_bound_not_enforced(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	/* Documented range is [1..99] (venc_core.h), but venc_new() never
	 * actually checks the upper bound in CQ mode - this pins down that
	 * known, current gap rather than the documented contract. */
	fill_valid_mjpeg_config(&config, VENC_ENCODER_IMPLEM_TURBOJPEG);
	config.mjpeg.rate_control = VENC_RATE_CONTROL_CQ;
	config.mjpeg.quality = 150;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, 0);

	if (enc != NULL)
		venc_destroy(enc);

	pomp_loop_destroy(loop);
}


static void test_venc_new_mjpeg_cbr_vbr_requires_max_bitrate(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_mjpeg_config(&config, VENC_ENCODER_IMPLEM_TURBOJPEG);
	config.mjpeg.max_bitrate = 0;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_venc_new_mjpeg_valid_config_succeeds(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_mjpeg_config(&config, VENC_ENCODER_IMPLEM_TURBOJPEG);
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(enc);

	if (enc != NULL)
		venc_destroy(enc);

	pomp_loop_destroy(loop);
}


/*
 * PNG-specific validation.
 */

static void test_venc_new_png_invalid_compression_level(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_png_config(&config, VENC_ENCODER_IMPLEM_PNG);
	config.png.compression_level = 10;
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_venc_new_png_valid_config_succeeds(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_config config;
	struct venc_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct venc_encoder *enc = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	fill_valid_png_config(&config, VENC_ENCODER_IMPLEM_PNG);
	res = venc_new(loop, &config, &cbs, NULL, &enc);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(enc);

	if (enc != NULL)
		venc_destroy(enc);

	pomp_loop_destroy(loop);
}


/*
 * venc_config_copy()/venc_config_free().
 */

static void test_config_copy_deep_copy_independence(void)
{
	struct venc_config config;
	struct venc_config *copy = NULL;
	int res;

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.name = "original-name";
	config.device = "original-device";

	res = venc_config_copy(&config, &copy);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(copy);

	CU_ASSERT_STRING_EQUAL(copy->name, "original-name");
	CU_ASSERT_STRING_EQUAL(copy->device, "original-device");
	/* Deep copy: the string storage must not be the same pointer */
	CU_ASSERT_PTR_NOT_EQUAL(copy->name, config.name);
	CU_ASSERT_PTR_NOT_EQUAL(copy->device, config.device);

	copy->h264.max_bitrate = 1;
	CU_ASSERT_NOT_EQUAL(copy->h264.max_bitrate, config.h264.max_bitrate);

	venc_config_free(copy);
}


static void test_config_copy_null_args(void)
{
	struct venc_config config;
	struct venc_config *copy = NULL;
	int res;

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);

	res = venc_config_copy(NULL, &copy);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_config_copy(&config, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	/* venc_config_free() tolerates NULL as a no-op */
	res = venc_config_free(NULL);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_config_copy_optional_name_device_may_be_null(void)
{
	struct venc_config config;
	struct venc_config *copy = NULL;
	int res;

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);
	config.name = NULL;
	config.device = NULL;

	res = venc_config_copy(&config, &copy);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NULL(copy->name);
	CU_ASSERT_PTR_NULL(copy->device);

	venc_config_free(copy);
}


/*
 * venc_config_copy() / venc_config_free() with implementation-specific config.
 *
 * When config.implem_cfg != NULL, venc_config_copy() must call the implem's
 * copy_implem_cfg() (lines 354-361 in src/venc.c), and venc_config_free()
 * must call free_implem_cfg() (lines 421-427). Triggered here by supplying a
 * heap-allocated struct venc_config_x264 as the implem_cfg.
 *
 * Ownership: implem_cfg must be heap-allocated because free_implem_cfg()
 * calls free() on the preset/tune strings AND free() on the struct itself.
 * The COPY's implem_cfg is owned by the copy and freed by venc_config_free().
 * The SOURCE's implem_cfg is owned by this test and freed manually at the end.
 */
#ifdef BUILD_LIBVIDEO_ENCODE_X264
static void test_config_copy_with_implem_cfg(void)
{
	int res;
	struct venc_config config;
	struct venc_config *copy = NULL;
	struct venc_config_x264 *x264_cfg;

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_X264);

	x264_cfg = calloc(1, sizeof(*x264_cfg));
	CU_ASSERT_PTR_NOT_NULL_FATAL(x264_cfg);
	x264_cfg->implem = VENC_ENCODER_IMPLEM_X264;
	x264_cfg->preset = strdup("ultrafast");
	x264_cfg->tune = strdup("zerolatency");
	config.implem_cfg = (struct venc_config_impl *)x264_cfg;

	/* copy_implem_cfg is called: produces a separate heap-allocated copy */
	res = venc_config_copy(&config, &copy);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(copy);
	CU_ASSERT_PTR_NOT_NULL(copy->implem_cfg);
	CU_ASSERT_PTR_NOT_EQUAL(copy->implem_cfg, config.implem_cfg);

	/* free_implem_cfg is called on copy's implem_cfg */
	venc_config_free(copy);

	/* Release the source's implem_cfg manually (it is not freed by copy) */
	free((void *)x264_cfg->preset);
	free((void *)x264_cfg->tune);
	free(x264_cfg);
	config.implem_cfg = NULL;
}
#endif /* BUILD_LIBVIDEO_ENCODE_X264 */


#ifdef BUILD_LIBVIDEO_ENCODE_X265
static void test_config_copy_with_x265_implem_cfg(void)
{
	int res;
	struct venc_config config;
	struct venc_config *copy = NULL;
	struct venc_config_x265 *x265_cfg;

	if (!implem_available(VENC_ENCODER_IMPLEM_X265))
		return;

	fill_valid_h265_config(&config, VENC_ENCODER_IMPLEM_X265);

	x265_cfg = calloc(1, sizeof(*x265_cfg));
	CU_ASSERT_PTR_NOT_NULL_FATAL(x265_cfg);
	x265_cfg->implem = VENC_ENCODER_IMPLEM_X265;
	x265_cfg->preset = strdup("ultrafast");
	x265_cfg->tune = strdup("zerolatency");
	config.implem_cfg = (struct venc_config_impl *)x265_cfg;

	/* copy_implem_cfg is called: produces a separate heap-allocated copy */
	res = venc_config_copy(&config, &copy);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(copy);
	CU_ASSERT_PTR_NOT_NULL(copy->implem_cfg);
	CU_ASSERT_PTR_NOT_EQUAL(copy->implem_cfg, config.implem_cfg);

	/* free_implem_cfg is called on copy's implem_cfg */
	venc_config_free(copy);

	/* Release the source's implem_cfg manually (it is not freed by copy) */
	free((void *)x265_cfg->preset);
	free((void *)x265_cfg->tune);
	free(x265_cfg);
	config.implem_cfg = NULL;
}
#endif /* BUILD_LIBVIDEO_ENCODE_X265 */


#ifdef BUILD_LIBVIDEO_ENCODE_FFMPEG
static void test_config_copy_with_ffmpeg_implem_cfg(void)
{
	int res;
	struct venc_config config;
	struct venc_config *copy = NULL;
	struct venc_config_ffmpeg *ffmpeg_cfg;

	if (!implem_available(VENC_ENCODER_IMPLEM_FFMPEG))
		return;

	fill_valid_h264_config(&config, VENC_ENCODER_IMPLEM_FFMPEG);

	ffmpeg_cfg = calloc(1, sizeof(*ffmpeg_cfg));
	CU_ASSERT_PTR_NOT_NULL_FATAL(ffmpeg_cfg);
	ffmpeg_cfg->implem = VENC_ENCODER_IMPLEM_FFMPEG;
	ffmpeg_cfg->preset = strdup("ultrafast");
	ffmpeg_cfg->tune = strdup("zerolatency");
	config.implem_cfg = (struct venc_config_impl *)ffmpeg_cfg;

	/* copy_implem_cfg is called: produces a separate heap-allocated copy */
	res = venc_config_copy(&config, &copy);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(copy);
	CU_ASSERT_PTR_NOT_NULL(copy->implem_cfg);
	CU_ASSERT_PTR_NOT_EQUAL(copy->implem_cfg, config.implem_cfg);

	/* free_implem_cfg is called on copy's implem_cfg */
	venc_config_free(copy);

	/* Release the source's implem_cfg manually (it is not freed by copy) */
	free((void *)ffmpeg_cfg->preset);
	free((void *)ffmpeg_cfg->tune);
	free(ffmpeg_cfg);
	config.implem_cfg = NULL;
}
#endif /* BUILD_LIBVIDEO_ENCODE_FFMPEG */


/*
 * Autoselection helpers.
 */

static void test_get_supported_implems_and_auto(void)
{
	const enum venc_encoder_implem *implems = NULL;
	int count;
	enum venc_encoder_implem auto_implem;

	count = venc_get_supported_implems(&implems);
	CU_ASSERT_TRUE(count > 0);
	CU_ASSERT_PTR_NOT_NULL(implems);

	/* AUTO must resolve to the first entry ("preferred first") */
	auto_implem = venc_get_auto_implem();
	CU_ASSERT_EQUAL(auto_implem, implems[0]);
}


static void test_get_auto_implem_by_encoding_invalid_args(void)
{
	enum venc_encoder_implem implem;

	implem = venc_get_auto_implem_by_encoding(VDEF_ENCODING_UNKNOWN);
	CU_ASSERT_EQUAL(implem, VENC_ENCODER_IMPLEM_AUTO);

	implem = venc_get_auto_implem_by_encoding_and_format(
		VDEF_ENCODING_UNKNOWN, &vdef_i420);
	CU_ASSERT_EQUAL(implem, VENC_ENCODER_IMPLEM_AUTO);

	implem = venc_get_auto_implem_by_encoding_and_format(VDEF_ENCODING_H264,
							     NULL);
	CU_ASSERT_EQUAL(implem, VENC_ENCODER_IMPLEM_AUTO);
}


static void test_get_auto_implem_by_encoding_h264(void)
{
	enum venc_encoder_implem implem;

	implem = venc_get_auto_implem_by_encoding(VDEF_ENCODING_H264);
	CU_ASSERT_NOT_EQUAL(implem, VENC_ENCODER_IMPLEM_AUTO);

	implem = venc_get_auto_implem_by_encoding_and_format(VDEF_ENCODING_H264,
							     &vdef_i420);
	CU_ASSERT_NOT_EQUAL(implem, VENC_ENCODER_IMPLEM_AUTO);
}


/*
 * venc_new() with a (implem, encoding) pair where the implem does not support
 * the encoding.  Each backend's create() validates encoding and returns
 * -EINVAL:
 *   x264   only supports H264 (venc_x264.c:1229)
 *   turbojpeg only supports MJPEG (venc_turbojpeg.c:793)
 *   png    only supports PNG (venc_png.c:820)
 *
 * All three combos use a fully valid config (dimensions, framerate, etc.) for
 * the chosen backend — only the encoding field is wrong — so venc_new() gets
 * past the generic validation and fails inside the backend create().
 */
static void test_venc_new_wrong_encoding_for_implem(void)
{
	int res;
	struct pomp_loop *loop = pomp_loop_new();
	struct venc_cbs cbs = {.frame_output = &mock_venc_frame_output_cb};
	struct venc_encoder *enc = NULL;
	struct venc_config config;
	struct mock_venc_cbs_ctx ctx;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_venc_cbs_ctx_reset(&ctx);

	/* x264 + MJPEG → -EINVAL */
	memset(&config, 0, sizeof(config));
	config.implem = VENC_ENCODER_IMPLEM_X264;
	config.encoding = VDEF_ENCODING_MJPEG;
	config.input.format = vdef_i420;
	config.input.info.resolution.width = 64;
	config.input.info.resolution.height = 64;
	config.input.info.framerate.num = 30;
	config.input.info.framerate.den = 1;
	config.mjpeg.rate_control = VENC_RATE_CONTROL_CQ;
	config.mjpeg.quality = 80;
	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);
	CU_ASSERT_PTR_NULL(enc);

	/* turbojpeg + H264 → -EINVAL */
	memset(&config, 0, sizeof(config));
	config.implem = VENC_ENCODER_IMPLEM_TURBOJPEG;
	config.encoding = VDEF_ENCODING_H264;
	config.input.format = vdef_i420;
	config.input.info.resolution.width = 64;
	config.input.info.resolution.height = 64;
	config.input.info.framerate.num = 30;
	config.input.info.framerate.den = 1;
	config.h264.rate_control = VENC_RATE_CONTROL_CBR;
	config.h264.max_bitrate = 500000;
	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);
	CU_ASSERT_PTR_NULL(enc);

	/* png + H265 → -EINVAL */
	memset(&config, 0, sizeof(config));
	config.implem = VENC_ENCODER_IMPLEM_PNG;
	config.encoding = VDEF_ENCODING_H265;
	config.input.format = vdef_gray;
	config.input.info.resolution.width = 64;
	config.input.info.resolution.height = 64;
	config.input.info.framerate.num = 30;
	config.input.info.framerate.den = 1;
	config.input.info.bit_depth = 8;
	config.h265.rate_control = VENC_RATE_CONTROL_CBR;
	config.h265.max_bitrate = 500000;
	res = venc_new(loop, &config, &cbs, &ctx, &enc);
	CU_ASSERT_EQUAL(res, -EINVAL);
	CU_ASSERT_PTR_NULL(enc);

	pomp_loop_destroy(loop);
}


/*
 * MJPEG: turbojpeg is the only MJPEG backend in this product.
 * PNG: the png backend is the only PNG backend.
 * Both must resolve to a non-AUTO implem (not UNKNOWN encoding / no-backend).
 */
static void test_get_auto_implem_by_encoding_mjpeg_and_png(void)
{
	enum venc_encoder_implem implem;

	implem = venc_get_auto_implem_by_encoding(VDEF_ENCODING_MJPEG);
	CU_ASSERT_EQUAL(implem, VENC_ENCODER_IMPLEM_TURBOJPEG);

	implem = venc_get_auto_implem_by_encoding_and_format(
		VDEF_ENCODING_MJPEG, &vdef_i420);
	CU_ASSERT_EQUAL(implem, VENC_ENCODER_IMPLEM_TURBOJPEG);

	implem = venc_get_auto_implem_by_encoding(VDEF_ENCODING_PNG);
	CU_ASSERT_EQUAL(implem, VENC_ENCODER_IMPLEM_PNG);

	implem = venc_get_auto_implem_by_encoding_and_format(VDEF_ENCODING_PNG,
							     &vdef_gray);
	CU_ASSERT_EQUAL(implem, VENC_ENCODER_IMPLEM_PNG);
}


/*
 * venc_request_idr() encoding-type guard (pure, independent of whether the
 * backend actually implements request_idr).
 */

static void test_request_idr_rejects_non_h26x_encodings(void)
{
	struct venc_encoder enc;
	int res;

	memset(&enc, 0, sizeof(enc));
	enc.config.encoding = VDEF_ENCODING_MJPEG;
	res = venc_request_idr(&enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	enc.config.encoding = VDEF_ENCODING_PNG;
	res = venc_request_idr(&enc);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = venc_request_idr(NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_venc_get_supported_encodings(void)
{
	const enum vdef_encoding *encodings = NULL;
	int res;
	bool found;

	/* NULL output pointer -> -EINVAL */
	res = venc_get_supported_encodings(VENC_ENCODER_IMPLEM_X264, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* Unbuilt implem: venc_get_implem() fails -> returns 0 (empty list),
	 * not an error code */
	res = venc_get_supported_encodings(VENC_ENCODER_IMPLEM_HISI,
					   &encodings);
	CU_ASSERT_EQUAL(res, 0);

	/* x264 must advertise H264 */
	encodings = NULL;
	res = venc_get_supported_encodings(VENC_ENCODER_IMPLEM_X264,
					   &encodings);
	CU_ASSERT_TRUE(res > 0);
	CU_ASSERT_PTR_NOT_NULL(encodings);
	found = false;
	for (int i = 0; i < res; i++) {
		if (encodings[i] == VDEF_ENCODING_H264)
			found = true;
	}
	CU_ASSERT_TRUE(found);

	/* png must advertise PNG */
	encodings = NULL;
	res = venc_get_supported_encodings(VENC_ENCODER_IMPLEM_PNG, &encodings);
	CU_ASSERT_TRUE(res > 0);
	CU_ASSERT_PTR_NOT_NULL(encodings);
}


static void test_venc_get_supported_input_formats(void)
{
	const struct vdef_raw_format *formats = NULL;
	int res;

	/* NULL output pointer -> -EINVAL */
	res = venc_get_supported_input_formats(
		VENC_ENCODER_IMPLEM_X264, VDEF_ENCODING_H264, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* x264: H264 encoding must have at least one supported input format */
	res = venc_get_supported_input_formats(
		VENC_ENCODER_IMPLEM_X264, VDEF_ENCODING_H264, &formats);
	CU_ASSERT_TRUE(res > 0);
	CU_ASSERT_PTR_NOT_NULL(formats);

	/* x264: H265 encoding not supported -> -ENOSYS */
	res = venc_get_supported_input_formats(
		VENC_ENCODER_IMPLEM_X264, VDEF_ENCODING_H265, &formats);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	/* AUTO + H264: resolves to x264 in this build -> at least one format */
	formats = NULL;
	res = venc_get_supported_input_formats(
		VENC_ENCODER_IMPLEM_AUTO, VDEF_ENCODING_H264, &formats);
	CU_ASSERT_TRUE(res > 0);
	CU_ASSERT_PTR_NOT_NULL(formats);
}


CU_TestInfo g_venc_test_config[] = {
	{FN("venc-new-null-args"), &test_venc_new_null_args},
	{FN("venc-new-unbuilt-implem-returns-enosys"),
	 &test_venc_new_unbuilt_implem_returns_enosys},
	{FN("venc-new-unsupported-input-format"),
	 &test_venc_new_unsupported_input_format},
	{FN("venc-new-invalid-dimensions-and-framerate"),
	 &test_venc_new_invalid_dimensions_and_framerate},
	{FN("venc-new-unknown-encoding"), &test_venc_new_unknown_encoding},

	{FN("venc-new-h264-cq-invalid-qp"), &test_venc_new_h264_cq_invalid_qp},
	{FN("venc-new-h264-cbr-vbr-requires-max-bitrate"),
	 &test_venc_new_h264_cbr_vbr_requires_max_bitrate},
	{FN("venc-new-h264-min-max-qp-range-and-order"),
	 &test_venc_new_h264_min_max_qp_range_and_order},
	{FN("venc-new-h264-base-ref-frame-interval-mismatch"),
	 &test_venc_new_h264_base_ref_frame_interval_mismatch},
	{FN("venc-new-h264-intra-refresh"), &test_venc_new_h264_intra_refresh},
	{FN("venc-new-h264-invalid-streaming-sei-version"),
	 &test_venc_new_h264_invalid_streaming_sei_version},
	{FN("venc-new-h264-valid-config-succeeds"),
	 &test_venc_new_h264_valid_config_succeeds},

	{FN("venc-new-h265-cq-invalid-qp"), &test_venc_new_h265_cq_invalid_qp},
	{FN("venc-new-h265-cbr-vbr-requires-max-bitrate"),
	 &test_venc_new_h265_cbr_vbr_requires_max_bitrate},
	{FN("venc-new-h265-min-max-qp-range"),
	 &test_venc_new_h265_min_max_qp_range},
	{FN("venc-new-h265-min-max-qp-order"),
	 &test_venc_new_h265_min_max_qp_order},
	{FN("venc-new-h265-invalid-streaming-sei-version"),
	 &test_venc_new_h265_invalid_streaming_sei_version},
	{FN("venc-new-h265-valid-config-succeeds"),
	 &test_venc_new_h265_valid_config_succeeds},

	{FN("venc-new-mjpeg-cq-invalid-quality"),
	 &test_venc_new_mjpeg_cq_invalid_quality},
	{FN("venc-new-mjpeg-quality-upper-bound-not-enforced"),
	 &test_venc_new_mjpeg_quality_upper_bound_not_enforced},
	{FN("venc-new-mjpeg-cbr-vbr-requires-max-bitrate"),
	 &test_venc_new_mjpeg_cbr_vbr_requires_max_bitrate},
	{FN("venc-new-mjpeg-valid-config-succeeds"),
	 &test_venc_new_mjpeg_valid_config_succeeds},

	{FN("venc-new-png-invalid-compression-level"),
	 &test_venc_new_png_invalid_compression_level},
	{FN("venc-new-png-valid-config-succeeds"),
	 &test_venc_new_png_valid_config_succeeds},

	{FN("config-copy-deep-copy-independence"),
	 &test_config_copy_deep_copy_independence},
	{FN("config-copy-null-args"), &test_config_copy_null_args},
	{FN("config-copy-optional-name-device-may-be-null"),
	 &test_config_copy_optional_name_device_may_be_null},
#ifdef BUILD_LIBVIDEO_ENCODE_X264
	{FN("config-copy-with-implem-cfg"), &test_config_copy_with_implem_cfg},
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_X265
	{FN("config-copy-with-x265-implem-cfg"),
	 &test_config_copy_with_x265_implem_cfg},
#endif
#ifdef BUILD_LIBVIDEO_ENCODE_FFMPEG
	{FN("config-copy-with-ffmpeg-implem-cfg"),
	 &test_config_copy_with_ffmpeg_implem_cfg},
#endif

	{FN("get-supported-implems-and-auto"),
	 &test_get_supported_implems_and_auto},
	{FN("get-auto-implem-by-encoding-invalid-args"),
	 &test_get_auto_implem_by_encoding_invalid_args},
	{FN("get-auto-implem-by-encoding-h264"),
	 &test_get_auto_implem_by_encoding_h264},
	{FN("get-auto-implem-by-encoding-mjpeg-and-png"),
	 &test_get_auto_implem_by_encoding_mjpeg_and_png},
	{FN("venc-new-wrong-encoding-for-implem"),
	 &test_venc_new_wrong_encoding_for_implem},

	{FN("request-idr-rejects-non-h26x-encodings"),
	 &test_request_idr_rejects_non_h26x_encodings},

	{FN("get-supported-encodings"), &test_venc_get_supported_encodings},
	{FN("get-supported-input-formats"),
	 &test_venc_get_supported_input_formats},

	CU_TEST_INFO_NULL,
};
