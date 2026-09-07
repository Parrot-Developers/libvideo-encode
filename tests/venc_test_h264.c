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
 * venc_h264_get_nalu_importance() (core/src/venc_h264.c): pure enum switch,
 * no dependencies. Exhaustive coverage of every branch.
 */

static void test_h264_nalu_importance_non_slice_types(void)
{
	uint32_t importance;

	/* SPS/PPS/end-of-seq/end-of-stream/IDR slice are always importance 0,
	 * regardless of nri/frame_type/layer */
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SPS, 0, VDEF_CODED_FRAME_TYPE_P, 0);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_PPS, 0, VDEF_CODED_FRAME_TYPE_P, 0);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_END_OF_SEQ, 0, VDEF_CODED_FRAME_TYPE_P, 0);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_END_OF_STREAM, 0, VDEF_CODED_FRAME_TYPE_P, 0);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SLICE_IDR, 0, VDEF_CODED_FRAME_TYPE_P, 0);
	CU_ASSERT_EQUAL(importance, 0);
}


static void test_h264_nalu_importance_slice_idr_and_i(void)
{
	uint32_t importance;

	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SLICE, 3, VDEF_CODED_FRAME_TYPE_IDR, 0);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SLICE, 3, VDEF_CODED_FRAME_TYPE_I, 0);
	CU_ASSERT_EQUAL(importance, 0);
}


static void test_h264_nalu_importance_slice_p_nri_levels(void)
{
	uint32_t importance;

	/* nri == 3: IR pattern -> 0 */
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SLICE, 3, VDEF_CODED_FRAME_TYPE_P, 0);
	CU_ASSERT_EQUAL(importance, 0);
	/* nri == 2: base layer, non IR -> 1 */
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SLICE, 2, VDEF_CODED_FRAME_TYPE_P, 0);
	CU_ASSERT_EQUAL(importance, 1);
	/* nri == 1: layer 1 -> 2 */
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SLICE, 1, VDEF_CODED_FRAME_TYPE_P, 0);
	CU_ASSERT_EQUAL(importance, 2);
	/* nri == 0 (unknown/default) -> 2 */
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SLICE, 0, VDEF_CODED_FRAME_TYPE_P, 0);
	CU_ASSERT_EQUAL(importance, 2);

	/* P_IR_START follows the same nri-based switch as P */
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SLICE, 3, VDEF_CODED_FRAME_TYPE_P_IR_START, 0);
	CU_ASSERT_EQUAL(importance, 0);
}


static void test_h264_nalu_importance_slice_p_non_ref_and_default(void)
{
	uint32_t importance;

	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SLICE, 0, VDEF_CODED_FRAME_TYPE_P_NON_REF, 0);
	CU_ASSERT_EQUAL(importance, 3);
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SLICE, 0, VDEF_CODED_FRAME_TYPE_UNKNOWN, 0);
	CU_ASSERT_EQUAL(importance, UINT32_MAX);
}


static void test_h264_nalu_importance_sei(void)
{
	uint32_t importance;

	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SEI, 0, VDEF_CODED_FRAME_TYPE_IDR, 0);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SEI, 0, VDEF_CODED_FRAME_TYPE_P_IR_START, 0);
	CU_ASSERT_EQUAL(importance, 0);
	/* P frame type: importance depends on layer */
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SEI, 0, VDEF_CODED_FRAME_TYPE_P, 0);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SEI, 0, VDEF_CODED_FRAME_TYPE_P, 1);
	CU_ASSERT_EQUAL(importance, 2);
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SEI, 0, VDEF_CODED_FRAME_TYPE_P, 2);
	CU_ASSERT_EQUAL(importance, 1);
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SEI, 0, VDEF_CODED_FRAME_TYPE_P_NON_REF, 0);
	CU_ASSERT_EQUAL(importance, 3);
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_SEI, 0, VDEF_CODED_FRAME_TYPE_UNKNOWN, 0);
	CU_ASSERT_EQUAL(importance, 1);
}


static void test_h264_nalu_importance_aud_filler_unknown(void)
{
	uint32_t importance;

	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_AUD, 0, VDEF_CODED_FRAME_TYPE_P, 0);
	CU_ASSERT_EQUAL(importance, UINT32_MAX);
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_FILLER, 0, VDEF_CODED_FRAME_TYPE_P, 0);
	CU_ASSERT_EQUAL(importance, UINT32_MAX);
	importance = venc_h264_get_nalu_importance(
		H264_NALU_TYPE_UNKNOWN, 0, VDEF_CODED_FRAME_TYPE_P, 0);
	CU_ASSERT_EQUAL(importance, UINT32_MAX);
}


/*
 * Build a minimal, valid H.264 SPS/PPS pair (mirrors the construction done
 * by libvideo-encode-parrot's fakeh264 backend, without depending on that
 * separate repository) so that venc_h264_writer_new()/venc_h264_patch_ps()
 * can be exercised with real, parseable parameter sets.
 */
int venc_test_build_h264_ps(unsigned int width,
			    unsigned int height,
			    uint8_t **ret_sps,
			    size_t *ret_sps_size,
			    uint8_t **ret_pps,
			    size_t *ret_pps_size)
{
	int res = 0;
	struct h264_ctx *ctx = NULL;
	struct h264_sps *sps = NULL;
	struct h264_pps *pps = NULL;
	struct h264_nalu_header nh;
	struct h264_bitstream bs;
	uint8_t *sps_buf = NULL;
	uint8_t *pps_buf = NULL;
	/* VDEF_ROUND_UP(n, d) is a ceiling division ((n+d-1)/d), so this
	 * already yields a macroblock count, not a pixel width - do not
	 * divide by 16 again. */
	unsigned int mb_width = VDEF_ROUND_UP(width, 16);
	unsigned int mb_height = VDEF_ROUND_UP(height, 16);

	sps = calloc(1, sizeof(*sps));
	pps = calloc(1, sizeof(*pps));
	if (sps == NULL || pps == NULL) {
		res = -ENOMEM;
		goto out;
	}

	res = h264_ctx_new(&ctx);
	if (res < 0)
		goto out;

	sps->profile_idc = 77; /* Main profile */
	sps->constraint_set1_flag = 1;
	sps->level_idc = 40;
	sps->log2_max_frame_num_minus4 = 4;
	sps->pic_order_cnt_type = 0;
	sps->log2_max_pic_order_cnt_lsb_minus4 = 4;
	sps->max_num_ref_frames = 1;
	sps->pic_width_in_mbs_minus1 = mb_width - 1;
	sps->pic_height_in_map_units_minus1 = mb_height - 1;
	sps->frame_mbs_only_flag = 1;
	sps->direct_8x8_inference_flag = 1;
	if ((mb_width * 16 > width) || (mb_height * 16 > height)) {
		sps->frame_cropping_flag = 1;
		sps->frame_crop_right_offset = mb_width * 16 - width;
		sps->frame_crop_bottom_offset = (mb_height * 16 - height) / 2;
	}
	sps->vui_parameters_present_flag = 1;
	sps->vui.aspect_ratio_info_present_flag = 1;
	sps->vui.aspect_ratio_idc = 1; /* 1:1 */
	sps->vui.video_signal_type_present_flag = 1;
	sps->vui.video_format = 5; /* Unspecified */
	sps->vui.colour_description_present_flag = 1;
	sps->vui.colour_primaries = 1; /* BT.709 */
	sps->vui.transfer_characteristics = 1; /* BT.709 */
	sps->vui.matrix_coefficients = 1; /* BT.709 */
	sps->vui.timing_info_present_flag = 1;
	sps->vui.num_units_in_tick = 1;
	sps->vui.time_scale = 60;
	sps->vui.fixed_frame_rate_flag = 1;
	sps->vui.bitstream_restriction_flag = 1;
	sps->vui.motion_vectors_over_pic_boundaries_flag = 1;
	sps->vui.log2_max_mv_length_horizontal = 16;
	sps->vui.log2_max_mv_length_vertical = 16;
	sps->vui.max_dec_frame_buffering = 1;

	res = h264_ctx_set_sps(ctx, sps);
	if (res < 0)
		goto out;

	pps->entropy_coding_mode_flag = 1;

	res = h264_ctx_set_pps(ctx, pps);
	if (res < 0)
		goto out;

	res = h264_ctx_clear_nalu(ctx);
	if (res < 0)
		goto out;
	nh.forbidden_zero_bit = 0;
	nh.nal_ref_idc = 3;
	nh.nal_unit_type = H264_NALU_TYPE_SPS;
	res = h264_ctx_set_nalu_header(ctx, &nh);
	if (res < 0)
		goto out;

	sps_buf = malloc(256);
	if (sps_buf == NULL) {
		res = -ENOMEM;
		goto out;
	}
	memset(&bs, 0, sizeof(bs));
	h264_bs_init(&bs, sps_buf, 256, 1);
	res = h264_write_nalu(&bs, ctx);
	if (res < 0)
		goto out;
	*ret_sps_size = bs.off;

	res = h264_ctx_clear_nalu(ctx);
	if (res < 0)
		goto out;
	nh.nal_unit_type = H264_NALU_TYPE_PPS;
	res = h264_ctx_set_nalu_header(ctx, &nh);
	if (res < 0)
		goto out;

	pps_buf = malloc(256);
	if (pps_buf == NULL) {
		res = -ENOMEM;
		goto out;
	}
	memset(&bs, 0, sizeof(bs));
	h264_bs_init(&bs, pps_buf, 256, 1);
	res = h264_write_nalu(&bs, ctx);
	if (res < 0)
		goto out;
	*ret_pps_size = bs.off;

	*ret_sps = sps_buf;
	*ret_pps = pps_buf;
	sps_buf = NULL;
	pps_buf = NULL;
	res = 0;

out:
	free(sps);
	free(pps);
	free(sps_buf);
	free(pps_buf);
	if (ctx != NULL)
		h264_ctx_destroy(ctx);
	return res;
}


static void test_h264_writer_new_destroy(void)
{
	int res;
	uint8_t *sps = NULL, *pps = NULL;
	size_t sps_size = 0, pps_size = 0;
	struct h264_ctx *ctx = NULL;

	res = venc_test_build_h264_ps(
		640, 480, &sps, &sps_size, &pps, &pps_size);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h264_writer_new(sps, sps_size, pps, pps_size, &ctx);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(ctx);

	if (ctx != NULL)
		venc_h264_writer_destroy(ctx);
	free(sps);
	free(pps);
}


static void test_h264_writer_new_invalid_args(void)
{
	struct h264_ctx *ctx = NULL;
	uint8_t dummy[4] = {0};
	int res;

	res = venc_h264_writer_new(NULL, 4, dummy, 4, &ctx);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_h264_writer_new(dummy, 0, dummy, 4, &ctx);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_h264_writer_new(dummy, 4, NULL, 4, &ctx);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_h264_writer_new(dummy, 4, dummy, 0, &ctx);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_h264_writer_new(dummy, 4, dummy, 4, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_h264_patch_ps_injects_vui(void)
{
	int res;
	struct venc_encoder enc;
	uint8_t *sps = NULL, *pps = NULL;
	size_t sps_size = 0, pps_size = 0;
	struct h264_sps parsed = {0};

	memset(&enc, 0, sizeof(enc));

	res = venc_test_build_h264_ps(
		320, 240, &sps, &sps_size, &pps, &pps_size);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h264_writer_new(sps, sps_size, pps, pps_size, &enc.h264.ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* patch_ps() only checks sps/pps are non-NULL with a non-zero size; it
	 * re-derives the actual SPS to rewrite from h264.ctx, then realloc()s
	 * h264.sps in place, so these must be independently heap-allocated. */
	enc.h264.sps = malloc(sps_size);
	memcpy(enc.h264.sps, sps, sps_size);
	enc.h264.sps_size = sps_size;
	enc.h264.pps = malloc(pps_size);
	memcpy(enc.h264.pps, pps, pps_size);
	enc.h264.pps_size = pps_size;

	enc.config.encoding = VDEF_ENCODING_H264;
	enc.config.input.info.sar.width = 1;
	enc.config.input.info.sar.height = 1;
	enc.config.input.info.full_range = false;
	enc.config.input.info.color_primaries = VDEF_COLOR_PRIMARIES_BT709;
	enc.config.input.info.transfer_function = VDEF_TRANSFER_FUNCTION_BT709;
	enc.config.input.info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;
	enc.config.input.info.framerate.num = 30;
	enc.config.input.info.framerate.den = 1;

	res = venc_h264_patch_ps(&enc);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(enc.h264.sps);
	CU_ASSERT_TRUE(enc.h264.sps_size > 0);

	/* Re-parse the patched SPS: VUI parameters must now be present */
	res = h264_parse_sps(enc.h264.sps, enc.h264.sps_size, &parsed);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(parsed.vui_parameters_present_flag, 1);
	CU_ASSERT_EQUAL(parsed.vui.aspect_ratio_info_present_flag, 1);
	CU_ASSERT_EQUAL(parsed.vui.colour_description_present_flag, 1);

	venc_h264_writer_destroy(enc.h264.ctx);
	free(enc.h264.sps);
	free(enc.h264.pps);
	free(sps);
	free(pps);
}


static void test_h264_patch_ps_invalid_args(void)
{
	struct venc_encoder enc;
	int res;

	memset(&enc, 0, sizeof(enc));
	res = venc_h264_patch_ps(NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	/* enc.h264.ctx / sps / pps all NULL -> -EINVAL */
	res = venc_h264_patch_ps(&enc);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


/* SAR 7:13 has no pre-defined H.264 aspect_ratio_idc (IDC 1..16 cover only
 * standard ratios) so h264_sar_to_aspect_ratio_idc() returns 255
 * (Extended_SAR), exercising the sar_width/sar_height assignment at
 * venc_h264.c:211-212. */
static void test_h264_patch_ps_extended_sar(void)
{
	int res;
	struct venc_encoder enc;
	uint8_t *sps = NULL, *pps = NULL;
	size_t sps_size = 0, pps_size = 0;
	struct h264_sps parsed = {0};

	memset(&enc, 0, sizeof(enc));

	res = venc_test_build_h264_ps(
		320, 240, &sps, &sps_size, &pps, &pps_size);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h264_writer_new(sps, sps_size, pps, pps_size, &enc.h264.ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	enc.h264.sps = malloc(sps_size);
	memcpy(enc.h264.sps, sps, sps_size);
	enc.h264.sps_size = sps_size;
	enc.h264.pps = malloc(pps_size);
	memcpy(enc.h264.pps, pps, pps_size);
	enc.h264.pps_size = pps_size;

	enc.config.encoding = VDEF_ENCODING_H264;
	enc.config.input.info.sar.width = 7;
	enc.config.input.info.sar.height = 13;
	enc.config.input.info.color_primaries = VDEF_COLOR_PRIMARIES_BT709;
	enc.config.input.info.transfer_function = VDEF_TRANSFER_FUNCTION_BT709;
	enc.config.input.info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;
	enc.config.input.info.framerate.num = 30;
	enc.config.input.info.framerate.den = 1;

	res = venc_h264_patch_ps(&enc);
	CU_ASSERT_EQUAL(res, 0);

	res = h264_parse_sps(enc.h264.sps, enc.h264.sps_size, &parsed);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(parsed.vui.aspect_ratio_idc, 255);
	CU_ASSERT_EQUAL(parsed.vui.sar_width, 7);
	CU_ASSERT_EQUAL(parsed.vui.sar_height, 13);

	venc_h264_writer_destroy(enc.h264.ctx);
	free(enc.h264.sps);
	free(enc.h264.pps);
	free(sps);
	free(pps);
}


/* insert_pic_timing_sei=1 with framerate=0: set_h264_vui() takes the else
 * branch (venc_h264.c:244-246, ULOGW) but patch_ps() still returns 0. */
static void test_h264_patch_ps_pic_timing_without_framerate(void)
{
	int res;
	struct venc_encoder enc;
	uint8_t *sps = NULL, *pps = NULL;
	size_t sps_size = 0, pps_size = 0;
	struct h264_sps parsed = {0};

	memset(&enc, 0, sizeof(enc));

	res = venc_test_build_h264_ps(
		320, 240, &sps, &sps_size, &pps, &pps_size);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h264_writer_new(sps, sps_size, pps, pps_size, &enc.h264.ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	enc.h264.sps = malloc(sps_size);
	memcpy(enc.h264.sps, sps, sps_size);
	enc.h264.sps_size = sps_size;
	enc.h264.pps = malloc(pps_size);
	memcpy(enc.h264.pps, pps, pps_size);
	enc.h264.pps_size = pps_size;

	enc.config.encoding = VDEF_ENCODING_H264;
	enc.config.input.info.color_primaries = VDEF_COLOR_PRIMARIES_BT709;
	enc.config.input.info.transfer_function = VDEF_TRANSFER_FUNCTION_BT709;
	enc.config.input.info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;
	enc.config.h264.insert_pic_timing_sei = 1;
	/* framerate stays 0/0 (memset) -> vdef_frac_is_null() is true */

	res = venc_h264_patch_ps(&enc);
	CU_ASSERT_EQUAL(res, 0);

	/* pic_struct_present_flag must NOT be set: set_h264_vui() only writes
	 * it inside the framerate-present branch, which was skipped (ULOGW
	 * path). */
	res = h264_parse_sps(enc.h264.sps, enc.h264.sps_size, &parsed);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(parsed.vui.pic_struct_present_flag, 0);

	venc_h264_writer_destroy(enc.h264.ctx);
	free(enc.h264.sps);
	free(enc.h264.pps);
	free(sps);
	free(pps);
}


/*
 * venc_h264_aud_write() / venc_h264_sei_add_recovery_point() /
 * venc_h264_sei_write() / venc_h264_sei_add_user_data(): none of these
 * require the h264_ctx to have a parsed SPS/PPS set (verified by reading
 * their implementations - they only manipulate the ctx's NALU/SEI list), so
 * a plain h264_ctx_new() is enough; no need for the full writer/SPS fixture.
 */

static struct mbuf_coded_video_frame *
new_test_coded_frame_fmt(enum vdef_coded_data_format data_format)
{
	struct vdef_coded_frame frame_info;
	struct mbuf_coded_video_frame *frame = NULL;
	int res;

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format.encoding = VDEF_ENCODING_H264;
	frame_info.format.data_format = data_format;
	frame_info.type = VDEF_CODED_FRAME_TYPE_IDR;

	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	return frame;
}


static struct mbuf_coded_video_frame *new_test_coded_frame(void)
{
	return new_test_coded_frame_fmt(VDEF_CODED_DATA_FORMAT_BYTE_STREAM);
}


static void test_h264_aud_write(void)
{
	int res;
	struct h264_ctx *ctx = NULL;
	struct mbuf_coded_video_frame *frame = new_test_coded_frame();
	int nalu_count;

	res = h264_ctx_new(&ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h264_aud_write(ctx, frame);
	CU_ASSERT_EQUAL(res, 0);

	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);
	nalu_count = mbuf_coded_video_frame_get_nalu_count(frame);
	CU_ASSERT_EQUAL(nalu_count, 1);

	mbuf_coded_video_frame_unref(frame);
	venc_h264_writer_destroy(ctx);
}


static void test_h264_aud_write_invalid_args(void)
{
	struct h264_ctx *ctx = NULL;
	struct mbuf_coded_video_frame *frame = new_test_coded_frame();
	int res;

	res = h264_ctx_new(&ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h264_aud_write(NULL, frame);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_h264_aud_write(ctx, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	mbuf_coded_video_frame_unref(frame);
	venc_h264_writer_destroy(ctx);
}


static void test_h264_sei_recovery_point_and_write(void)
{
	int res;
	struct h264_ctx *ctx = NULL;
	struct mbuf_coded_video_frame *frame = new_test_coded_frame();
	int nalu_count;

	res = h264_ctx_new(&ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* No SEI added yet: sei_write() is a documented no-op (returns 0,
	 * adds no NALU) when the SEI count is zero. get_nalu_count() cannot
	 * be checked here to confirm the "no NALU added" part: it requires
	 * the frame to be finalized first (-EBUSY otherwise), and finalize()
	 * itself requires at least one NALU already present - so this only
	 * verifies the return value, not the NALU count, for the empty case. */
	res = venc_h264_sei_write(ctx, frame);
	CU_ASSERT_EQUAL(res, 0);

	res = venc_h264_sei_add_recovery_point(ctx, 5);
	CU_ASSERT_EQUAL(res, 0);

	res = venc_h264_sei_write(ctx, frame);
	CU_ASSERT_EQUAL(res, 0);

	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);
	nalu_count = mbuf_coded_video_frame_get_nalu_count(frame);
	CU_ASSERT_EQUAL(nalu_count, 1);

	mbuf_coded_video_frame_unref(frame);
	venc_h264_writer_destroy(ctx);
}


static void test_h264_sei_add_user_data(void)
{
	int res;
	struct h264_ctx *ctx = NULL;
	/* First 16 bytes are the UUID, remainder is the payload */
	uint8_t data[20] = {0};

	res = h264_ctx_new(&ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h264_sei_add_user_data(ctx, data, sizeof(data));
	CU_ASSERT_EQUAL(res, 0);

	/* Below the 16-byte UUID length -> -EINVAL */
	res = venc_h264_sei_add_user_data(ctx, data, 15);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = venc_h264_sei_add_user_data(NULL, data, sizeof(data));
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_h264_sei_add_user_data(ctx, NULL, sizeof(data));
	CU_ASSERT_EQUAL(res, -EINVAL);

	venc_h264_writer_destroy(ctx);
}


static void test_h264_format_convert(void)
{
	int res;
	struct mbuf_coded_video_frame *frame;
	struct mbuf_mem *mem = NULL;
	void *data = NULL;
	size_t capacity = 0;
	struct vdef_nalu nalu = {0};
	struct vdef_coded_format target = {
		.encoding = VDEF_ENCODING_H264,
		.data_format = VDEF_CODED_DATA_FORMAT_AVCC,
	};
	struct vdef_coded_format raw_nalu_target = {
		.encoding = VDEF_ENCODING_H264,
		.data_format = VDEF_CODED_DATA_FORMAT_RAW_NALU,
	};
	struct vdef_coded_format byte_stream = {
		.encoding = VDEF_ENCODING_H264,
		.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM,
	};

	frame = new_test_coded_frame(); /* BYTE_STREAM format */

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_mem_get_data(mem, &data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	/* Byte-stream start code, followed by a few bytes of fake payload */
	memset(data, 0, capacity);
	((uint8_t *)data)[3] = 1;

	nalu.size = capacity;
	nalu.h264.type = H264_NALU_TYPE_AUD;
	nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;

	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* No-op: same format */
	res = venc_h264_format_convert(frame, &byte_stream);
	CU_ASSERT_EQUAL(res, 0);

	/* Byte-stream -> AVCC: the first 4 bytes become a big-endian NALU
	 * size instead of a start code */
	res = venc_h264_format_convert(frame, &target);
	CU_ASSERT_EQUAL(res, 0);

	/* AVCC/byte-stream <-> RAW_NALU is unsupported */
	res = venc_h264_format_convert(frame, &raw_nalu_target);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	res = venc_h264_format_convert(NULL, &target);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_h264_format_convert(frame, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_h264_format_convert_avcc_to_byte_stream(void)
{
	int res;
	struct mbuf_coded_video_frame *frame;
	struct mbuf_mem *mem = NULL;
	void *data = NULL;
	size_t capacity = 0;
	struct vdef_nalu nalu = {0};
	struct vdef_coded_format byte_stream = {
		.encoding = VDEF_ENCODING_H264,
		.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM,
	};

	/* This time the frame's stored format is genuinely AVCC from
	 * creation (format_convert() never updates a frame's own stored
	 * format metadata, only its NALU bytes - so exercising the AVCC ->
	 * byte-stream branch specifically requires starting from a frame
	 * that already claims to be AVCC, not one converted in place by a
	 * prior call). */
	frame = new_test_coded_frame_fmt(VDEF_CODED_DATA_FORMAT_AVCC);

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_mem_get_data(mem, &data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memset(data, 0, capacity);

	nalu.size = capacity;
	nalu.h264.type = H264_NALU_TYPE_AUD;
	nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;

	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h264_format_convert(frame, &byte_stream);
	CU_ASSERT_EQUAL(res, 0);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_h264_format_convert_unsupported_pair(void)
{
	int res;
	struct mbuf_coded_video_frame *frame;
	struct mbuf_mem *mem = NULL;
	void *data = NULL;
	size_t capacity = 0;
	struct vdef_nalu nalu = {0};
	/* Neither RAW_NALU (rejected earlier) nor a supported
	 * byte-stream<->AVCC pair: falls through to the final -ENOSYS. */
	struct vdef_coded_format unknown_target = {
		.encoding = VDEF_ENCODING_H264,
		.data_format = VDEF_CODED_DATA_FORMAT_UNKNOWN,
	};

	frame = new_test_coded_frame_fmt(VDEF_CODED_DATA_FORMAT_BYTE_STREAM);

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_mem_get_data(mem, &data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memset(data, 0, capacity);

	nalu.size = capacity;
	nalu.h264.type = H264_NALU_TYPE_AUD;
	nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;

	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h264_format_convert(frame, &unknown_target);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


/* venc_h264_aud_write() AVCC branch (venc_h264.c:425): when the frame's
 * data_format is AVCC, the 4-byte length prefix is written instead of a
 * byte-stream start code. */
static void test_h264_aud_write_avcc(void)
{
	int res;
	struct h264_ctx *ctx = NULL;
	struct mbuf_coded_video_frame *frame =
		new_test_coded_frame_fmt(VDEF_CODED_DATA_FORMAT_AVCC);
	int nalu_count;

	res = h264_ctx_new(&ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h264_aud_write(ctx, frame);
	CU_ASSERT_EQUAL(res, 0);

	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);
	nalu_count = mbuf_coded_video_frame_get_nalu_count(frame);
	CU_ASSERT_EQUAL(nalu_count, 1);

	mbuf_coded_video_frame_unref(frame);
	venc_h264_writer_destroy(ctx);
}


/* venc_h264_sps_pps_copy() AVCC branch (venc_h264.c:542,599): when the frame's
 * data_format is AVCC, SPS and PPS NALUs are prefixed with a 4-byte big-endian
 * length field instead of a byte-stream start code. */
static void test_h264_sps_pps_copy_avcc(void)
{
	int res;
	uint8_t *sps = NULL, *pps = NULL;
	size_t sps_size = 0, pps_size = 0;
	struct h264_ctx *ctx = NULL;
	struct mbuf_coded_video_frame *frame;
	int nalu_count;

	res = venc_test_build_h264_ps(
		320, 240, &sps, &sps_size, &pps, &pps_size);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h264_writer_new(sps, sps_size, pps, pps_size, &ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	frame = new_test_coded_frame_fmt(VDEF_CODED_DATA_FORMAT_AVCC);

	res = venc_h264_sps_pps_copy(ctx, frame, sps, sps_size, pps, pps_size);
	CU_ASSERT_EQUAL(res, 0);

	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);
	nalu_count = mbuf_coded_video_frame_get_nalu_count(frame);
	CU_ASSERT_EQUAL(nalu_count, 2); /* SPS + PPS */

	mbuf_coded_video_frame_unref(frame);
	venc_h264_writer_destroy(ctx);
	free(sps);
	free(pps);
}


/* venc_h264_sei_write() AVCC branch (venc_h264.c:953): SEI NALU is prefixed
 * with a 4-byte big-endian length field when the frame is AVCC. */
static void test_h264_sei_write_avcc(void)
{
	int res;
	struct h264_ctx *ctx = NULL;
	struct mbuf_coded_video_frame *frame =
		new_test_coded_frame_fmt(VDEF_CODED_DATA_FORMAT_AVCC);
	int nalu_count;

	res = h264_ctx_new(&ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h264_sei_add_recovery_point(ctx, 0);
	CU_ASSERT_EQUAL(res, 0);

	res = venc_h264_sei_write(ctx, frame);
	CU_ASSERT_EQUAL(res, 0);

	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);
	nalu_count = mbuf_coded_video_frame_get_nalu_count(frame);
	CU_ASSERT_EQUAL(nalu_count, 1);

	mbuf_coded_video_frame_unref(frame);
	venc_h264_writer_destroy(ctx);
}


/* venc_h264_sei_add_picture_timing(): requires VUI with pic_struct_present_flag
 * and timing info set in the ctx's SPS. */
static void test_h264_sei_add_picture_timing(void)
{
	int res;
	struct h264_ctx *ctx = NULL;
	struct h264_sps sps = {0};
	struct venc_encoder enc;
	uint8_t *raw_sps = NULL, *raw_pps = NULL;
	size_t raw_sps_size = 0, raw_pps_size = 0;

	/* NULL guard */
	res = venc_h264_sei_add_picture_timing(NULL, 0);
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* -ENOENT: vui_parameters_present_flag == 0 */
	res = h264_ctx_new(&ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	sps.vui_parameters_present_flag = 0;
	res = h264_ctx_set_sps(ctx, &sps);
	CU_ASSERT_EQUAL(res, 0);
	res = venc_h264_sei_add_picture_timing(ctx, 0);
	CU_ASSERT_EQUAL(res, -ENOENT);
	h264_ctx_destroy(ctx);
	ctx = NULL;

	/* -ENOENT: VUI present but no pic_struct/HRD flags */
	res = h264_ctx_new(&ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memset(&sps, 0, sizeof(sps));
	sps.vui_parameters_present_flag = 1;
	/* pic_struct_present_flag = 0, nal_hrd = 0, vcl_hrd = 0 */
	res = h264_ctx_set_sps(ctx, &sps);
	CU_ASSERT_EQUAL(res, 0);
	res = venc_h264_sei_add_picture_timing(ctx, 0);
	CU_ASSERT_EQUAL(res, -ENOENT);
	h264_ctx_destroy(ctx);
	ctx = NULL;

	/* -EPROTO: pic_struct=1 but timing_info_present_flag=0 (time_scale=0)
	 */
	res = h264_ctx_new(&ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memset(&sps, 0, sizeof(sps));
	sps.vui_parameters_present_flag = 1;
	sps.vui.pic_struct_present_flag = 1;
	/* timing_info_present_flag stays 0 */
	res = h264_ctx_set_sps(ctx, &sps);
	CU_ASSERT_EQUAL(res, 0);
	res = venc_h264_sei_add_picture_timing(ctx, 0);
	CU_ASSERT_EQUAL(res, -EPROTO);
	h264_ctx_destroy(ctx);
	ctx = NULL;

	/* Success: patch_ps with insert_pic_timing_sei=1 injects pic_struct=1
	 * and timing info into the ctx's SPS. */
	memset(&enc, 0, sizeof(enc));
	res = venc_test_build_h264_ps(
		320, 240, &raw_sps, &raw_sps_size, &raw_pps, &raw_pps_size);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = venc_h264_writer_new(
		raw_sps, raw_sps_size, raw_pps, raw_pps_size, &enc.h264.ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	enc.h264.sps = malloc(raw_sps_size);
	memcpy(enc.h264.sps, raw_sps, raw_sps_size);
	enc.h264.sps_size = raw_sps_size;
	enc.h264.pps = malloc(raw_pps_size);
	memcpy(enc.h264.pps, raw_pps, raw_pps_size);
	enc.h264.pps_size = raw_pps_size;
	enc.config.encoding = VDEF_ENCODING_H264;
	enc.config.input.info.framerate.num = 30;
	enc.config.input.info.framerate.den = 1;
	enc.config.input.info.color_primaries = VDEF_COLOR_PRIMARIES_BT709;
	enc.config.input.info.transfer_function = VDEF_TRANSFER_FUNCTION_BT709;
	enc.config.input.info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;
	enc.config.h264.insert_pic_timing_sei = 1;
	res = venc_h264_patch_ps(&enc);
	CU_ASSERT_EQUAL(res, 0);
	/* After patch_ps: ctx's SPS has pic_struct_present_flag=1 + timing info
	 */
	res = venc_h264_sei_add_picture_timing(enc.h264.ctx, 1000000);
	CU_ASSERT_EQUAL(res, 0);
	venc_h264_writer_destroy(enc.h264.ctx);
	free(enc.h264.sps);
	free(enc.h264.pps);
	free(raw_sps);
	free(raw_pps);
}


CU_TestInfo g_venc_test_h264[] = {
	{FN("nalu-importance-non-slice-types"),
	 &test_h264_nalu_importance_non_slice_types},
	{FN("nalu-importance-slice-idr-and-i"),
	 &test_h264_nalu_importance_slice_idr_and_i},
	{FN("nalu-importance-slice-p-nri-levels"),
	 &test_h264_nalu_importance_slice_p_nri_levels},
	{FN("nalu-importance-slice-p-non-ref-and-default"),
	 &test_h264_nalu_importance_slice_p_non_ref_and_default},
	{FN("nalu-importance-sei"), &test_h264_nalu_importance_sei},
	{FN("nalu-importance-aud-filler-unknown"),
	 &test_h264_nalu_importance_aud_filler_unknown},
	{FN("writer-new-destroy"), &test_h264_writer_new_destroy},
	{FN("writer-new-invalid-args"), &test_h264_writer_new_invalid_args},
	{FN("patch-ps-injects-vui"), &test_h264_patch_ps_injects_vui},
	{FN("patch-ps-invalid-args"), &test_h264_patch_ps_invalid_args},
	{FN("patch-ps-extended-sar"), &test_h264_patch_ps_extended_sar},
	{FN("patch-ps-pic-timing-without-framerate"),
	 &test_h264_patch_ps_pic_timing_without_framerate},
	{FN("aud-write"), &test_h264_aud_write},
	{FN("aud-write-invalid-args"), &test_h264_aud_write_invalid_args},
	{FN("aud-write-avcc"), &test_h264_aud_write_avcc},
	{FN("sps-pps-copy-avcc"), &test_h264_sps_pps_copy_avcc},
	{FN("sei-recovery-point-and-write"),
	 &test_h264_sei_recovery_point_and_write},
	{FN("sei-write-avcc"), &test_h264_sei_write_avcc},
	{FN("sei-add-picture-timing"), &test_h264_sei_add_picture_timing},
	{FN("sei-add-user-data"), &test_h264_sei_add_user_data},
	{FN("format-convert"), &test_h264_format_convert},
	{FN("format-convert-avcc-to-byte-stream"),
	 &test_h264_format_convert_avcc_to_byte_stream},
	{FN("format-convert-unsupported-pair"),
	 &test_h264_format_convert_unsupported_pair},

	CU_TEST_INFO_NULL,
};
