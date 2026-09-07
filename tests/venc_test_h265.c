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
 * venc_h265_get_nalu_importance() (core/src/venc_h265.c): pure enum switch,
 * no dependencies. Exhaustive coverage of every branch.
 *
 * Note: unlike venc_h265_writer_new()/venc_h265_patch_ps(), which require a
 * full, valid VPS+SPS+PPS triplet (HEVC parameter sets are considerably more
 * complex to hand-synthesize than H.264's, with nested profile-tier-level and
 * sub-layer structures), this suite does not attempt to build one. Those two
 * functions are left uncovered here; see the top-level CLAUDE.md for the
 * H.264 equivalent, which is covered in venc_test_h264.c.
 */

static void test_h265_nalu_importance_zero_importance_types(void)
{
	uint32_t importance;

	importance = venc_h265_get_nalu_importance(H265_NALU_TYPE_VPS_NUT,
						   VDEF_CODED_FRAME_TYPE_P);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h265_get_nalu_importance(H265_NALU_TYPE_SPS_NUT,
						   VDEF_CODED_FRAME_TYPE_P);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h265_get_nalu_importance(H265_NALU_TYPE_PPS_NUT,
						   VDEF_CODED_FRAME_TYPE_P);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h265_get_nalu_importance(H265_NALU_TYPE_AUD_NUT,
						   VDEF_CODED_FRAME_TYPE_P);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h265_get_nalu_importance(H265_NALU_TYPE_EOS_NUT,
						   VDEF_CODED_FRAME_TYPE_P);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h265_get_nalu_importance(H265_NALU_TYPE_EOB_NUT,
						   VDEF_CODED_FRAME_TYPE_P);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h265_get_nalu_importance(H265_NALU_TYPE_FD_NUT,
						   VDEF_CODED_FRAME_TYPE_P);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h265_get_nalu_importance(H265_NALU_TYPE_IDR_W_RADL,
						   VDEF_CODED_FRAME_TYPE_P);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h265_get_nalu_importance(H265_NALU_TYPE_IDR_N_LP,
						   VDEF_CODED_FRAME_TYPE_P);
	CU_ASSERT_EQUAL(importance, 0);
}


static void test_h265_nalu_importance_trail(void)
{
	uint32_t importance;

	importance = venc_h265_get_nalu_importance(H265_NALU_TYPE_TRAIL_N,
						   VDEF_CODED_FRAME_TYPE_P);
	CU_ASSERT_EQUAL(importance, 1);
	importance = venc_h265_get_nalu_importance(H265_NALU_TYPE_TRAIL_R,
						   VDEF_CODED_FRAME_TYPE_P);
	CU_ASSERT_EQUAL(importance, 1);
}


static void test_h265_nalu_importance_sei(void)
{
	uint32_t importance;

	importance = venc_h265_get_nalu_importance(
		H265_NALU_TYPE_PREFIX_SEI_NUT, VDEF_CODED_FRAME_TYPE_IDR);
	CU_ASSERT_EQUAL(importance, 0);
	importance =
		venc_h265_get_nalu_importance(H265_NALU_TYPE_SUFFIX_SEI_NUT,
					      VDEF_CODED_FRAME_TYPE_P_IR_START);
	CU_ASSERT_EQUAL(importance, 0);
	importance = venc_h265_get_nalu_importance(
		H265_NALU_TYPE_PREFIX_SEI_NUT, VDEF_CODED_FRAME_TYPE_P);
	CU_ASSERT_EQUAL(importance, 1);
	importance = venc_h265_get_nalu_importance(
		H265_NALU_TYPE_SUFFIX_SEI_NUT, VDEF_CODED_FRAME_TYPE_P_NON_REF);
	CU_ASSERT_EQUAL(importance, 2);
	importance = venc_h265_get_nalu_importance(
		H265_NALU_TYPE_PREFIX_SEI_NUT, VDEF_CODED_FRAME_TYPE_UNKNOWN);
	CU_ASSERT_EQUAL(importance, 1);
}


static void test_h265_nalu_importance_unknown(void)
{
	uint32_t importance;

	importance = venc_h265_get_nalu_importance(H265_NALU_TYPE_UNKNOWN,
						   VDEF_CODED_FRAME_TYPE_P);
	CU_ASSERT_EQUAL(importance, UINT32_MAX);
}


/*
 * venc_h265_aud_write() / venc_h265_sei_add_recovery_point() /
 * venc_h265_sei_write() / venc_h265_sei_add_user_data(): like their H.264
 * counterparts, none of these need a parsed VPS/SPS/PPS - a plain
 * h265_ctx_new() is enough, avoiding the need to hand-synthesize a full
 * HEVC parameter set triplet just for this coverage.
 */

static struct mbuf_coded_video_frame *
new_test_coded_frame_h265_fmt(enum vdef_coded_data_format data_format)
{
	struct vdef_coded_frame frame_info;
	struct mbuf_coded_video_frame *frame = NULL;
	int res;

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format.encoding = VDEF_ENCODING_H265;
	frame_info.format.data_format = data_format;
	frame_info.type = VDEF_CODED_FRAME_TYPE_IDR;

	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	return frame;
}


static struct mbuf_coded_video_frame *new_test_coded_frame_h265(void)
{
	return new_test_coded_frame_h265_fmt(
		VDEF_CODED_DATA_FORMAT_BYTE_STREAM);
}


static void test_h265_aud_write(void)
{
	int res;
	struct h265_ctx *ctx = NULL;
	struct mbuf_coded_video_frame *frame = new_test_coded_frame_h265();
	int nalu_count;

	res = h265_ctx_new(&ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h265_aud_write(ctx, frame);
	CU_ASSERT_EQUAL(res, 0);

	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);
	nalu_count = mbuf_coded_video_frame_get_nalu_count(frame);
	CU_ASSERT_EQUAL(nalu_count, 1);

	mbuf_coded_video_frame_unref(frame);
	venc_h265_writer_destroy(ctx);
}


static void test_h265_aud_write_invalid_args(void)
{
	struct h265_ctx *ctx = NULL;
	struct mbuf_coded_video_frame *frame = new_test_coded_frame_h265();
	int res;

	res = h265_ctx_new(&ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h265_aud_write(NULL, frame);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_h265_aud_write(ctx, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	mbuf_coded_video_frame_unref(frame);
	venc_h265_writer_destroy(ctx);
}


static void test_h265_sei_recovery_point_and_write(void)
{
	int res;
	struct h265_ctx *ctx = NULL;
	struct mbuf_coded_video_frame *frame = new_test_coded_frame_h265();
	int nalu_count;

	res = h265_ctx_new(&ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* No SEI added yet: sei_write() is a documented no-op (returns 0,
	 * adds no NALU) when the SEI count is zero. get_nalu_count() cannot
	 * be checked here to confirm the "no NALU added" part: it requires
	 * the frame to be finalized first (-EBUSY otherwise), and finalize()
	 * itself requires at least one NALU already present - so this only
	 * verifies the return value, not the NALU count, for the empty case. */
	res = venc_h265_sei_write(ctx, frame);
	CU_ASSERT_EQUAL(res, 0);

	res = venc_h265_sei_add_recovery_point(ctx, 5);
	CU_ASSERT_EQUAL(res, 0);

	res = venc_h265_sei_write(ctx, frame);
	CU_ASSERT_EQUAL(res, 0);

	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL(res, 0);
	nalu_count = mbuf_coded_video_frame_get_nalu_count(frame);
	CU_ASSERT_EQUAL(nalu_count, 1);

	mbuf_coded_video_frame_unref(frame);
	venc_h265_writer_destroy(ctx);
}


static void test_h265_sei_add_user_data(void)
{
	int res;
	struct h265_ctx *ctx = NULL;
	uint8_t data[20] = {0};

	res = h265_ctx_new(&ctx);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h265_sei_add_user_data(ctx, data, sizeof(data));
	CU_ASSERT_EQUAL(res, 0);

	res = venc_h265_sei_add_user_data(ctx, data, 15);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = venc_h265_sei_add_user_data(NULL, data, sizeof(data));
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_h265_sei_add_user_data(ctx, NULL, sizeof(data));
	CU_ASSERT_EQUAL(res, -EINVAL);

	venc_h265_writer_destroy(ctx);
}


static void test_h265_format_convert(void)
{
	int res;
	struct mbuf_coded_video_frame *frame;
	struct mbuf_mem *mem = NULL;
	void *data = NULL;
	size_t capacity = 0;
	struct vdef_nalu nalu = {0};
	struct vdef_coded_format target = {
		.encoding = VDEF_ENCODING_H265,
		.data_format = VDEF_CODED_DATA_FORMAT_AVCC,
	};
	struct vdef_coded_format raw_nalu_target = {
		.encoding = VDEF_ENCODING_H265,
		.data_format = VDEF_CODED_DATA_FORMAT_RAW_NALU,
	};
	struct vdef_coded_format byte_stream = {
		.encoding = VDEF_ENCODING_H265,
		.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM,
	};

	frame = new_test_coded_frame_h265(); /* BYTE_STREAM format */

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_mem_get_data(mem, &data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memset(data, 0, capacity);
	((uint8_t *)data)[3] = 1;

	nalu.size = capacity;
	nalu.h265.type = H265_NALU_TYPE_AUD_NUT;

	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h265_format_convert(frame, &byte_stream);
	CU_ASSERT_EQUAL(res, 0);

	res = venc_h265_format_convert(frame, &target);
	CU_ASSERT_EQUAL(res, 0);

	res = venc_h265_format_convert(frame, &raw_nalu_target);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	res = venc_h265_format_convert(NULL, &target);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_h265_format_convert(frame, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_h265_format_convert_avcc_to_byte_stream(void)
{
	int res;
	struct mbuf_coded_video_frame *frame;
	struct mbuf_mem *mem = NULL;
	void *data = NULL;
	size_t capacity = 0;
	struct vdef_nalu nalu = {0};
	struct vdef_coded_format byte_stream = {
		.encoding = VDEF_ENCODING_H265,
		.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM,
	};

	frame = new_test_coded_frame_h265_fmt(VDEF_CODED_DATA_FORMAT_AVCC);

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_mem_get_data(mem, &data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memset(data, 0, capacity);

	nalu.size = capacity;
	nalu.h265.type = H265_NALU_TYPE_AUD_NUT;

	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h265_format_convert(frame, &byte_stream);
	CU_ASSERT_EQUAL(res, 0);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_h265_format_convert_unsupported_pair(void)
{
	int res;
	struct mbuf_coded_video_frame *frame;
	struct mbuf_mem *mem = NULL;
	void *data = NULL;
	size_t capacity = 0;
	struct vdef_nalu nalu = {0};
	struct vdef_coded_format unknown_target = {
		.encoding = VDEF_ENCODING_H265,
		.data_format = VDEF_CODED_DATA_FORMAT_UNKNOWN,
	};

	frame = new_test_coded_frame_h265_fmt(
		VDEF_CODED_DATA_FORMAT_BYTE_STREAM);

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_mem_get_data(mem, &data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memset(data, 0, capacity);

	nalu.size = capacity;
	nalu.h265.type = H265_NALU_TYPE_AUD_NUT;

	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_h265_format_convert(frame, &unknown_target);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


CU_TestInfo g_venc_test_h265[] = {
	{FN("nalu-importance-zero-importance-types"),
	 &test_h265_nalu_importance_zero_importance_types},
	{FN("nalu-importance-trail"), &test_h265_nalu_importance_trail},
	{FN("nalu-importance-sei"), &test_h265_nalu_importance_sei},
	{FN("nalu-importance-unknown"), &test_h265_nalu_importance_unknown},
	{FN("aud-write"), &test_h265_aud_write},
	{FN("aud-write-invalid-args"), &test_h265_aud_write_invalid_args},
	{FN("sei-recovery-point-and-write"),
	 &test_h265_sei_recovery_point_and_write},
	{FN("sei-add-user-data"), &test_h265_sei_add_user_data},
	{FN("format-convert"), &test_h265_format_convert},
	{FN("format-convert-avcc-to-byte-stream"),
	 &test_h265_format_convert_avcc_to_byte_stream},
	{FN("format-convert-unsupported-pair"),
	 &test_h265_format_convert_unsupported_pair},

	CU_TEST_INFO_NULL,
};
