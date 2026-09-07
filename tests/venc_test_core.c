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
 * enum <-> string helpers (core/src/venc_core.c). Pure functions, no
 * encoder/backend/loop needed.
 */

static void test_encoder_implem_str_round_trip(void)
{
	static const struct {
		enum venc_encoder_implem implem;
		const char *canonical; /* uppercase, matches to_str() output */
		const char *lower; /* lowercase, verifies from_str()
				      case-insensitivity */
	} cases[] = {
		{VENC_ENCODER_IMPLEM_X264, "X264", "x264"},
		{VENC_ENCODER_IMPLEM_X265, "X265", "x265"},
		{VENC_ENCODER_IMPLEM_HISI, "HISI", "hisi"},
		{VENC_ENCODER_IMPLEM_QCOM, "QCOM", "qcom"},
		{VENC_ENCODER_IMPLEM_QCOM_JPEG, "QCOM_JPEG", "qcom_jpeg"},
		{VENC_ENCODER_IMPLEM_MEDIACODEC, "MEDIACODEC", "mediacodec"},
		{VENC_ENCODER_IMPLEM_FAKEH264, "FAKEH264", "fakeh264"},
		{VENC_ENCODER_IMPLEM_FFMPEG, "FFMPEG", "ffmpeg"},
		{VENC_ENCODER_IMPLEM_VIDEOTOOLBOX,
		 "VIDEOTOOLBOX",
		 "videotoolbox"},
		{VENC_ENCODER_IMPLEM_TURBOJPEG, "TURBOJPEG", "turbojpeg"},
		{VENC_ENCODER_IMPLEM_PNG, "PNG", "png"},
	};

	for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		CU_ASSERT_STRING_EQUAL(
			venc_encoder_implem_to_str(cases[i].implem),
			cases[i].canonical);
		CU_ASSERT_EQUAL(
			venc_encoder_implem_from_str(cases[i].canonical),
			cases[i].implem);
		CU_ASSERT_EQUAL(venc_encoder_implem_from_str(cases[i].lower),
				cases[i].implem);
	}
}


static void test_encoder_implem_from_str_unknown_falls_back_to_auto(void)
{
	enum venc_encoder_implem implem;

	implem = venc_encoder_implem_from_str("not-an-implem");
	CU_ASSERT_EQUAL(implem, VENC_ENCODER_IMPLEM_AUTO);
	implem = venc_encoder_implem_from_str("");
	CU_ASSERT_EQUAL(implem, VENC_ENCODER_IMPLEM_AUTO);
}


static void test_encoder_implem_to_str_unknown_and_auto(void)
{
	const char *str;

	str = venc_encoder_implem_to_str(VENC_ENCODER_IMPLEM_AUTO);
	CU_ASSERT_STRING_EQUAL(str, "UNKNOWN");
	str = venc_encoder_implem_to_str(VENC_ENCODER_IMPLEM_MAX);
	CU_ASSERT_STRING_EQUAL(str, "UNKNOWN");
}


static void test_rate_control_str_round_trip(void)
{
	static const struct {
		enum venc_rate_control rc;
		const char *canonical; /* uppercase, matches to_str() output */
		const char *lower; /* lowercase, verifies from_str()
				      case-insensitivity */
	} cases[] = {
		{VENC_RATE_CONTROL_CBR, "CBR", "cbr"},
		{VENC_RATE_CONTROL_VBR, "VBR", "vbr"},
		{VENC_RATE_CONTROL_CQ, "CQ", "cq"},
	};

	for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		CU_ASSERT_STRING_EQUAL(venc_rate_control_to_str(cases[i].rc),
				       cases[i].canonical);
		CU_ASSERT_EQUAL(venc_rate_control_from_str(cases[i].canonical),
				cases[i].rc);
		CU_ASSERT_EQUAL(venc_rate_control_from_str(cases[i].lower),
				cases[i].rc);
	}
}


static void test_rate_control_from_str_unknown_falls_back_to_cbr(void)
{
	enum venc_rate_control rc;

	/* Unlike most other from_str() helpers, an unrecognized rate control
	 * string does NOT fall back to the enum's zero value via a generic
	 * "else" branch matching value 0 by coincidence: VENC_RATE_CONTROL_CBR
	 * is explicitly returned, and happens to also be the zero value. */
	rc = venc_rate_control_from_str("garbage");
	CU_ASSERT_EQUAL(rc, VENC_RATE_CONTROL_CBR);
}


static void test_entropy_coding_str_round_trip(void)
{
	enum venc_entropy_coding coding;
	const char *str;

	coding = venc_entropy_coding_from_str("CABAC");
	CU_ASSERT_EQUAL(coding, VENC_ENTROPY_CODING_CABAC);
	coding = venc_entropy_coding_from_str("cavlc");
	CU_ASSERT_EQUAL(coding, VENC_ENTROPY_CODING_CAVLC);

	str = venc_entropy_coding_to_str(VENC_ENTROPY_CODING_CABAC);
	CU_ASSERT_STRING_EQUAL(str, "CABAC");
	str = venc_entropy_coding_to_str(VENC_ENTROPY_CODING_CAVLC);
	CU_ASSERT_STRING_EQUAL(str, "CAVLC");
}


static void test_entropy_coding_from_str_unknown_falls_back_to_cabac(void)
{
	enum venc_entropy_coding coding;

	coding = venc_entropy_coding_from_str("garbage");
	CU_ASSERT_EQUAL(coding, VENC_ENTROPY_CODING_CABAC);
}


static void test_intra_refresh_str_round_trip(void)
{
	enum venc_intra_refresh ir;
	const char *str;

	ir = venc_intra_refresh_from_str("VERTICAL_SCAN");
	CU_ASSERT_EQUAL(ir, VENC_INTRA_REFRESH_VERTICAL_SCAN);
	ir = venc_intra_refresh_from_str("smart_scan");
	CU_ASSERT_EQUAL(ir, VENC_INTRA_REFRESH_SMART_SCAN);

	str = venc_intra_refresh_to_str(VENC_INTRA_REFRESH_NONE);
	CU_ASSERT_STRING_EQUAL(str, "NONE");
	str = venc_intra_refresh_to_str(VENC_INTRA_REFRESH_VERTICAL_SCAN);
	CU_ASSERT_STRING_EQUAL(str, "VERTICAL_SCAN");
	str = venc_intra_refresh_to_str(VENC_INTRA_REFRESH_SMART_SCAN);
	CU_ASSERT_STRING_EQUAL(str, "SMART_SCAN");
}


static void test_intra_refresh_from_str_unknown_falls_back_to_none(void)
{
	enum venc_intra_refresh ir;

	ir = venc_intra_refresh_from_str("garbage");
	CU_ASSERT_EQUAL(ir, VENC_INTRA_REFRESH_NONE);
}


/*
 * venc_count_unreleased_frames() (core/src/venc_core.c): pure arithmetic on
 * a hand-populated struct venc_encoder, no ops/loop/backend needed.
 */

static void test_count_unreleased_frames(void)
{
	struct venc_encoder enc;
	int count;

	memset(&enc, 0, sizeof(enc));
	atomic_init(&enc.counters.out, 0);
	atomic_init(&enc.counters.released, 0);
	count = venc_count_unreleased_frames(&enc);
	CU_ASSERT_EQUAL(count, 0);

	atomic_store(&enc.counters.out, 5);
	atomic_store(&enc.counters.released, 2);
	count = venc_count_unreleased_frames(&enc);
	CU_ASSERT_EQUAL(count, 3);

	atomic_store(&enc.counters.out, 5);
	atomic_store(&enc.counters.released, 5);
	count = venc_count_unreleased_frames(&enc);
	CU_ASSERT_EQUAL(count, 0);
}


/*
 * venc_config_get_specific() (core/src/venc_core.c): pure 3-way branch on a
 * hand-built struct venc_config, no encoder instance needed.
 */

static void test_config_get_specific_no_implem_cfg(void)
{
	struct venc_config config;
	struct venc_config_impl *specific;

	memset(&config, 0, sizeof(config));
	config.implem = VENC_ENCODER_IMPLEM_X264;
	config.implem_cfg = NULL;

	specific = venc_config_get_specific(&config, VENC_ENCODER_IMPLEM_X264);
	CU_ASSERT_PTR_NULL(specific);
}


static void test_config_get_specific_implem_mismatch(void)
{
	struct venc_config config;
	struct venc_config_impl impl_cfg = {.implem = VENC_ENCODER_IMPLEM_X264};
	struct venc_config_impl *specific;

	memset(&config, 0, sizeof(config));
	config.implem = VENC_ENCODER_IMPLEM_X264;
	config.implem_cfg = &impl_cfg;

	/* config.implem != the requested implem */
	specific = venc_config_get_specific(&config, VENC_ENCODER_IMPLEM_X265);
	CU_ASSERT_PTR_NULL(specific);
}


static void test_config_get_specific_implem_cfg_mismatch(void)
{
	struct venc_config config;
	struct venc_config_impl impl_cfg = {.implem = VENC_ENCODER_IMPLEM_X265};
	struct venc_config_impl *specific;

	memset(&config, 0, sizeof(config));
	config.implem = VENC_ENCODER_IMPLEM_X264;
	config.implem_cfg = &impl_cfg;

	/* config.implem_cfg->implem != config.implem */
	specific = venc_config_get_specific(&config, VENC_ENCODER_IMPLEM_X264);
	CU_ASSERT_PTR_NULL(specific);
}


static void test_config_get_specific_match(void)
{
	struct venc_config config;
	struct venc_config_impl impl_cfg = {.implem = VENC_ENCODER_IMPLEM_X264};
	struct venc_config_impl *specific;

	memset(&config, 0, sizeof(config));
	config.implem = VENC_ENCODER_IMPLEM_X264;
	config.implem_cfg = &impl_cfg;

	specific = venc_config_get_specific(&config, VENC_ENCODER_IMPLEM_X264);
	CU_ASSERT_PTR_EQUAL(specific, &impl_cfg);
}


/*
 * venc_copy_raw_frame_as_metadata() (core/src/venc_core.c): needs a real
 * mbuf_raw_video_frame + mbuf_mem, but no encoder/backend/loop at all.
 */

static struct mbuf_raw_video_frame *
new_test_raw_frame_fmt_ts(struct mbuf_mem *mem,
			  struct vdef_raw_format format,
			  uint64_t timestamp)
{
	struct vdef_raw_frame frame_info;
	struct mbuf_raw_video_frame *frame = NULL;
	unsigned int plane_count;
	int res;

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = format;
	frame_info.info.resolution.width = 64;
	frame_info.info.resolution.height = 64;
	frame_info.info.timestamp = timestamp;
	frame_info.info.timescale = 1000000;
	frame_info.plane_stride[0] = 64;
	frame_info.plane_stride[1] = 32;
	frame_info.plane_stride[2] = 32;

	res = mbuf_raw_video_frame_new(&frame_info, &frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Plane count/layout genuinely differs per format (e.g. NV12 has 2
	 * planes, I420 has 3) - only set as many planes as this format
	 * actually declares. Exact plane sizes don't matter here: these
	 * frames are only ever used to exercise venc_default_input_filter*()'s
	 * format/timestamp/resolution checks, never real pixel data. */
	plane_count = vdef_get_raw_frame_plane_count(&format);
	for (unsigned int i = 0; i < plane_count; i++) {
		res = mbuf_raw_video_frame_set_plane(frame, i, mem, 0, 64 * 64);
		CU_ASSERT_EQUAL_FATAL(res, 0);
	}

	res = mbuf_raw_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	return frame;
}


static struct mbuf_raw_video_frame *new_test_raw_frame(struct mbuf_mem *mem)
{
	return new_test_raw_frame_fmt_ts(mem, vdef_i420, 1000);
}


static void test_copy_raw_frame_as_metadata(void)
{
	int res;
	struct mbuf_mem *src_mem = NULL;
	struct mbuf_mem *dst_mem = NULL;
	struct mbuf_raw_video_frame *frame;
	struct mbuf_raw_video_frame *meta_frame = NULL;
	struct vdef_raw_frame meta_info;
	bool same_format;

	res = mbuf_mem_generic_new(64 * 64 + 2 * 32 * 32, &src_mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_test_raw_frame(src_mem);

	/* The destination mem is only ever referenced with offset=0/len=0 by
	 * venc_copy_raw_frame_as_metadata() (it copies frame info, metadata
	 * and ancillary data, but deliberately not the plane data itself),
	 * so a minimal allocation is enough. */
	res = mbuf_mem_generic_new(1, &dst_mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_copy_raw_frame_as_metadata(frame, dst_mem, &meta_frame);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(meta_frame);

	res = mbuf_raw_video_frame_get_frame_info(meta_frame, &meta_info);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(meta_info.info.timestamp, 1000u);
	same_format = vdef_raw_format_cmp(&meta_info.format, &vdef_i420);
	CU_ASSERT_TRUE(same_format);

	mbuf_raw_video_frame_unref(meta_frame);
	mbuf_raw_video_frame_unref(frame);
	mbuf_mem_unref(src_mem);
	mbuf_mem_unref(dst_mem);
}


static void test_copy_raw_frame_as_metadata_with_vmeta(void)
{
	int res;
	struct mbuf_mem *src_mem = NULL;
	struct mbuf_mem *dst_mem = NULL;
	struct mbuf_raw_video_frame *frame;
	struct mbuf_raw_video_frame *meta_frame = NULL;
	struct vmeta_frame *metadata = NULL;
	struct vmeta_frame *out_metadata = NULL;

	res = mbuf_mem_generic_new(64 * 64 + 2 * 32 * 32, &src_mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vmeta_frame_new(VMETA_FRAME_TYPE_V3, &metadata);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	frame = new_test_raw_frame(src_mem);
	res = mbuf_raw_video_frame_set_metadata(frame, metadata);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	/* frame holds its own ref; drop ours */
	vmeta_frame_unref(metadata);

	res = mbuf_mem_generic_new(1, &dst_mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = venc_copy_raw_frame_as_metadata(frame, dst_mem, &meta_frame);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(meta_frame);

	res = mbuf_raw_video_frame_get_metadata(meta_frame, &out_metadata);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(out_metadata);
	CU_ASSERT_EQUAL(out_metadata->type, VMETA_FRAME_TYPE_V3);
	vmeta_frame_unref(out_metadata);

	mbuf_raw_video_frame_unref(meta_frame);
	mbuf_raw_video_frame_unref(frame);
	mbuf_mem_unref(src_mem);
	mbuf_mem_unref(dst_mem);
}


static void test_copy_raw_frame_as_metadata_invalid_args(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame;
	struct mbuf_raw_video_frame *ret_obj = NULL;
	int res;

	res = mbuf_mem_generic_new(64 * 64 + 2 * 32 * 32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_test_raw_frame(mem);

	res = venc_copy_raw_frame_as_metadata(NULL, mem, &ret_obj);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_copy_raw_frame_as_metadata(frame, NULL, &ret_obj);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = venc_copy_raw_frame_as_metadata(frame, mem, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	mbuf_raw_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


/*
 * venc_default_input_filter() (core/src/venc_core.c): the public wrapper
 * around venc_default_input_filter_internal(); needs a struct venc_encoder
 * with a working ops->get_supported_input_formats(), which a minimal
 * self-contained fake ops table provides (no real backend needed).
 */

static int
fake_get_supported_input_formats(enum vdef_encoding encoding,
				 const struct vdef_raw_format **formats)
{
	(void)encoding;
	*formats = &vdef_i420;
	return 1;
}


static const struct venc_ops s_fake_filter_ops = {
	.get_supported_input_formats = fake_get_supported_input_formats,
};


static void test_default_input_filter_accepts_matching_frame(void)
{
	struct venc_encoder enc;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame;
	int res;
	bool accepted;

	memset(&enc, 0, sizeof(enc));
	enc.ops = &s_fake_filter_ops;
	enc.last_timestamp = UINT64_MAX;
	enc.config.input.info.resolution.width = 64;
	enc.config.input.info.resolution.height = 64;

	res = mbuf_mem_generic_new(64 * 64 + 2 * 32 * 32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_test_raw_frame(mem);

	accepted = venc_default_input_filter(frame, &enc);
	CU_ASSERT_TRUE(accepted);

	mbuf_raw_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_default_input_filter_rejects_mismatched_resolution(void)
{
	struct venc_encoder enc;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame;
	int res;
	bool accepted;

	memset(&enc, 0, sizeof(enc));
	enc.ops = &s_fake_filter_ops;
	enc.last_timestamp = UINT64_MAX;
	/* Encoder configured for a different resolution than the frame */
	enc.config.input.info.resolution.width = 320;
	enc.config.input.info.resolution.height = 240;

	res = mbuf_mem_generic_new(64 * 64 + 2 * 32 * 32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_test_raw_frame(mem);

	accepted = venc_default_input_filter(frame, &enc);
	CU_ASSERT_FALSE(accepted);

	mbuf_raw_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_default_input_filter_invalid_args(void)
{
	struct venc_encoder enc;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame;
	int res;
	bool accepted;

	memset(&enc, 0, sizeof(enc));
	enc.ops = &s_fake_filter_ops;

	res = mbuf_mem_generic_new(64 * 64 + 2 * 32 * 32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_test_raw_frame(mem);

	accepted = venc_default_input_filter(NULL, &enc);
	CU_ASSERT_FALSE(accepted);
	accepted = venc_default_input_filter(frame, NULL);
	CU_ASSERT_FALSE(accepted);

	mbuf_raw_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_default_input_filter_rejects_unsupported_format(void)
{
	struct venc_encoder enc;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame;
	int res;
	bool accepted;

	memset(&enc, 0, sizeof(enc));
	enc.ops = &s_fake_filter_ops;
	enc.last_timestamp = UINT64_MAX;
	enc.config.input.info.resolution.width = 64;
	enc.config.input.info.resolution.height = 64;

	res = mbuf_mem_generic_new(64 * 64 + 2 * 32 * 32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	/* s_fake_filter_ops always advertises vdef_i420 as the only
	 * supported format: a frame in a different format must be rejected */
	frame = new_test_raw_frame_fmt_ts(mem, vdef_nv12, 1000);

	accepted = venc_default_input_filter(frame, &enc);
	CU_ASSERT_FALSE(accepted);

	mbuf_raw_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_default_input_filter_rejects_non_monotonic_timestamp(void)
{
	struct venc_encoder enc;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *frame1;
	struct mbuf_raw_video_frame *frame2;
	int res;
	bool accepted;

	memset(&enc, 0, sizeof(enc));
	enc.ops = &s_fake_filter_ops;
	enc.last_timestamp = UINT64_MAX;
	enc.config.input.info.resolution.width = 64;
	enc.config.input.info.resolution.height = 64;

	res = mbuf_mem_generic_new(64 * 64 + 2 * 32 * 32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	frame1 = new_test_raw_frame_fmt_ts(mem, vdef_i420, 1000);
	accepted = venc_default_input_filter(frame1, &enc);
	CU_ASSERT_TRUE(accepted);

	/* Same timestamp as the previous accepted frame -> rejected */
	frame2 = new_test_raw_frame_fmt_ts(mem, vdef_i420, 1000);
	accepted = venc_default_input_filter(frame2, &enc);
	CU_ASSERT_FALSE(accepted);

	mbuf_raw_video_frame_unref(frame1);
	mbuf_raw_video_frame_unref(frame2);
	mbuf_mem_unref(mem);
}


static void test_rate_control_to_str_unknown(void)
{
	const char *str;

	str = venc_rate_control_to_str((enum venc_rate_control)999);
	CU_ASSERT_STRING_EQUAL(str, "UNKNOWN");
}


static void test_entropy_coding_to_str_unknown(void)
{
	const char *str;

	str = venc_entropy_coding_to_str((enum venc_entropy_coding)999);
	CU_ASSERT_STRING_EQUAL(str, "UNKNOWN");
}


static void test_intra_refresh_to_str_unknown(void)
{
	const char *str;

	str = venc_intra_refresh_to_str((enum venc_intra_refresh)999);
	CU_ASSERT_STRING_EQUAL(str, "UNKNOWN");
}


CU_TestInfo g_venc_test_core[] = {
	{FN("encoder-implem-str-round-trip"),
	 &test_encoder_implem_str_round_trip},
	{FN("encoder-implem-from-str-unknown-falls-back-to-auto"),
	 &test_encoder_implem_from_str_unknown_falls_back_to_auto},
	{FN("encoder-implem-to-str-unknown-and-auto"),
	 &test_encoder_implem_to_str_unknown_and_auto},
	{FN("rate-control-str-round-trip"), &test_rate_control_str_round_trip},
	{FN("rate-control-from-str-unknown-falls-back-to-cbr"),
	 &test_rate_control_from_str_unknown_falls_back_to_cbr},
	{FN("rate-control-to-str-unknown"), &test_rate_control_to_str_unknown},
	{FN("entropy-coding-str-round-trip"),
	 &test_entropy_coding_str_round_trip},
	{FN("entropy-coding-from-str-unknown-falls-back-to-cabac"),
	 &test_entropy_coding_from_str_unknown_falls_back_to_cabac},
	{FN("entropy-coding-to-str-unknown"),
	 &test_entropy_coding_to_str_unknown},
	{FN("intra-refresh-str-round-trip"),
	 &test_intra_refresh_str_round_trip},
	{FN("intra-refresh-from-str-unknown-falls-back-to-none"),
	 &test_intra_refresh_from_str_unknown_falls_back_to_none},
	{FN("intra-refresh-to-str-unknown"),
	 &test_intra_refresh_to_str_unknown},
	{FN("count-unreleased-frames"), &test_count_unreleased_frames},
	{FN("config-get-specific-no-implem-cfg"),
	 &test_config_get_specific_no_implem_cfg},
	{FN("config-get-specific-implem-mismatch"),
	 &test_config_get_specific_implem_mismatch},
	{FN("config-get-specific-implem-cfg-mismatch"),
	 &test_config_get_specific_implem_cfg_mismatch},
	{FN("config-get-specific-match"), &test_config_get_specific_match},
	{FN("copy-raw-frame-as-metadata"), &test_copy_raw_frame_as_metadata},
	{FN("copy-raw-frame-as-metadata-with-vmeta"),
	 &test_copy_raw_frame_as_metadata_with_vmeta},
	{FN("copy-raw-frame-as-metadata-invalid-args"),
	 &test_copy_raw_frame_as_metadata_invalid_args},
	{FN("default-input-filter-accepts-matching-frame"),
	 &test_default_input_filter_accepts_matching_frame},
	{FN("default-input-filter-rejects-mismatched-resolution"),
	 &test_default_input_filter_rejects_mismatched_resolution},
	{FN("default-input-filter-invalid-args"),
	 &test_default_input_filter_invalid_args},
	{FN("default-input-filter-rejects-unsupported-format"),
	 &test_default_input_filter_rejects_unsupported_format},
	{FN("default-input-filter-rejects-non-monotonic-timestamp"),
	 &test_default_input_filter_rejects_non_monotonic_timestamp},

	CU_TEST_INFO_NULL,
};
