/**
 * Copyright (c) 2017 Parrot Drones SAS
 */


static struct {
	CFStringRef kVTProfileLevel_H264_Baseline_AutoLevel;
	CFStringRef kVTProfileLevel_H264_Baseline_4_0;
	CFStringRef kVTProfileLevel_H264_Baseline_4_2;
	CFStringRef kVTProfileLevel_H264_Baseline_5_0;
	CFStringRef kVTProfileLevel_H264_Baseline_5_1;
	CFStringRef kVTProfileLevel_H264_Baseline_5_2;

	CFStringRef kVTProfileLevel_H264_Main_AutoLevel;
	CFStringRef kVTProfileLevel_H264_Main_4_2;
	CFStringRef kVTProfileLevel_H264_Main_5_1;
	CFStringRef kVTProfileLevel_H264_Main_5_2;

	CFStringRef kVTProfileLevel_H264_High_AutoLevel;
	CFStringRef kVTProfileLevel_H264_High_3_0;
	CFStringRef kVTProfileLevel_H264_High_3_1;
	CFStringRef kVTProfileLevel_H264_High_3_2;
	CFStringRef kVTProfileLevel_H264_High_4_0;
	CFStringRef kVTProfileLevel_H264_High_4_1;
	CFStringRef kVTProfileLevel_H264_High_4_2;
	CFStringRef kVTProfileLevel_H264_High_5_1;
	CFStringRef kVTProfileLevel_H264_High_5_2;

	CFStringRef kVTProfileLevel_H264_Extended_AutoLevel;
	CFStringRef kVTProfileLevel_H264_Extended_5_0;

	CFStringRef kVTProfileLevel_HEVC_Main_AutoLevel;
	CFStringRef kVTProfileLevel_HEVC_Main10_AutoLevel;

	CFStringRef kVTH264EntropyMode_CAVLC;
	CFStringRef kVTH264EntropyMode_CABAC;

	CFStringRef kCVImageBufferTransferFunction_SMPTE_ST_2084_PQ;
	CFStringRef kCVImageBufferTransferFunction_ITU_R_2100_HLG;
	CFStringRef kCVImageBufferTransferFunction_sRGB;
} s_compat_keys;


static struct {
	unsigned int p;
	unsigned int l;
	const CFStringRef *key;
} h264_profile_level_mapping[] = {
	/* Auto */
	{0, 0, NULL},
	/* Baseline */
	{H264_PROFILE_BASELINE,
	 0,
	 &s_compat_keys.kVTProfileLevel_H264_Baseline_AutoLevel},
	/* Main */
	{H264_PROFILE_MAIN,
	 0,
	 &s_compat_keys.kVTProfileLevel_H264_Main_AutoLevel},
	{H264_PROFILE_MAIN, 30, &kVTProfileLevel_H264_Main_3_0},
	{H264_PROFILE_MAIN, 31, &kVTProfileLevel_H264_Main_3_1},
	{H264_PROFILE_MAIN, 32, &kVTProfileLevel_H264_Main_3_2},
	{H264_PROFILE_MAIN, 40, &kVTProfileLevel_H264_Main_4_0},
	{H264_PROFILE_MAIN, 41, &kVTProfileLevel_H264_Main_4_1},
	{H264_PROFILE_MAIN, 42, &s_compat_keys.kVTProfileLevel_H264_Main_4_2},
	{H264_PROFILE_MAIN, 50, &kVTProfileLevel_H264_Main_5_0},
	{H264_PROFILE_MAIN, 51, &s_compat_keys.kVTProfileLevel_H264_Main_5_1},
	{H264_PROFILE_MAIN, 52, &s_compat_keys.kVTProfileLevel_H264_Main_5_2},
	/* High */
	{H264_PROFILE_HIGH,
	 0,
	 &s_compat_keys.kVTProfileLevel_H264_High_AutoLevel},
	/* Extended */
	{H264_PROFILE_EXTENDED,
	 0,
	 &s_compat_keys.kVTProfileLevel_H264_Extended_AutoLevel},
};


static inline int h264_profile_level_to_videotoolbox(unsigned int p,
						     unsigned int l,
						     CFStringRef *ret)
{
	for (size_t i = 0; i < SIZEOF_ARRAY(h264_profile_level_mapping); i++) {
		if (h264_profile_level_mapping[i].p == p &&
		    h264_profile_level_mapping[i].l == l) {
			if (h264_profile_level_mapping[i].key != NULL) {
				*ret = *h264_profile_level_mapping[i].key;
				return 0;
			}
		}
	}
	ULOGW("unsupported profile(%u)/level(%u)", p, l);
	return -EINVAL;
}


/* TODO */
#define H265_MAIN_PROFILE 1
#define H265_MAIN10_PROFILE 2


static struct {
	unsigned int p;
	const CFStringRef *key;
} h265_profile_level_mapping[] = {
	/* Auto */
	{0, NULL},
	/* Main */
	{H265_MAIN_PROFILE, &s_compat_keys.kVTProfileLevel_HEVC_Main_AutoLevel},
	/* Main10 */
	{H265_MAIN10_PROFILE,
	 &s_compat_keys.kVTProfileLevel_HEVC_Main10_AutoLevel},
};


static inline int h265_profile_level_to_videotoolbox(unsigned int p,
						     CFStringRef *ret)
{
	for (size_t i = 0; i < SIZEOF_ARRAY(h265_profile_level_mapping); i++) {
		if (h265_profile_level_mapping[i].p == p) {
			if (h265_profile_level_mapping[i].key != NULL) {
				*ret = *h265_profile_level_mapping[i].key;
				return 0;
			}
		}
	}
	ULOGW("unsupported profile(%u)", p);
	return -EINVAL;
}


static struct {
	enum venc_entropy_coding ec;
	const CFStringRef *key;
} entropy_coding_mapping[] = {
	{VENC_ENTROPY_CODING_CAVLC, &s_compat_keys.kVTH264EntropyMode_CAVLC},
	{VENC_ENTROPY_CODING_CABAC, &s_compat_keys.kVTH264EntropyMode_CABAC},
};


static inline int entropy_coding_to_videotoolbox(enum venc_entropy_coding ec,
						 CFStringRef *ret)
{
	for (size_t i = 0; i < SIZEOF_ARRAY(entropy_coding_mapping); i++) {
		if (entropy_coding_mapping[i].ec == ec) {
			if (entropy_coding_mapping[i].key != NULL) {
				*ret = *entropy_coding_mapping[i].key;
				return 0;
			}
		}
	}
	ULOGW("unsupported venc_entropy_coding(%u)", ec);
	return -EINVAL;
}


static struct {
	enum vdef_color_primaries cp;
	const CFStringRef *key;
} color_primaries_mapping[] = {
	{VDEF_COLOR_PRIMARIES_BT709, &kCVImageBufferColorPrimaries_ITU_R_709_2},
	{VDEF_COLOR_PRIMARIES_BT2020, &kCVImageBufferColorPrimaries_ITU_R_2020},
	{VDEF_COLOR_PRIMARIES_DCI_P3, &kCVImageBufferColorPrimaries_DCI_P3},
};


static inline int color_primaries_to_videotoolbox(enum vdef_color_primaries cp,
						  CFStringRef *ret)
{
	for (size_t i = 0; i < SIZEOF_ARRAY(color_primaries_mapping); i++) {
		if (color_primaries_mapping[i].cp == cp) {
			if (color_primaries_mapping[i].key != NULL) {
				*ret = *color_primaries_mapping[i].key;
				return 0;
			}
		}
	}
	ULOGW("unsupported vdef_color_primaries(%u)", cp);
	return -EINVAL;
}


static struct {
	enum vdef_transfer_function tf;
	const CFStringRef *key;
} transfer_function_mapping[] = {
	{VDEF_TRANSFER_FUNCTION_BT709,
	 &kCVImageBufferTransferFunction_ITU_R_709_2},
	{VDEF_TRANSFER_FUNCTION_BT2020,
	 &kCVImageBufferTransferFunction_ITU_R_2020},
	{VDEF_TRANSFER_FUNCTION_PQ,
	 &s_compat_keys.kCVImageBufferTransferFunction_SMPTE_ST_2084_PQ},
	{VDEF_TRANSFER_FUNCTION_HLG,
	 &s_compat_keys.kCVImageBufferTransferFunction_ITU_R_2100_HLG},
	{VDEF_TRANSFER_FUNCTION_SRGB,
	 &s_compat_keys.kCVImageBufferTransferFunction_sRGB},
};


static inline int
transfer_function_to_videotoolbox(enum vdef_transfer_function tf,
				  CFStringRef *ret)
{
	for (size_t i = 0; i < SIZEOF_ARRAY(transfer_function_mapping); i++) {
		if (transfer_function_mapping[i].tf == tf) {
			if (transfer_function_mapping[i].key != NULL) {
				*ret = *transfer_function_mapping[i].key;
				return 0;
			}
		}
	}
	ULOGW("unsupported vdef_transfer_function(%u)", tf);
	return -EINVAL;
}


static struct {
	enum vdef_matrix_coefs mc;
	const CFStringRef *key;
} matrix_coefs_mapping[] = {
	{VDEF_MATRIX_COEFS_BT709, &kCVImageBufferYCbCrMatrix_ITU_R_709_2},
	{VDEF_MATRIX_COEFS_BT2020_NON_CST,
	 &kCVImageBufferYCbCrMatrix_ITU_R_2020},
};


static inline int matrix_coefs_to_videotoolbox(enum vdef_matrix_coefs mc,
					       CFStringRef *ret)
{
	for (size_t i = 0; i < SIZEOF_ARRAY(matrix_coefs_mapping); i++) {
		if (matrix_coefs_mapping[i].mc == mc) {
			if (matrix_coefs_mapping[i].key != NULL) {
				*ret = *matrix_coefs_mapping[i].key;
				return 0;
			}
		}
	}
	ULOGW("unsupported vdef_matrix_coefs(%u)", mc);
	return -EINVAL;
}


static inline int
vdef_raw_format_to_videotoolbox(const struct vdef_raw_format *format,
				bool full_range,
				OSType *ret)
{
	if (vdef_raw_format_cmp(format, &vdef_i420)) {
		if (full_range)
			*ret = kCVPixelFormatType_420YpCbCr8PlanarFullRange;
		else
			*ret = kCVPixelFormatType_420YpCbCr8Planar;
	} else if (vdef_raw_format_cmp(format, &vdef_nv12)) {
		if (full_range)
			*ret = kCVPixelFormatType_420YpCbCr8BiPlanarFullRange;
		else
			*ret = kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange;
	} else {
		ULOGW("unsupported format: " VDEF_RAW_FORMAT_TO_STR_FMT,
		      VDEF_RAW_FORMAT_TO_STR_ARG(format));
		return -EINVAL;
	}
	return 0;
}
