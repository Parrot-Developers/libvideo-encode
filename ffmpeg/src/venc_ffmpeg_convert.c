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

#include "venc_ffmpeg_priv.h"


#define _MAKE_PROFILE(enc, be, bit_depth, profile, str)                        \
	{                                                                      \
		VDEF_ENCODING_##enc, VENC_FFMPEG_BACKEND_TYPE_##be, bit_depth, \
			profile, str                                           \
	}

#define MAKE_PROFILE(enc, be, profile, str)                                    \
	_MAKE_PROFILE(enc, be, 8, profile, str)
#define MAKE_PROFILE_10(enc, be, profile, str)                                 \
	_MAKE_PROFILE(enc, be, 10, profile, str)


static struct {
	enum vdef_encoding encoding;
	enum venc_ffmpeg_backend_type be;
	int profile;
	int bit_depth;
	const char *profile_str;
} s_profile_map[] = {
	/* OPENH264 */
	MAKE_PROFILE(H264,
		     OPENH264,
		     H264_PROFILE_BASELINE,
		     "constrained_baseline"),
	MAKE_PROFILE(H264, OPENH264, H264_PROFILE_MAIN, "main"),
	MAKE_PROFILE(H264, OPENH264, H264_PROFILE_HIGH, "high"),
	/* H264_NVENC */
	MAKE_PROFILE(H264, H264_NVENC, H264_PROFILE_BASELINE, "baseline"),
	MAKE_PROFILE(H264, H264_NVENC, H264_PROFILE_MAIN, "main"),
	MAKE_PROFILE(H264, H264_NVENC, H264_PROFILE_HIGH, "high"),
	MAKE_PROFILE(H264, H264_NVENC, H264_PROFILE_HIGH_444, "high444p"),
	/* HEVC_NVENC */
	MAKE_PROFILE(H265, HEVC_NVENC, 1, "main"),
	MAKE_PROFILE_10(H265, HEVC_NVENC, 0, "main10"),
};


const char *profile_to_av(enum vdef_encoding encoding,
			  enum venc_ffmpeg_backend_type be,
			  int bit_depth,
			  int profile)
{
	for (size_t i = 0; i < SIZEOF_ARRAY(s_profile_map); i++) {
		if ((s_profile_map[i].encoding != encoding) ||
		    (s_profile_map[i].be != be) ||
		    (s_profile_map[i].bit_depth != bit_depth) ||
		    (s_profile_map[i].profile != profile))
			continue;
		return s_profile_map[i].profile_str;
	}
	return NULL;
}


#define MAKE_LEVEL(enc, be, level, str)                                        \
	{                                                                      \
		VDEF_ENCODING_##enc, VENC_FFMPEG_BACKEND_TYPE_##be, level, str \
	}

static struct {
	enum vdef_encoding encoding;
	enum venc_ffmpeg_backend_type be;
	int level;
	const char *level_str;
} s_level_map[] = {
	/* OPENH264: unable to set  */
	/* H264_NVENC */
	MAKE_LEVEL(H264, H264_NVENC, 0, "auto"),
	MAKE_LEVEL(H264, H264_NVENC, 10, "1.0"),
	MAKE_LEVEL(H264, H264_NVENC, 11, "1.1"),
	MAKE_LEVEL(H264, H264_NVENC, 12, "1.2"),
	MAKE_LEVEL(H264, H264_NVENC, 13, "1.3"),
	MAKE_LEVEL(H264, H264_NVENC, 20, "2.0"),
	MAKE_LEVEL(H264, H264_NVENC, 21, "2.1"),
	MAKE_LEVEL(H264, H264_NVENC, 22, "2.2"),
	MAKE_LEVEL(H264, H264_NVENC, 30, "3.0"),
	MAKE_LEVEL(H264, H264_NVENC, 31, "3.1"),
	MAKE_LEVEL(H264, H264_NVENC, 32, "3.2"),
	MAKE_LEVEL(H264, H264_NVENC, 40, "4.0"),
	MAKE_LEVEL(H264, H264_NVENC, 41, "4.1"),
	MAKE_LEVEL(H264, H264_NVENC, 42, "4.2"),
	MAKE_LEVEL(H264, H264_NVENC, 50, "5.0"),
	MAKE_LEVEL(H264, H264_NVENC, 51, "5.1"),
	MAKE_LEVEL(H264, H264_NVENC, 52, "5.2"),
	MAKE_LEVEL(H264, H264_NVENC, 60, "6.0"),
	MAKE_LEVEL(H264, H264_NVENC, 61, "6.1"),
	MAKE_LEVEL(H264, H264_NVENC, 62, "6.2"),
	/* HEVC_NVENC */
	MAKE_LEVEL(H265, HEVC_NVENC, 0, "auto"),
	MAKE_LEVEL(H265, HEVC_NVENC, 10, "1.0"),
	MAKE_LEVEL(H265, HEVC_NVENC, 20, "2.0"),
	MAKE_LEVEL(H265, HEVC_NVENC, 21, "2.1"),
	MAKE_LEVEL(H265, HEVC_NVENC, 30, "3.0"),
	MAKE_LEVEL(H265, HEVC_NVENC, 31, "3.1"),
	MAKE_LEVEL(H265, HEVC_NVENC, 40, "4.0"),
	MAKE_LEVEL(H265, HEVC_NVENC, 41, "4.1"),
	MAKE_LEVEL(H265, HEVC_NVENC, 50, "5.0"),
	MAKE_LEVEL(H265, HEVC_NVENC, 51, "5.1"),
	MAKE_LEVEL(H265, HEVC_NVENC, 52, "5.2"),
	MAKE_LEVEL(H265, HEVC_NVENC, 60, "6.0"),
	MAKE_LEVEL(H265, HEVC_NVENC, 61, "6.1"),
	MAKE_LEVEL(H265, HEVC_NVENC, 62, "6.2"),
};


const char *level_to_av(enum vdef_encoding encoding,
			enum venc_ffmpeg_backend_type be,
			int level)
{
	for (size_t i = 0; i < SIZEOF_ARRAY(s_level_map); i++) {
		if ((s_level_map[i].encoding != encoding) ||
		    (s_level_map[i].be != be) ||
		    (s_level_map[i].level != level))
			continue;
		return s_level_map[i].level_str;
	}
	return NULL;
}


#define _MAKE_RATE_CONTROL(be, rc, key, ld, hq, val)                           \
	{                                                                      \
		VENC_FFMPEG_BACKEND_TYPE_##be, VENC_RATE_CONTROL_##rc, key,    \
			ld, hq, val                                            \
	}
#define MAKE_RATE_CONTROL(be, rc, key, val)                                    \
	_MAKE_RATE_CONTROL(be, rc, false, false, key, val)
#define MAKE_RATE_CONTROL_HQ(be, rc, key, val)                                 \
	_MAKE_RATE_CONTROL(be, rc, false, true, key, val)
#define MAKE_RATE_CONTROL_LD_HQ(be, rc, key, val)                              \
	_MAKE_RATE_CONTROL(be, rc, true, true, key, val)


static struct {
	enum venc_ffmpeg_backend_type be;
	enum venc_rate_control rc;
	bool low_delay;
	bool high_quality;
	const char *key_str;
	const char *val_str;
} s_rc_map[] = {
	/* OPENH264 */
	MAKE_RATE_CONTROL(OPENH264, CBR, "rc_mode", "timestamp"),
	MAKE_RATE_CONTROL(OPENH264, VBR, "rc_mode", "timestamp"),
	MAKE_RATE_CONTROL(OPENH264, CQ, "rc_mode", "quality"),
	/* H264_NVENC */
	MAKE_RATE_CONTROL(H264_NVENC, CBR, "rc", "cbr"),
	MAKE_RATE_CONTROL(H264_NVENC, VBR, "rc", "vbr"),
	MAKE_RATE_CONTROL(H264_NVENC, CQ, "rc", "constqp"),
	MAKE_RATE_CONTROL_HQ(H264_NVENC, CBR, "rc", "cbr_hq"),
	MAKE_RATE_CONTROL_HQ(H264_NVENC, VBR, "rc", "vbr_qh"),
	MAKE_RATE_CONTROL_LD_HQ(H264_NVENC, CBR, "rc", "cbr_ld_hq"),
	/* HEVC_NVENC */
	MAKE_RATE_CONTROL(HEVC_NVENC, CBR, "rc", "cbr"),
	MAKE_RATE_CONTROL(HEVC_NVENC, VBR, "rc", "vbr"),
	MAKE_RATE_CONTROL(HEVC_NVENC, CQ, "rc", "constqp"),
	MAKE_RATE_CONTROL_HQ(HEVC_NVENC, CBR, "rc", "cbr_hq"),
	MAKE_RATE_CONTROL_HQ(HEVC_NVENC, VBR, "rc", "vbr_qh"),
	MAKE_RATE_CONTROL_LD_HQ(HEVC_NVENC, CBR, "rc", "cbr_ld_hq"),
};


int rate_control_to_av(enum venc_ffmpeg_backend_type be,
		       enum venc_rate_control rc,
		       bool low_delay,
		       bool high_quality,
		       const char **key_str,
		       const char **val_str)
{
	ULOG_ERRNO_RETURN_ERR_IF(key_str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(val_str == NULL, EINVAL);

	for (size_t i = 0; i < 2; i++) {
		for (size_t j = 0; j < SIZEOF_ARRAY(s_rc_map); j++) {
			if ((s_rc_map[j].be != be) || (s_rc_map[j].rc != rc))
				continue;
			/* Search with ld and hq on first pass */
			if ((i == 0) &&
			    ((low_delay != s_rc_map[j].low_delay) ||
			     (high_quality != s_rc_map[j].high_quality)))
				continue;

			*key_str = s_rc_map[j].key_str;
			*val_str = s_rc_map[j].val_str;
			return 0;
		}
	}
	return -ENOENT;
}


#define MAKE_CODING(be, coding, val)                                           \
	{                                                                      \
		VENC_FFMPEG_BACKEND_TYPE_##be, VENC_ENTROPY_CODING_##coding,   \
			val                                                    \
	}


static struct {
	enum venc_ffmpeg_backend_type be;
	enum venc_entropy_coding coding;
	const char *val_str;
} s_coding_map[] = {
	/* OPENH264 */
	MAKE_CODING(OPENH264, CABAC, "cabac"),
	MAKE_CODING(OPENH264, CAVLC, "cavlc"),
	/* H264_NVENC */
	MAKE_CODING(H264_NVENC, CABAC, "cabac"),
	MAKE_CODING(H264_NVENC, CAVLC, "cavlc"),
	/* HEVC_NVENC: unsupported */
};


const char *entroy_coding_to_av(enum venc_ffmpeg_backend_type be,
				enum venc_entropy_coding coding)
{
	for (size_t i = 0; i < SIZEOF_ARRAY(s_coding_map); i++) {
		if ((s_coding_map[i].be != be) ||
		    (s_coding_map[i].coding != coding))
			continue;
		return s_coding_map[i].val_str;
	}
	return NULL;
}
