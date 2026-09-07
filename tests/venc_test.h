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

#ifndef _VENC_TEST_H_
#define _VENC_TEST_H_

#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <CUnit/Automated.h>
#include <CUnit/Basic.h>
#include <CUnit/CUnit.h>

#include <video-encode/venc.h>
#include <video-encode/venc_h264.h>
#include <video-encode/venc_h265.h>
#include <video-encode/venc_internal.h>

#define FN(_name) (char *)(_name)

static inline bool implem_available(enum venc_encoder_implem implem)
{
	const enum vdef_encoding *encs = NULL;
	return venc_get_supported_encodings(implem, &encs) > 0;
}

/* Unlike implem_available(), which only checks that an implem is buildable
 * at all, this checks whether a specific encoding is actually usable for it
 * on this machine (e.g. ffmpeg's H.265 support depends on hevc_nvenc, which
 * requires an NVIDIA GPU/driver at runtime). */
static inline bool encoding_supported(enum venc_encoder_implem implem,
				      enum vdef_encoding encoding)
{
	const enum vdef_encoding *encs = NULL;
	int n = venc_get_supported_encodings(implem, &encs);
	for (int i = 0; i < n; i++) {
		if (encs[i] == encoding)
			return true;
	}
	return false;
}

#define VENC_TEST_MOCK_MAX_CAPTURED 32

/* Shared mock venc_cbs implementation: captures every output frame (with a
 * reference taken, so it survives past the callback) and records whether the
 * flush/stop callbacks fired. */
struct mock_venc_cbs_ctx {
	struct mbuf_coded_video_frame *captured[VENC_TEST_MOCK_MAX_CAPTURED];
	int frame_count;
	int status[VENC_TEST_MOCK_MAX_CAPTURED];
	bool flush_done;
	bool stop_done;
};

void mock_venc_cbs_ctx_reset(struct mock_venc_cbs_ctx *ctx);

void mock_venc_cbs_ctx_clear(struct mock_venc_cbs_ctx *ctx);

void mock_venc_frame_output_cb(struct venc_encoder *enc,
			       int status,
			       struct mbuf_coded_video_frame *frame,
			       void *userdata);

void mock_venc_flush_cb(struct venc_encoder *enc, void *userdata);

void mock_venc_stop_cb(struct venc_encoder *enc, void *userdata);

/* Build a valid, minimal H.264 SPS/PPS NAL unit pair for a given resolution,
 * usable as input to venc_h264_writer_new(). Returned buffers are malloc'd
 * and must be freed by the caller. */
int venc_test_build_h264_ps(unsigned int width,
			    unsigned int height,
			    uint8_t **ret_sps,
			    size_t *ret_sps_size,
			    uint8_t **ret_pps,
			    size_t *ret_pps_size);

extern CU_TestInfo g_venc_test_core[];
extern CU_TestInfo g_venc_test_h264[];
extern CU_TestInfo g_venc_test_h265[];
extern CU_TestInfo g_venc_test_config[];
extern CU_TestInfo g_venc_test_lifecycle[];

#endif /* !_VENC_TEST_H_ */
