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

#ifndef _VENC_MEDIACODEC_PRIV_H_
#define _VENC_MEDIACODEC_PRIV_H_

#include <dlfcn.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <android/native_window.h>
#include <futils/random.h>
#include <futils/timetools.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media/NdkMediaCodec.h>
#include <media/NdkMediaFormat.h>
#include <video-encode/venc_core.h>
#include <video-encode/venc_h264.h>
#include <video-encode/venc_h265.h>
#include <video-encode/venc_internal.h>
#include <video-encode/venc_mediacodec.h>


enum state {
	RUNNING,
	WAITING_FOR_FLUSH,
	WAITING_FOR_STOP,
};


/* See
 * https://developer.android.com/reference/android/media/MediaCodecInfo.CodecCapabilities.html
 * for reference. */
enum color_format {
	YUV420_PLANAR = 0x00000013,
	YUV420_PACKED_PLANAR = 0x00000014,
	YUV420_SEMIPLANAR = 0x00000015,
	YUV420_PACKED_SEMIPLANAR = 0x00000027,
	TI_YUV420_PACKED_SEMIPLANAR = 0x7F000100,
	COLOR_FORMAT_SURFACE = 0x7F000789,
	QCOM_YUV420_SEMIPLANAR = 0x7FA30C00,
	QCOM_YUV420_PACKED_SEMIPLANAR64X32_TILE2_M8KA = 0x7FA30C03,
	QCOM_YUV420_SEMIPLANAR32_M = 0x7FA30C04,
};


/* See
 * https://developer.android.com/reference/android/media/MediaCodecInfo.CodecProfileLevel
 * for reference. */
enum mc_h264_profile {
	MC_H264_PROFILE_BASELINE = 0x1,
	MC_H264_PROFILE_CONSTRAINED_BASELINE = 0x10000,
	MC_H264_PROFILE_CONSTRAINED_HIGH = 0x80000,
	MC_H264_PROFILE_EXTENDED = 0x4,
	MC_H264_PROFILE_HIGH = 0x8,
	MC_H264_PROFILE_HIGH10 = 0x10,
	MC_H264_PROFILE_HIGH422 = 0x20,
	MC_H264_PROFILE_HIGH444 = 0x40,
	MC_H264_PROFILE_MAIN = 0x2,
};


/* See
 * https://developer.android.com/reference/android/media/MediaCodecInfo.CodecProfileLevel
 * for reference. */
enum mc_h264_level {
	MC_H264_LEVEL_1 = 0x1,
	MC_H264_LEVEL_11 = 0x4,
	MC_H264_LEVEL_12 = 0x8,
	MC_H264_LEVEL_13 = 0x10,
	MC_H264_LEVEL_1b = 0x2,
	MC_H264_LEVEL_2 = 0x20,
	MC_H264_LEVEL_21 = 0x40,
	MC_H264_LEVEL_22 = 0x80,
	MC_H264_LEVEL_3 = 0x100,
	MC_H264_LEVEL_31 = 0x200,
	MC_H264_LEVEL_32 = 0x400,
	MC_H264_LEVEL_4 = 0x800,
	MC_H264_LEVEL_41 = 0x1000,
	MC_H264_LEVEL_42 = 0x2000,
	MC_H264_LEVEL_5 = 0x4000,
	MC_H264_LEVEL_51 = 0x8000,
	MC_H264_LEVEL_52 = 0x10000,
	MC_H264_LEVEL_6 = 0x20000,
	MC_H264_LEVEL_61 = 0x40000,
	MC_H264_LEVEL_62 = 0x80000,
};


/* See
 * https://developer.android.com/reference/android/media/MediaCodecInfo.CodecProfileLevel
 * for reference. */
enum mc_h265_profile {
	MC_H265_PROFILE_MAIN = 1,
	MC_H265_PROFILE_MAIN10 = 2,
	MC_H265_PROFILE_MAIN10_HDR10 = 4096,
	MC_H265_PROFILE_MAIN10_HDR10PLUS = 8192,
	MC_H265_PROFILE_MAIN_STILL = 4,
};


/* See
 * https://developer.android.com/reference/android/media/MediaCodecInfo.CodecProfileLevel
 * for reference. */
enum mc_h265_level {
	MC_H265_HIGH_TIER_LEVEL1 = 0x2,
	MC_H265_HIGH_TIER_LEVEL2 = 0x8,
	MC_H265_HIGH_TIER_LEVEL21 = 0x20,
	MC_H265_HIGH_TIER_LEVEL3 = 0x80,
	MC_H265_HIGH_TIER_LEVEL31 = 0x200,
	MC_H265_HIGH_TIER_LEVEL4 = 0x800,
	MC_H265_HIGH_TIER_LEVEL41 = 0x2000,
	MC_H265_HIGH_TIER_LEVEL5 = 0x8000,
	MC_H265_HIGH_TIER_LEVEL51 = 0x20000,
	MC_H265_HIGH_TIER_LEVEL52 = 0x80000,
	MC_H265_HIGH_TIER_LEVEL6 = 0x200000,
	MC_H265_HIGH_TIER_LEVEL61 = 0x800000,
	MC_H265_HIGH_TIER_LEVEL62 = 0x2000000,
	MC_H265_MAIN_TIER_LEVEL1 = 0x1,
	MC_H265_MAIN_TIER_LEVEL2 = 0x4,
	MC_H265_MAIN_TIER_LEVEL21 = 0x10,
	MC_H265_MAIN_TIER_LEVEL3 = 0x40,
	MC_H265_MAIN_TIER_LEVEL31 = 0x100,
	MC_H265_MAIN_TIER_LEVEL4 = 0x400,
	MC_H265_MAIN_TIER_LEVEL41 = 0x1000,
	MC_H265_MAIN_TIER_LEVEL5 = 0x4000,
	MC_H265_MAIN_TIER_LEVEL51 = 0x10000,
	MC_H265_MAIN_TIER_LEVEL52 = 0x40000,
	MC_H265_MAIN_TIER_LEVEL6 = 0x100000,
	MC_H265_MAIN_TIER_LEVEL61 = 0x400000,
	MC_H265_MAIN_TIER_LEVEL62 = 0x1000000,
};


struct venc_mediacodec {
	struct venc_encoder *base;

	AMediaCodec *mc;
	AMediaFormat *format;
	ANativeWindow *surface;

	struct mbuf_raw_video_frame_queue *in_queue;
	struct mbuf_raw_video_frame_queue *meta_queue;
	struct mbuf_coded_video_frame_queue *out_queue;
	struct mbuf_mem *mem;
	struct pomp_evt *out_evt;

	char *enc_name;

	struct {
		pthread_t thread;
		bool thread_created;
		pthread_mutex_t mutex;
		pthread_cond_t cond;
		bool cond_signalled;
		bool stop_flag;
		bool flush_flag;
		bool eos_flag;
	} push;

	struct {
		pthread_t thread;
		bool thread_created;
		pthread_mutex_t mutex;
		pthread_cond_t cond;
		bool cond_signalled;
		bool stop_flag;
		bool flush_flag;
		bool eos_flag;
	} pull;

	bool eos_flag;

	struct venc_dyn_config dynconf;

	void *libmediandk_handle;
	media_status_t (*set_parameters)(AMediaCodec *codec,
					 const AMediaFormat *params);

	atomic_int state;
};


/* Type conversion functions between Mediacodec & venc types */
#include "venc_mediacodec_convert.h"


#endif /* !_VENC_MEDIACODEC_PRIV_H_ */
