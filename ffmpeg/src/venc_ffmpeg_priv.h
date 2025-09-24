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

#ifndef _VENC_FFMPEG_PRIV_H_
#define _VENC_FFMPEG_PRIV_H_

#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>

#include <futils/futils.h>
#include <libpomp.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-encode/venc_core.h>
#include <video-encode/venc_ffmpeg.h>
#include <video-encode/venc_h264.h>
#include <video-encode/venc_h265.h>
#include <video-encode/venc_internal.h>

#define VENC_FFMPEG_DEFAULT_THREAD_COUNT 8

#define VENC_MSG_FLUSH 'f'
#define VENC_MSG_STOP 's'

#define MAX_SUPPORTED_FORMATS 3
#define MAX_SUPPORTED_ENCODINGS 2


static inline void xfree(void **ptr)
{
	if (ptr) {
		free(*ptr);
		*ptr = NULL;
	}
}


struct venc_ffmpeg_nalu_info {
	struct vdef_nalu n;
	size_t offset;
};


enum venc_ffmpeg_backend_type {
	VENC_FFMPEG_BACKEND_TYPE_UNKNOWN = 0,
	VENC_FFMPEG_BACKEND_TYPE_H264_NVENC,
	VENC_FFMPEG_BACKEND_TYPE_HEVC_NVENC,
	VENC_FFMPEG_BACKEND_TYPE_OPENH264,
};


struct venc_ffmpeg_backend {
	enum venc_ffmpeg_backend_type type;
	const char *name;
	const AVCodec *codec;
	bool is_hw;
	bool is_tested;
	bool is_supported;
	struct vdef_raw_format supported_formats[MAX_SUPPORTED_FORMATS];
	size_t nb_supported_formats;
	enum vdef_encoding supported_encodings[MAX_SUPPORTED_ENCODINGS];
	size_t nb_supported_encodings;
};


struct venc_ffmpeg {
	struct venc_encoder *base;
	struct mbuf_raw_video_frame_queue *in_queue;
	struct mbuf_raw_video_frame_queue *enc_in_queue;
	struct mbuf_coded_video_frame_queue *enc_out_queue;
	struct pomp_evt *enc_out_queue_evt;

	struct venc_ffmpeg_backend *backend;
	AVCodecContext *avcodec;
	struct vdef_raw_format input_format;
	AVFrame *avframe;
	AVPacket *avpacket;
	AVPacket *dummy_packet;

	unsigned int input_frame_cnt;

	struct venc_ffmpeg_nalu_info *nalus;
	unsigned int nalu_max_count;
	unsigned int nalu_count;

	struct {
		enum venc_rate_control rc;
		uint64_t max_bitrate;
		uint64_t target_bitrate;
		uint32_t gop;
		const char *profile_str;
		const char *level_str;
		const char *rc_key_str;
		const char *rc_val_str;
		const char *coder_str;
		uint32_t qp;
		uint32_t decimation;
		uint8_t min_qp;
		uint8_t max_qp;
		bool use_intra_refresh;
	} attrs;

	struct {
		uint32_t nb_slice;
		uint32_t cur_index;
		uint32_t first_size;
		uint32_t last_size;
	} slice;

	struct {
		uint8_t *buf;
		size_t len;
		size_t stride;
		size_t count;
	} dummy_uv_plane;

	pthread_t thread;
	bool thread_launched;
	atomic_bool insert_idr;
	atomic_bool update_bitrate;
	atomic_bool should_stop;

	atomic_bool flush_requested;
	atomic_bool flushing;
	atomic_bool flush_discard;

	struct mbox *mbox;
	bool ps_ready;

	struct venc_dyn_config dynconf;
};


/* Type conversion functions between FFMPEG & venc types */
#include "venc_ffmpeg_convert.h"


#endif /* !_VENC_FFMPEG_PRIV_H_ */
