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

#include <VideoToolbox/VideoToolbox.h>
#include <futils/futils.h>
#include <libpomp.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <video-encode/venc_core.h>
#include <video-encode/venc_h264.h>
#include <video-encode/venc_h265.h>
#include <video-encode/venc_internal.h>
#include <video-encode/venc_videotoolbox.h>


#define VENC_VIDEOTOOLBOX_OUT_POOL_DEFAULT_MIN_BUF_COUNT 10
#define VENC_VIDEOTOOLBOX_BUF_TYPE_CVIB 0x43564942 /* "CVIB" */


enum venc_videotoolbox_message_type {
	VENC_VIDEOTOOLBOX_MESSAGE_TYPE_FLUSH = 'f',
	VENC_VIDEOTOOLBOX_MESSAGE_TYPE_STOP = 's',
	VENC_VIDEOTOOLBOX_MESSAGE_TYPE_ERROR = 'e',
};


struct venc_videotoolbox_message {
	enum venc_videotoolbox_message_type type;
	int error;
};


struct frame_data {
	struct mbuf_raw_video_frame *frame;
	const void *data;
};


struct venc_videotoolbox {
	struct venc_encoder *base;
	struct mbuf_raw_video_frame_queue *in_queue;
	struct mbuf_coded_video_frame_queue *out_queue;
	struct pomp_evt *out_queue_evt;
	VTCompressionSessionRef compress_ref;
	pthread_t thread;

	bool thread_launched;
	bool ps_lock_created;
	atomic_bool should_stop;
	atomic_bool is_stopped;
	atomic_bool flush;
	atomic_bool flushing;
	atomic_bool flush_discard;
	atomic_bool ps_stored;
	pthread_mutex_t ps_lock;

	struct venc_dyn_config dynconf;
	unsigned int input_frame_cnt;
	struct mbox *mbox;
};


#endif /* !_VENC_MEDIACODEC_PRIV_H_ */
