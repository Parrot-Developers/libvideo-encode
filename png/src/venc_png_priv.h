/**
 * Copyright (c) 2023 Parrot Drones SAS
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

#ifndef _VENC_PNG_PRIV_H_
#define _VENC_PNG_PRIV_H_

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>

#if defined(__APPLE__)
#	include <TargetConditionals.h>
#endif

#include <futils/futils.h>
#include <libpomp.h>

#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>

#include <video-encode/venc_core.h>
#include <video-encode/venc_internal.h>
#include <video-encode/venc_png.h>

#include <png.h>

#define VENC_MSG_FLUSH 'f'
#define VENC_MSG_STOP 's'
#define SOFTPNG_MAX_PLANES 1

static inline void xfree(void **ptr)
{
	if (ptr) {
		free(*ptr);
		*ptr = NULL;
	}
}

struct png_picture {
	int color_type;
	unsigned int bit_depth;
	unsigned int width;
	unsigned int height;
	unsigned int planes;

	unsigned int compression_level;
	bool data_little_endian;
	int stride[SOFTPNG_MAX_PLANES];
	const unsigned char *plane[SOFTPNG_MAX_PLANES];
};

struct venc_png {
	struct venc_encoder *base;

	struct mbuf_raw_video_frame_queue *in_queue;
	struct mbuf_coded_video_frame_queue *enc_out_queue;
	struct pomp_evt *enc_out_queue_evt;

	struct png_picture in_picture;

	struct vdef_coded_format output_format;
	unsigned int input_frame_cnt;

	pthread_t thread;
	bool thread_launched;
	atomic_bool should_stop;
	atomic_bool flushing;
	atomic_bool flush_discard;
	struct mbox *mbox;
};


#endif /* !_VENC_PNG_PRIV_H_ */
