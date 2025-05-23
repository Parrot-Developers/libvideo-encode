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

#ifndef _VENC_PRIV_H_
#define _VENC_PRIV_H_

#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>

#include <futils/futils.h>
#include <h264/h264.h>
#include <h265/h265.h>
#include <video-encode/venc.h>
#include <video-encode/venc_h264.h>
#include <video-encode/venc_h265.h>
#include <video-encode/venc_internal.h>
#include <video-streaming/vstrm.h>


#define VENC_H264_MAIN_PROFILE 77
#define VENC_H264_LEVEL_4_0 40
#define VENC_DEFAULT_GOP_LENGTH_SEC 1.f

#define VENC_H265_MAIN_PROFILE 1
#define VENC_H265_LEVEL_4_0 40

#ifdef BUILD_LIBVIDEO_ENCODE_X264
#	include <video-encode/venc_x264.h>
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_X265
#	include <video-encode/venc_x265.h>
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_HISI
#	include <video-encode/venc_hisi.h>
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_QCOM
#	include <video-encode/venc_qcom.h>
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_QCOM_JPEG
#	include <video-encode/venc_qcom_jpeg.h>
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_MEDIACODEC
#	include <video-encode/venc_mediacodec.h>
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_FAKEH264
#	include <video-encode/venc_fakeh264.h>
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_VIDEOTOOLBOX
#	include <video-encode/venc_videotoolbox.h>
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_TURBOJPEG
#	include <video-encode/venc_turbojpeg.h>
#endif

#ifdef BUILD_LIBVIDEO_ENCODE_PNG
#	include <video-encode/venc_png.h>
#endif

static inline void xfree(void **ptr)
{
	if (ptr) {
		free(*ptr);
		*ptr = NULL;
	}
}


static inline char *xstrdup(const char *s)
{
	return s == NULL ? NULL : strdup(s);
}


#endif /* !_VENC_PRIV_H_ */
