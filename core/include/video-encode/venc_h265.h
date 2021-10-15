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

#ifndef _VENC_H265_H_
#define _VENC_H265_H_

#include <stdint.h>

#include <h265/h265.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* To be used for all public API */
#ifdef VENC_API_EXPORTS
#	ifdef _WIN32
#		define VENC_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define VENC_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !VENC_API_EXPORTS */
#	define VENC_API
#endif /* !VENC_API_EXPORTS */


VENC_API int venc_h265_writer_new(const uint8_t *vps,
				  size_t vps_size,
				  const uint8_t *sps,
				  size_t sps_size,
				  const uint8_t *pps,
				  size_t pps_size,
				  struct h265_ctx **ret_obj);


VENC_API int venc_h265_writer_destroy(struct h265_ctx *h265);


VENC_API int venc_h265_aud_write(struct h265_ctx *h265,
				 struct mbuf_coded_video_frame *frame);


VENC_API int venc_h265_ps_copy(struct h265_ctx *h265,
			       struct mbuf_coded_video_frame *frame,
			       const uint8_t *vps,
			       size_t vps_size,
			       const uint8_t *sps,
			       size_t sps_size,
			       const uint8_t *pps,
			       size_t pps_size);


VENC_API int venc_h265_sei_reset(struct h265_ctx *h265);


VENC_API int venc_h265_sei_add_recovery_point(struct h265_ctx *h265,
					      unsigned int recovery_poc_cnt);


VENC_API int venc_h265_sei_add_time_code(struct h265_ctx *h265,
					 uint64_t timestamp);


VENC_API int venc_h265_sei_add_mdcv(struct h265_ctx *h265,
				    const struct vdef_format_info *format);


VENC_API int venc_h265_sei_add_cll(struct h265_ctx *h265,
				   const struct vdef_format_info *format);


VENC_API int venc_h265_sei_add_user_data(struct h265_ctx *h265,
					 const uint8_t *data,
					 size_t len);


VENC_API int venc_h265_sei_write(struct h265_ctx *h265,
				 struct mbuf_coded_video_frame *frame);


VENC_API int venc_h265_generate_nalus(struct venc_encoder *self,
				      struct mbuf_coded_video_frame *frame,
				      const struct vdef_coded_frame *info);


VENC_API int
venc_h265_format_convert(struct mbuf_coded_video_frame *frame,
			 const struct vdef_coded_format *target_format);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_VENC_H265_H_ */
