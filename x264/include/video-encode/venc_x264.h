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

#ifndef _VENC_X264_H_
#define _VENC_X264_H_

#include <stdint.h>

#include <video-encode/venc_core.h>

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


struct venc_x264;


struct venc_config_x264 {
	/* Encoder implementation for this extension.
	 * Keep this field for compatibility with 'struct venc_config_impl' */
	enum venc_encoder_implem implem;

	/* A preset is a collection of options that will provide a certain
	 * encoding speed to compression ratio. The changes it makes will be
	 * applied before all other parameters are applied.
	 * Must be dynamically allocated (will be freed). */
	const char *preset;

	/* Tune the settings. The changes it makes will be applied after the
	 * preset but before all other parameters.
	 * Must be dynamically allocated (will be freed). */
	const char *tune;
};


extern VENC_API const struct venc_ops venc_x264_ops;


#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* !_VENC_X264_H_ */
