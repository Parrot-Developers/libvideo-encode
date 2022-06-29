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

#ifndef _VENC_H_
#define _VENC_H_

#include <video-encode/venc_core.h>

#include <stdint.h>
#include <unistd.h>

#include <libpomp.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-defs/vdefs.h>

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

/**
 * Get the supported encodings for the given encoder implementation.
 * Each implementation supports at least one encoding, and optionally more.
 * The returned encodings array is a static array whose size is the return value
 * of this function. If this function returns an error (negative errno value),
 * then the value of *encodings is undefined.
 * @param implem: encoder implementation
 * @return bit field of the supported encodings
 */
VENC_API int venc_get_supported_encodings(enum venc_encoder_implem implem,
					  const enum vdef_encoding **encodings);


/**
 * Get the supported input buffer data formats for the given
 * encoder implementation.
 * Each implementation supports at least one input format,
 * and optionally more. All input buffers need to be in one of
 * the supported formats, otherwise they will be discarded.
 * The returned formats array is a static array whose size is the return value
 * of this function. If this function returns an error (negative errno value),
 * then the value of *formats is undefined.
 * @param implem: decoder implementation
 * @param formats: pointer to the supported formats list (output)
 * @return the size of the formats array, or a negative errno on error.
 */
VENC_API int
venc_get_supported_input_formats(enum venc_encoder_implem implem,
				 const struct vdef_raw_format **formats);


/**
 * Get the implementation that will be chosen in case VENC_ENCODER_IMPLEM_AUTO
 * is used
 * @return the encoder implementation, or VENC_ENCODER_IMPLEM_AUTO in
 * case of error
 */
VENC_API enum venc_encoder_implem venc_get_auto_implem(void);


/**
 * Get the implementation that supports the required encodings.
 * @param encoding: encoding to support
 * @return the encoder implementation, or VENC_ENCODER_IMPLEM_AUTO in
 * case of error
 */
VENC_API enum venc_encoder_implem
venc_get_auto_implem_by_encoding(enum vdef_encoding encoding);

/**
 * Create an encoder instance.
 * The configuration and callbacks structures must be filled.
 * The instance handle is returned through the venc parameter.
 * When no longer needed, the instance must be freed using the
 * venc_destroy() function.
 * @param loop: event loop to use
 * @param config: encoder configuration
 * @param cbs: encoder callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: encoder instance handle (output)
 * @return 0 on success, negative errno value in case of error
 */
VENC_API int venc_new(struct pomp_loop *loop,
		      const struct venc_config *config,
		      const struct venc_cbs *cbs,
		      void *userdata,
		      struct venc_encoder **ret_obj);


/**
 * Flush the encoder.
 * This function flushes all queues and optionally discards all buffers
 * retained by the encoder. If the buffers are not discarded the frame
 * output callback is called for each frame when the encoding is complete.
 * The function is asynchronous and returns immediately. When flushing is
 * complete the flush callback function is called if defined. After flushing
 * the encoder new input buffers can still be queued.
 * @param self: encoder instance handle
 * @param discard: if null, all pending buffers are output, otherwise they
 *        are discarded
 * @return 0 on success, negative errno value in case of error
 */
VENC_API int venc_flush(struct venc_encoder *self, int discard);


/**
 * Stop the encoder.
 * This function stops any running threads. The function is asynchronous
 * and returns immediately. When stopping is complete the stop callback
 * function is called if defined. After stopping the encoder no new input
 * buffers can be queued and the encoder instance must be freed using the
 * venc_destroy() function.
 * @param self: encoder instance handle
 * @return 0 on success, negative errno value in case of error
 */
VENC_API int venc_stop(struct venc_encoder *self);


/**
 * Free an encoder instance.
 * This function frees all resources associated with an encoder instance.
 * @note this function blocks until all internal threads (if any) can be
 * joined; therefore the application should call venc_stop() and wait for
 * the stop callback function to be called before calling venc_destroy().
 * @param self: encoder instance handle
 * @return 0 on success, negative errno value in case of error
 */
VENC_API int venc_destroy(struct venc_encoder *self);


/**
 * Get the input buffer pool.
 * The input buffer pool is defined only for implementations that require
 * using input buffers from the encoder's own pool. This function must
 * be called prior to encoding and if the returned value is not NULL the
 * input buffer pool must be used to get input buffers. If the input
 * buffers provided are not originating from the pool, they will be copied
 * resulting in a loss of performance.
 * @param self: encoder instance handle
 * @return a pointer on the input buffer pool on success, NULL in case of
 * error of if no pool is used
 */
VENC_API struct mbuf_pool *
venc_get_input_buffer_pool(struct venc_encoder *self);


/**
 * Get the input buffer queue.
 * This function must be called prior to encoding and the input
 * buffer queue must be used to push input buffers for encoding.
 * @param self: encoder instance handle
 * @return a pointer on the input buffer queue on success, NULL in case of error
 */
VENC_API struct mbuf_raw_video_frame_queue *
venc_get_input_buffer_queue(struct venc_encoder *self);


/**
 * Get the H.264 encoding parameter sets.
 * The caller must provide buffers with sufficient space for the SPS
 * and/or PPS. When buffers are provided the size parameters must point to
 * the available space in the buffers. If there is enough space the SPS
 * and PPS data will be copied into the provided buffer and the size will
 * be replaced by the actual size of the data.
 * The ownership of the SPS and PPS buffers stays with the caller.
 * The required size for the buffers can be retrieved by calling the function
 * with NULL SPS and PPS data pointers and non-NULL size pointers.
 * The format of the SPS and PPS is raw NAL units (no start codes).
 * @param self: encoder instance handle
 * @param sps: pointer to the SPS data (input/output, optional, can be NULL)
 * @param sps_size: pointer to the SPS size
 *                  (input/output, optional, can be NULL)
 * @param pps: pointer to the PPS data (input/output, optional, can be NULL)
 * @param pps_size: pointer to the PPS size
 *                  (input/output, optional, can be NULL)
 * @return 0 on success, negative errno value in case of error
 */
VENC_API int venc_get_h264_ps(struct venc_encoder *self,
			      uint8_t *sps,
			      size_t *sps_size,
			      uint8_t *pps,
			      size_t *pps_size);


/**
 * Get the H.265 encoding parameter sets.
 * The caller must provide buffers with sufficient space for the VPS, SPS
 * and/or PPS. When buffers are provided, the size parameters must point to
 * the available space in the buffers. If there is enough space, the VPS, SPS
 * and PPS data will be copied into the provided buffer and the size will
 * be replaced by the actual size of the data.
 * The ownership of the VPS, SPS and PPS buffers stays with the caller.
 * The required size for the buffers can be retrieved by calling the function
 * with NULL VPS, SPS and PPS data pointers and non-NULL size pointers.
 * The format of the VPS, SPS and PPS is raw NAL units (no start codes).
 * @param self: encoder instance handle
 * @param vps: pointer to the VPS data (input/output, optional, can be NULL)
 * @param vps_size: pointer to the VPS size
 *                  (input/output, optional, can be NULL)
 * @param sps: pointer to the SPS data (input/output, optional, can be NULL)
 * @param sps_size: pointer to the SPS size
 *                  (input/output, optional, can be NULL)
 * @param pps: pointer to the PPS data (input/output, optional, can be NULL)
 * @param pps_size: pointer to the PPS size
 *                  (input/output, optional, can be NULL)
 * @return 0 on success, negative errno value in case of error
 */
VENC_API int venc_get_h265_ps(struct venc_encoder *self,
			      uint8_t *vps,
			      size_t *vps_size,
			      uint8_t *sps,
			      size_t *sps_size,
			      uint8_t *pps,
			      size_t *pps_size);


/**
 * Get the encoder implementation used.
 * @param self: encoder instance handle
 * @return the encoder implementation used, or VENC_ENCODER_IMPLEM_AUTO
 * in case of error
 */
VENC_API enum venc_encoder_implem
venc_get_used_implem(struct venc_encoder *self);


/**
 * Get the dynamic encoder configuration.
 * The caller must provide a configuration structure to fill.
 * @param self: encoder instance handle
 * @param config: pointer to a dynamic configuration structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VENC_API int venc_get_dyn_config(struct venc_encoder *self,
				 struct venc_dyn_config *config);


/**
 * Set the dynamic encoder configuration.
 * The caller must provide a filled configuration structure.
 * @param self: encoder instance handle
 * @param config: pointer to a dynamic configuration structure
 * @return 0 on success, negative errno value in case of error
 */
VENC_API int venc_set_dyn_config(struct venc_encoder *self,
				 const struct venc_dyn_config *config);


/**
 * Get the input buffer constraints.
 * The caller must provide a constraints structure to fill.
 * @param implem: encoder implementation
 * @param format: pointer to an input format
 * @param constraints: pointer to a venc_input_buffer_constraints
 * structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VENC_API int venc_get_input_buffer_constraints(
	enum venc_encoder_implem implem,
	const struct vdef_raw_format *format,
	struct venc_input_buffer_constraints *constraints);


/**
 * Request an IDR frame.
 * This function requests encoding an IDR frame as soon as possible.
 * If frames have already been submitted through the input queue, there is
 * no guarantee that the next frame to be output will be an IDR frame.
 * Inserting IDR frames only makes sense for H.264 and H.265 encodings;
 * for other encoding types the function returns -EINVAL. Also not all
 * implementations support inserting IDR frames on-demand, therefore the
 * requirement may never be met; in that case the function returns -ENOSYS.
 * @param self: encoder instance handle
 * @return 0 on success, negative errno value in case of error
 */
VENC_API int venc_request_idr(struct venc_encoder *self);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_VENC_H_ */
