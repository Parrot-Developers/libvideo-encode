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

#ifndef _VENC_INTERNAL_H_
#define _VENC_INTERNAL_H_

#ifndef _GNU_SOURCE
#	define _GNU_SOURCE
#endif

#include <h264/h264.h>
#include <inttypes.h>
#include <video-streaming/vstrm.h>

#include <video-encode/venc_core.h>

#include <video-metadata/vmeta.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct venc_ops {
	/**
	 * Get the supported encodings for the implementation.
	 * Each implementation supports at least one encoding, and
	 * optionally more. The returned encodings array
	 * is a static array whose size is the return value of this function.
	 * If this function returns an error (negative errno value), then the
	 * value of *encodings is undefined.
	 * @param formats: pointer to the supported encodings list (output)
	 * @return the size of the encodings array, or a negative errno on
	 * error.
	 */
	int (*get_supported_encodings)(const enum vdef_encoding **encodings);

	/**
	 * Get the supported input buffer data formats for the implementation.
	 * Each implementation supports at least one input format, and
	 * optionally more. All input buffers need to be in one of the supported
	 * formats, otherwise they will be discarded. The returned formats array
	 * is a static array whose size is the return value of this function.
	 * If this function returns an error (negative errno value), then the
	 * value of *formats is undefined.
	 * @param formats: pointer to the supported formats list (output)
	 * @return the size of the formats array, or a negative errno on error.
	 */
	int (*get_supported_input_formats)(
		const struct vdef_raw_format **formats);

	/**
	 * Create an encoder implementation instance.
	 * When no longer needed, the instance must be freed using the
	 * destroy() function.
	 * @param base: base instance
	 * @return 0 on success, negative errno value in case of error
	 */
	int (*create)(struct venc_encoder *base);

	/**
	 * Flush the encoder implemetation.
	 * This function flushes all queues and optionally discards all buffers
	 * retained by the encoder. If the buffers are not discarded the
	 * frame output callback is called for each frame when the encoding
	 * is complete. The function is asynchronous and returns immediately.
	 * When flushing is complete the flush callback function is called if
	 * defined. After flushing the encoder new input buffers can still
	 * be queued.
	 * @param base: base instance
	 * @param discard: if null, all pending buffers are output, otherwise
	 *                 they are discarded
	 * @return 0 on success, negative errno value in case of error
	 */
	int (*flush)(struct venc_encoder *base, int discard);

	/**
	 * Stop the encoder implementation.
	 * This function stops any running threads. The function is asynchronous
	 * and returns immediately. When stopping is complete the stop callback
	 * function is called if defined. After stopping the encoder no new
	 * input buffers can be queued and the encoder instance must be freed
	 * using the destroy() function.
	 * @param base: base instance
	 * @return 0 on success, negative errno value in case of error
	 */
	int (*stop)(struct venc_encoder *base);

	/**
	 * Free an encoder implementation instance.
	 * This function frees all resources associated with an encoder
	 * implementation instance.
	 * @note this function blocks until all internal threads (if any)
	 * can be joined
	 * @param base: base instance
	 * @return 0 on success, negative errno value in case of error
	 */
	int (*destroy)(struct venc_encoder *base);

	/**
	 * Get the input buffer pool.
	 * The input buffer pool is defined only for implementations that
	 * require using input buffers from the encoder's own pool. This
	 * function must be called prior to encoding and if the returned
	 * value is not NULL the input buffer pool should be used to get input
	 * buffers. If the input buffers provided are not originating from the
	 * pool, they will be copied resulting in a loss of performance.
	 * @param base: base instance
	 * @return a pointer on the input buffer pool on success, NULL in case
	 * of error of if no pool is used
	 */
	struct mbuf_pool *(*get_input_buffer_pool)(struct venc_encoder *base);

	/**
	 * Get the input buffer queue.
	 * This function must be called prior to encoding and the input
	 * buffer queue must be used to push input buffers for encoding.
	 * @param base: base instance
	 * @return a pointer on the input buffer queue on success, NULL in case
	 * of error
	 */
	struct mbuf_raw_video_frame_queue *(*get_input_buffer_queue)(
		struct venc_encoder *base);

	/**
	 * Get the dynamic encoder configuration.
	 * The caller must provide a configuration structure to fill.
	 * @param base: base instance
	 * @param config: pointer to a dynamic configuration structure (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	int (*get_dyn_config)(struct venc_encoder *base,
			      struct venc_dyn_config *config);

	/**
	 * Set the dynamic encoder configuration.
	 * The caller must provide a filled configuration structure.
	 * @param base: base instance
	 * @param config: pointer to a dynamic configuration structure
	 * @return 0 on success, negative errno value in case of error
	 */
	int (*set_dyn_config)(struct venc_encoder *base,
			      const struct venc_dyn_config *config);

	/**
	 * Get the input buffer constraints (optional).
	 * The caller must provide a constraints structure to fill.
	 * If the implementation does not have any input buffers constraints,
	 * the pointer can be NULL.
	 * @param format: input format used within the encoder
	 * @param constraints: pointer to a venc_input_buffer_constraints
	 * structure (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	int (*get_input_buffer_constraints)(
		const struct vdef_raw_format *format,
		struct venc_input_buffer_constraints *constraints);

	/**
	 * Request an IDR frame (optional).
	 * This function requests encoding an IDR frame as soon as possible.
	 * If frames have already been submitted through the input queue,
	 * there is no guarantee that the next frame to be output will be
	 * an IDR frame. If the implementation does not support inserting
	 * IDR frames on-demand, the pointer should be NULL.
	 * @param base: base instance
	 * @return 0 on success, negative errno value in case of error
	 */
	int (*request_idr)(struct venc_encoder *base);
};

struct venc_encoder {
	void *derived;
	const struct venc_ops *ops;
	struct pomp_loop *loop;
	struct venc_cbs cbs;
	void *userdata;
	struct venc_config config;

	union {
		struct {
			struct h264_ctx *ctx;
			uint8_t *sps;
			size_t sps_size;
			uint8_t *pps;
			size_t pps_size;
		} h264;
		struct {
			struct h265_ctx *ctx;
			uint8_t *vps;
			size_t vps_size;
			uint8_t *sps;
			size_t sps_size;
			uint8_t *pps;
			size_t pps_size;
		} h265;
	};
	unsigned int mb_width;
	unsigned int mb_height;
	unsigned int slice_count;
	unsigned int slice_mb_count;
	unsigned int slice_mb_count_recovery_point;
	unsigned int recovery_frame_cnt;
	uint64_t last_timestamp;
};

/**
 * Default filter for the input frame queue.
 * This function is intended to be used as a standalone input filter.
 * It will call venc_default_input_filter_internal(), and then
 * venc_default_input_filter_internal_confirm_frame() if the former returned
 * true.
 *
 * @param frame: The frame to filter.
 * @param userdata: The venc_encoder structure.
 *
 * @return true if the frame passes the checks, false otherwise
 */
VENC_API bool venc_default_input_filter(struct mbuf_raw_video_frame *frame,
					void *userdata);

/**
 * Default filter for the input frame queue.
 * This filter does the following checks:
 * - frame is in a supported format
 * - frame info matches input config
 * - frame timestamp is strictly monotonic
 * This version is intended to be used by custom filters, to avoid calls to
 * mbuf_raw_video_frame_get_frame_info() or get_supported_input_formats().
 *
 * @warning This function does NOT check input validity. Arguments must not be
 * NULL, except for supported_formats if nb_supported_formats is zero.
 *
 * @param encoder: The base video encoder.
 * @param frame: The frame to filter.
 * @param frame_info: The associated vdef_raw_frame.
 * @param supported_formats: The formats supported by the implementation.
 * @param nb_supported_formats: The size of the supported_formats array.
 *
 * @return true if the frame passes the checks, false otherwise
 */
VENC_API bool venc_default_input_filter_internal(
	struct venc_encoder *encoder,
	struct mbuf_raw_video_frame *frame,
	struct vdef_raw_frame *frame_info,
	const struct vdef_raw_format *supported_formats,
	unsigned int nb_supported_formats);

/**
 * Filter update function.
 * This function should be called at the end of a custom filter. It registers
 * that the frame was accepted. This function saves the frame timestamp for
 * monotonic checks, and sets the VENC_ANCILLARY_KEY_INPUT_TIME ancillary data
 * on the frame.
 *
 * @param encoder: The base video encoder.
 * @param frame: The accepted frame.
 * @param frame_info: The associated vdef_raw_frame.
 */
VENC_API void venc_default_input_filter_internal_confirm_frame(
	struct venc_encoder *encoder,
	struct mbuf_raw_video_frame *frame,
	struct vdef_raw_frame *frame_info);


/**
 * Specific config getter with proper implem checks.
 * @param config: base configuration
 * @param implem: requested implementation
 * @return the specific config if it matches the requested implem,
 *         NULL otherwise
 */
VENC_API struct venc_config_impl *
venc_config_get_specific(struct venc_config *config,
			 enum venc_encoder_implem implem);


/**
 * Copy raw frame as frame info, meta data and ancillary data
 * without copying the data related to the planes.
 * It prevents holding the input raw frames too long
 * on systems with memory constraint.
 * @param frame: Pointer to the frame object.
 * @param mem: Pointer to the memory.
 * @param ret_obj: [out] Pointer to the new frame object.
 * @return 0 on success, negative errno on error.
 */
VENC_API int
venc_copy_raw_frame_as_metadata(struct mbuf_raw_video_frame *frame,
				struct mbuf_mem *mem,
				struct mbuf_raw_video_frame **ret_obj);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_VENC_INTERNAL_H_ */
