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

#ifndef _VENC_CORE_H_
#define _VENC_CORE_H_

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
 * mbuf ancillary data key for the input timestamp.
 *
 * Content is a 64bits microseconds value on a monotonic clock
 */
#define VENC_ANCILLARY_KEY_INPUT_TIME "venc.input_time"

/**
 * mbuf ancillary data key for the dequeue timestamp.
 *
 * Content is a 64bits microseconds value on a monotonic clock
 */
#define VENC_ANCILLARY_KEY_DEQUEUE_TIME "venc.dequeue_time"

/**
 * mbuf ancillary data key for the output timestamp.
 *
 * Content is a 64bits microseconds value on a monotonic clock
 */
#define VENC_ANCILLARY_KEY_OUTPUT_TIME "venc.output_time"

/**
 * mbuf ancillary data key for the input status.
 * This key is only set by certain implementations.
 *
 * Content is a struct venc_input_status
 */
#define VENC_ANCILLARY_KEY_INPUT_QUEUE_STATUS "venc.input_status"


/* Forward declarations */
struct venc_encoder;


struct venc_input_status {
	/* Number of frames waiting in the input queue */
	uint32_t queue_frames;

	/* Number of frames waiting in the encoder input */
	uint32_t encoder_frames;
};


/* Supported encoder implementations */
enum venc_encoder_implem {
	/* Automatically select encoder */
	VENC_ENCODER_IMPLEM_AUTO = 0,

	/* x264 H.264 encoder */
	VENC_ENCODER_IMPLEM_X264,

	/* x264 H.265 encoder */
	VENC_ENCODER_IMPLEM_X265,

	/* HiSilicon encoder */
	VENC_ENCODER_IMPLEM_HISI,

	/* Qualcomm encoder */
	VENC_ENCODER_IMPLEM_QCOM,

	/* Qualcomm encoder: JPEG implementation */
	VENC_ENCODER_IMPLEM_QCOM_JPEG,

	/* Mediacodec implementation */
	VENC_ENCODER_IMPLEM_MEDIACODEC,

	/* Fake H.264 encoder */
	VENC_ENCODER_IMPLEM_FAKEH264,

	/* Videotoolbox implementation */
	VENC_ENCODER_IMPLEM_VIDEOTOOLBOX,

	/* Turbojpeg implementation */
	VENC_ENCODER_IMPLEM_TURBOJPEG,

	VENC_ENCODER_IMPLEM_MAX,
};


/* Rate-control algorithm */
enum venc_rate_control {
	/* Constant bitrate */
	VENC_RATE_CONTROL_CBR = 0,

	/* Variable bitrate */
	VENC_RATE_CONTROL_VBR,

	/* Constant quantizer */
	VENC_RATE_CONTROL_CQ,
};


/* Entropy coding */
enum venc_entropy_coding {
	/* CABAC entropy coding */
	VENC_ENTROPY_CODING_CABAC = 0,

	/* CAVLC entropy coding */
	VENC_ENTROPY_CODING_CAVLC,
};


/* Intra refresh pattern */
enum venc_intra_refresh {
	/* No intra refresh */
	VENC_INTRA_REFRESH_NONE = 0,

	/* Refresh from top to bottom */
	VENC_INTRA_REFRESH_VERTICAL_SCAN,

	/* Refresh from the center and alternating down/up */
	VENC_INTRA_REFRESH_SMART_SCAN,
};


/* Encoder initial configuration, implementation specific extension
 * Each implementation might provide implementation specific configuration with
 * a structure compatible with this base structure (i.e. which starts with the
 * same implem field). */
struct venc_config_impl {
	/* Encoder implementation for this extension */
	enum venc_encoder_implem implem;
};


/* Encoder intital configuration */
struct venc_config {
	/* Encoder instance name (optional, can be NULL, copied internally) */
	const char *name;

	/* Device name (mandatory for some implementations,
	 * copied internally) */
	const char *device;

	/* Encoder implementation (AUTO means no preference,
	 * use the default implementation for the platform) */
	enum venc_encoder_implem implem;

	/* Preferred encoding thread count (0 means no preference,
	 * use the default value; 1 means no multi-threading;
	 * only relevant for CPU encoding implementations) */
	unsigned int preferred_thread_count;

	/* Limit the number of frames that can be in the encoder pipeline
	 * at any time. The frames will be waiting in the input queue
	 * if the limit is reached. (0 means no limit). */
	unsigned int preferred_max_frames_in_encoder;

	/* Encoding type (mandatory) */
	enum vdef_encoding encoding;

	/* Input configuration */
	struct {
		/* Input buffer pool preferred minimum buffer count, used
		 * only if the implementation uses it's own input buffer pool
		 * (0 means no preference, use the default value) */
		unsigned int preferred_min_buf_count;

		/* Input buffers data format (mandatory) */
		struct vdef_raw_format format;

		/* Input format info */
		struct vdef_format_info info;
	} input;

	/* Output configuration */
	struct {
		/* Output buffer pool preferred minimum buffer count
		 * (0 means no preference, use the default value) */
		unsigned int preferred_min_buf_count;

		/* Preferred output buffers data format
		 * (VDEF_CODED_DATA_FORMAT_UNKNOWN means no preference,
		 * use the default format of the implementation) */
		enum vdef_coded_data_format preferred_format;
	} output;

	/* Encoding-specific configuration */
	union {
		/* H.264-specific configuration */
		struct {
			/* Encoding profile (H.264 profile_idc)
			 * (if 0, defaults to main profile) */
			unsigned int profile;

			/* Encoding level (H.264 level_idc = level * 10)
			 * (if 0, defaults to level 4.0) */
			unsigned int level;

			/* Rate control algorithm (defaults to CBR) */
			enum venc_rate_control rate_control;

			/* Minimum QP ([1..51], optional, can be 0) */
			unsigned int min_qp;

			/* Maximum QP ([1..51], optional, can be 0) */
			unsigned int max_qp;

			/* QP for CQ rate-control mode
			 * (mandatory in that case) [1..51] */
			unsigned int qp;

			/* Intra QP delta ([-50..50], optional, set to default
			 * value if < -50 or > 50) */
			int intra_qp_delta;

			/* Chroma QP delta ([-12..12], optional, set to default
			 * value if < -12 or > 12) */
			int chroma_qp_delta;

			/* Maximum bitrate in bits per second for VBR and CBR
			 * rate-control modes (mandatory for these modes; also
			 * the initial target bitrate if target_bitrate is 0) */
			unsigned int max_bitrate;

			/* Initial target bitrate in bits per second for
			 * VBR and CBR rate-control modes (optional,
			 * set to max_bitrate if 0) */
			unsigned int target_bitrate;

			/* CPB size in bits for VBR and CBR rate-control modes
			 * (optional, can be 0) */
			unsigned int cpb_size;

			/* GOP length in seconds at nominal framerate
			 * (ignored in intra refresh modes; if 0.0, defaults
			 * to 1.0) */
			float gop_length_sec;

			/* Framerate decimation factor (1 means no decimation;
			 * if 0, defaults to 1) */
			unsigned int decimation;

			/* Base frame layer interval; the minimum value is
			 * ref_frame_interval as non-ref frames cannot be in the
			 * base layer; the value must be a multiple of
			 * ref_frame_interval (1 means every frame is in base
			 * layer; if 0 or if less than ref_frame_interval,
			 * defaults to ref_frame_interval)
			 *
			 * - E.g. every frame is ref and in the base layer
			 *   (i.e. 1 layer in total):
			 *   base_frame_interval = 1
			 *   ref_frame_interval = 1
			 *   1(1)     2(1)      3(1)      4(1)      5(1)
			 *    ^________|^________|^________|^________|
			 *
			 * - E.g. 1 frame out of 2 is ref and in the base layer
			 *   (i.e. 2 layers in total):
			 *   base_frame_interval = 2
			 *   ref_frame_interval = 2
			 *   1(1)     2(2)      3(1)      4(2)      5(1)
			 *    ^________|         |^________|         |
			 *    ^__________________|^__________________|
			 *
			 * - E.g. 1 frame out of 2 is ref and 1 out of 4 is in
			 *   the base layer (i.e. 3 layers in total):
			 *   base_frame_interval = 4
			 *   ref_frame_interval = 2
			 *   1(1)     2(3)      3(2)      4(3)      5(1)
			 *    ^________|         |^________|         |
			 *    ^__________________|                   |
			 *    ^______________________________________|
			 *
			 *    */
			unsigned int base_frame_interval;

			/* Reference frame interval (1 means every frame is
			 * reference; if 0, defaults to 1) */
			unsigned int ref_frame_interval;

			/* Slice size in macroblock row units (optional, can be
			 * 0, defaults to the frame macroblock height) */
			unsigned int slice_size_mbrows;

			/* Entropy coding (defaults to CABAC) */
			enum venc_entropy_coding entropy_coding;

			/* Intra refresh mode (defaults to NONE) */
			enum venc_intra_refresh intra_refresh;

			/* Intra refresh period in frame units (mandatory if
			 * intra refresh is enabled) */
			unsigned int intra_refresh_period;

			/* Intra refresh length in frame units, excluding
			 * non-reference frames; the intra_refresh_period must
			 * be at least ref_frame_interval*intra_refresh_length
			 * (mandatory if intra refresh is enabled) */
			unsigned int intra_refresh_length;

			/* Insert SPS and PPS NAL units at start of GOP (or
			 * start of refresh in intra refresh modes) */
			int insert_ps;

			/* Insert AUD NAL unit before each frame */
			int insert_aud;

			/* Insert recovery point SEI before each IDR frame
			 * (or start of refresh in intra refresh modes) */
			int insert_recovery_point_sei;

			/* Insert picture timing SEI before each frame */
			int insert_pic_timing_sei;

			/* Insert "Parrot Streaming" v2 or v4 user data SEI;
			 * the parameter must be respectively 2 or 4, or 0
			 * to disable */
			int streaming_user_data_sei_version;

			/* Serialize user data as H.264 user data SEI */
			int serialize_user_data;

			/* Set the H.264 NAL units header NRI bits according
			 * to RFC6184 */
			int rfc6184_nri_bits;
		} h264;
		struct {
			/* Encoding profile (H.265 general_profile_idc)
			 * (if 0, defaults to main profile) */
			unsigned int profile;

			/* Encoding level (H.265 general_level_idc = level * 10,
			 * e.g. general_level_idc = 51 -> level 5.1)
			 * (if 0, defaults to level 4.0) */
			unsigned int level;

			/* Rate control algorithm (defaults to CBR) */
			enum venc_rate_control rate_control;

			/* Minimum QP ([1..51], optional, can be 0) */
			unsigned int min_qp;

			/* Maximum QP ([1..51], optional, can be 0) */
			unsigned int max_qp;

			/* QP for CQ rate-control mode
			 * (mandatory in that case) [1..51] */
			unsigned int qp;

			/* Intra QP delta ([-50..50], optional, set to default
			 * value if < -50 or > 50) */
			int intra_qp_delta;

			/* Chroma QP delta ([-12..12], optional, set to default
			 * value if < -12 or > 12) */
			int chroma_qp_delta;

			/* Maximum bitrate in bits per second for VBR and CBR
			 * rate-control modes (mandatory for these modes; also
			 * the initial target bitrate if target_bitrate is 0) */
			unsigned int max_bitrate;

			/* Initial target bitrate in bits per second for
			 * VBR and CBR rate-control modes (optional,
			 * set to max_bitrate if 0) */
			unsigned int target_bitrate;

			/* CPB size in bits for VBR and CBR rate-control modes
			 * (optional, can be 0) */
			unsigned int cpb_size;

			/* GOP length in seconds at nominal framerate
			 * (if 0.0, defaults to 1.0) */
			float gop_length_sec;

			/* Framerate decimation factor (1 means no decimation;
			 * if 0, defaults to 1) */
			unsigned int decimation;

			/* Insert VPS, SPS and PPS NAL units at start of GOP */
			int insert_ps;

			/* Insert AUD NAL unit before each frame */
			int insert_aud;

			/* Insert recovery point SEI before each IDR frame */
			int insert_recovery_point_sei;

			/* Insert picture timing SEI before each frame */
			int insert_time_code_sei;

			/* Insert mastering display colour volume SEI
			 * at start of GOP */
			int insert_mdcv_sei;

			/* Insert content light level SEI at start of GOP */
			int insert_cll_sei;

			/* Insert "Parrot Streaming" v2 or v4 user data SEI;
			 * the parameter must be respectively 2 or 4, or 0
			 * to disable */
			int streaming_user_data_sei_version;

			/* Serialize user data as H.265 user data SEI */
			int serialize_user_data;
		} h265;
		struct {
			/* Rate control algorithm */
			enum venc_rate_control rate_control;

			/* Quality factor ([1..99], mandatory) */
			unsigned int quality;

			/* Maximum bitrate in bits per second for VBR and CBR
			 * rate-control modes (mandatory for these modes; also
			 * the initial target bitrate if target_bitrate is 0) */
			unsigned int max_bitrate;

			/* Initial target bitrate in bits per second for
			 * VBR and CBR rate-control modes (mandatory in
			 * that case) */
			unsigned int target_bitrate;
		} mjpeg;
	};

	/* Implementation specific extensions (optional, can be NULL)
	 * If not null, implem_cfg must match the following requirements:
	 *  - this->implem_cfg->implem == this->implem
	 *  - this->implem != VENC_ENCODER_IMPLEM_AUTO
	 *  - The real type of implem_cfg must be the implementation specific
	 *    structure, not struct venc_config_impl */
	struct venc_config_impl *implem_cfg;
};


/* Encoder dynamic configuration */
struct venc_dyn_config {
	/* QP for CQ rate-control mode (mandatory in that case) [1..51]
	 * (if 0, defaults to current value) */
	unsigned int qp;

	/* Target bitrate in bits per second for VBR and CBR rate-control
	 * modes (if 0, defaults to current value) */
	unsigned int target_bitrate;

	/* Framerate decimation factor (1 means no decimation;
	 * if 0, defaults to current value) */
	unsigned int decimation;
};


/* Encoder input buffer constraints */
struct venc_input_buffer_constraints {
	/* Stride alignment values: these values are used to align the width of
	 * each plane in bytes */
	unsigned int plane_stride_align[VDEF_RAW_MAX_PLANE_COUNT];

	/* Scanline alignment values: these values are used to align the height
	 * of each plane in lines */
	unsigned int plane_scanline_align[VDEF_RAW_MAX_PLANE_COUNT];

	/* Size alignment values: these values are used to align the size of
	 * each plane to the upper size in bytes */
	unsigned int plane_size_align[VDEF_RAW_MAX_PLANE_COUNT];
};


/* Encoder callback functions */
struct venc_cbs {
	/* Frame output callback function (mandatory)
	 * The library retains ownership of the output frame and the
	 * application must reference it if needed after returning from the
	 * callback function. The status is 0 in case of success, a negative
	 * errno otherwise. In case of error no frame is output and frame
	 * is NULL.
	 * @param enc: encoder instance handle
	 * @param status: frame output status
	 * @param frame: output frame
	 * @param userdata: user data pointer */
	void (*frame_output)(struct venc_encoder *enc,
			     int status,
			     struct mbuf_coded_video_frame *frame,
			     void *userdata);

	/* Flush callback function, called when flushing is complete (optional)
	 * @param enc: encoder instance handle
	 * @param userdata: user data pointer */
	void (*flush)(struct venc_encoder *enc, void *userdata);

	/* Stop callback function, called when stopping is complete (optional)
	 * @param enc: encoder instance handle
	 * @param userdata: user data pointer */
	void (*stop)(struct venc_encoder *enc, void *userdata);

	/* Pre-release callback function. (optional)
	 * If defined, this function will be called on each output frame when
	 * its ref-count reaches zero.
	 */
	mbuf_coded_video_frame_pre_release_t pre_release;
};


/**
 * Get an enum venc_encoder_implem value from a string.
 * Valid strings are only the suffix of the implementation name (eg. 'X264').
 * The case is ignored.
 * @param str: implementation name to convert
 * @return the enum venc_encoder_implem value or VENC_ENCODER_IMPLEM_AUTO
 *         if unknown
 */
VENC_API enum venc_encoder_implem venc_encoder_implem_from_str(const char *str);


/**
 * Get a string from an enum venc_encoder_implem value.
 * @param implem: implementation value to convert
 * @return a string description of the implementation
 */
VENC_API const char *
venc_encoder_implem_to_str(enum venc_encoder_implem implem);


/**
 * Get an enum venc_rate_control value from a string.
 * Valid strings are only the suffix of the rate control value (eg. 'CBR').
 * The case is ignored.
 * @param str: rate control value to convert
 * @return the enum venc_rate_control value or VENC_RATE_CONTROL_CBR
 *         if unknown
 */
VENC_API enum venc_rate_control venc_rate_control_from_str(const char *str);


/**
 * Get a string from an enum venc_rate_control value.
 * @param rc: rate control value to convert
 * @return a string description of the rate control
 */
VENC_API const char *venc_rate_control_to_str(enum venc_rate_control rc);


/**
 * Get an enum venc_entropy_coding value from a string.
 * Valid strings are only the suffix of the entropy coding value (eg. 'CABAC').
 * The case is ignored.
 * @param str: entropy coding value to convert
 * @return the enum venc_entropy_coding value or VENC_ENTROPY_CODING_CABAC
 *         if unknown
 */
VENC_API enum venc_entropy_coding venc_entropy_coding_from_str(const char *str);


/**
 * Get a string from an enum venc_entropy_coding value.
 * @param coding: entropy coding value to convert
 * @return a string description of the entropy coding
 */
VENC_API const char *
venc_entropy_coding_to_str(enum venc_entropy_coding coding);


/**
 * Get an enum venc_intra_refresh value from a string.
 * Valid strings are only the suffix of the intra refresh mode name name (eg.
 * 'SMART_SCAN'). The case is ignored.
 * @param str: intra refresh mode name to convert
 * @return the enum venc_intra_refresh value or VENC_INTRA_REFRESH_NONE
 *         if unknown
 */
VENC_API enum venc_intra_refresh venc_intra_refresh_from_str(const char *str);


/**
 * Get a string from an enum venc_intra_refresh value.
 * @param ir: intra refresh mode value to convert
 * @return a string description of the intra refresh mode
 */
VENC_API const char *venc_intra_refresh_to_str(enum venc_intra_refresh ir);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_VENC_CORE_H_ */
