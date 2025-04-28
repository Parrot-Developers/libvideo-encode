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

#ifndef ANDROID
#	ifndef _FILE_OFFSET_BITS
#		define _FILE_OFFSET_BITS 64
#	endif /* _FILE_OFFSET_BITS */
#endif /* ANDROID */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef _WIN32
#	include <winsock2.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#endif /* !_WIN32 */

#include <futils/futils.h>
#include <libpomp.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-encode/venc.h>
#include <video-raw/vraw.h>

#define ULOG_TAG venc_prog
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);


#define DEFAULT_IN_BUF_COUNT 30


struct venc_prog {
	struct pomp_loop *loop;
	struct venc_encoder *encoder;
	int finishing;
	int flushed;
	int stopped;
	struct venc_config config;

	struct pomp_timer *encoder_timer;

	struct {
		struct vraw_reader *reader;
		int loop;
		unsigned int start_index;
		unsigned int count;
		unsigned int max_count;
		unsigned int decimation;
		int finished;
		struct vdef_raw_format format;
		struct vdef_format_info info;
		uint64_t fake_ts;
		struct vdef_raw_frame frame_info;
		struct mbuf_pool *pool;
		struct pomp_evt *pool_evt;
		int pool_allocated;
		int waiting;
		struct mbuf_raw_video_frame_queue *queue;
	} input;
	struct {
		unsigned int count;
		int finished;
		FILE *file;
		enum vdef_encoding encoding;
	} output;
};


static int s_stopping;
struct pomp_loop *s_loop;
struct venc_prog *s_prog;


static void finish_idle(void *userdata)
{
	int res;
	struct venc_prog *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	if (self->finishing)
		return;

	if ((s_stopping) || (self->input.finished)) {
		self->finishing = 1;

		/* Flush the encoder */
		res = venc_flush(self->encoder, (s_stopping) ? 1 : 0);
		if (res < 0)
			ULOG_ERRNO("venc_flush", -res);
	}

	if (self->encoder_timer)
		pomp_timer_clear(self->encoder_timer);
}


static int encode_frame(struct venc_prog *self)
{
	int res = 0, err;
	void *mem_data;
	uint8_t *in_data;
	size_t in_capacity;
	size_t plane_offset;
	struct mbuf_raw_video_frame *in_frame = NULL;
	struct timespec cur_ts = {0, 0};
	struct vraw_frame vframe;
	struct mbuf_mem *in_mem;
	struct vdef_raw_frame frame_info;
	unsigned int plane_count;

	res = mbuf_pool_get(self->input.pool, &in_mem);
	if (res == -EAGAIN) {
		self->input.waiting = 1;
		res = pomp_evt_signal(self->input.pool_evt);
		if (res < 0)
			ULOG_ERRNO("pomp_evt_signal", -res);
		return 0;
	} else if (res < 0) {
		ULOG_ERRNO("mbuf_pool_get:input", -res);
		return res;
	}

	res = mbuf_mem_get_data(in_mem, &mem_data, &in_capacity);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_get_data", -res);
		goto out;
	}
	in_data = mem_data;

	switch (self->config.implem) {
	case VENC_ENCODER_IMPLEM_FAKEH264:
		frame_info.format = vdef_i420;
		vdef_format_to_frame_info(&self->input.info, &frame_info.info);
		frame_info.info.timestamp = self->input.fake_ts;
		frame_info.info.timescale = self->input.info.framerate.num;
		frame_info.info.index = self->input.count;
		self->input.fake_ts += self->input.info.framerate.den;
		frame_info.plane_stride[0] = 0;
		frame_info.plane_stride[1] = 0;
		frame_info.plane_stride[2] = 0;
		break;
	default:

		res = vraw_reader_frame_read(
			self->input.reader, in_data, in_capacity, &vframe);
		if (res == -ENOENT) {
			res = 0;
			self->input.finished = 1;
			goto out;
		} else if (res < 0) {
			ULOG_ERRNO("vraw_reader_frame_read", -res);
			goto out;
		}
		self->input.frame_info.plane_stride[0] =
			vframe.frame.plane_stride[0];
		self->input.frame_info.plane_stride[1] =
			vframe.frame.plane_stride[1];
		self->input.frame_info.plane_stride[2] =
			vframe.frame.plane_stride[2];
		frame_info = vframe.frame;
		break;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &self->input.frame_info.info.timestamp);

	frame_info.info.capture_timestamp =
		(uint64_t)frame_info.info.timestamp * 1000000 /
		frame_info.info.timescale;

	res = mbuf_raw_video_frame_new(&frame_info, &in_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_new:input", -res);
		return res;
	}

	plane_count = vdef_get_raw_frame_plane_count(&self->input.format);

	switch (self->config.implem) {
	case VENC_ENCODER_IMPLEM_FAKEH264:
		res = mbuf_raw_video_frame_set_plane(in_frame, 0, in_mem, 0, 0);
		if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane[0]", -res);
			goto out;
		}
		if (plane_count >= 2) {
			res = mbuf_raw_video_frame_set_plane(
				in_frame, 1, in_mem, 0, 0);
			if (res < 0) {
				ULOG_ERRNO("mbuf_raw_video_frame_set_plane[0]",
					   -res);
				goto out;
			}
		}
		if (plane_count >= 3) {
			res = mbuf_raw_video_frame_set_plane(
				in_frame, 2, in_mem, 0, 0);
			if (res < 0) {
				ULOG_ERRNO("mbuf_raw_video_frame_set_plane[0]",
					   -res);
				goto out;
			}
		}
		break;
	default:
		plane_offset = vframe.data[0] - in_data;
		res = mbuf_raw_video_frame_set_plane(
			in_frame,
			0,
			in_mem,
			plane_offset,
			self->input.frame_info.plane_stride[0] *
				self->input.info.resolution.height);
		if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane[0]", -res);
			goto out;
		}
		if (plane_count >= 2) {
			plane_offset = vframe.data[1] - in_data;
			res = mbuf_raw_video_frame_set_plane(
				in_frame,
				1,
				in_mem,
				plane_offset,
				self->input.frame_info.plane_stride[1] *
					self->input.info.resolution.height / 2);
			if (res < 0) {
				ULOG_ERRNO("mbuf_raw_video_frame_set_plane[1]",
					   -res);
				goto out;
			}
		}
		if (plane_count >= 3) {
			plane_offset = vframe.data[2] - in_data;
			res = mbuf_raw_video_frame_set_plane(
				in_frame,
				2,
				in_mem,
				plane_offset,
				self->input.frame_info.plane_stride[2] *
					self->input.info.resolution.height / 2);
			if (res < 0) {
				ULOG_ERRNO("mbuf_raw_video_frame_set_plane[2]",
					   -res);
				goto out;
			}
		}
	}

	res = mbuf_raw_video_frame_finalize(in_frame);
	if (res < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_finalize", -res);

	res = mbuf_raw_video_frame_queue_push(self->input.queue, in_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_push", -res);
		goto out;
	}

	self->input.frame_info.info.index++;
	self->input.count++;
	self->output.finished = 0;
	if ((self->input.max_count > 0) &&
	    (self->input.count >= self->input.max_count))
		self->input.finished = 1;

out:
	if (in_frame) {
		err = mbuf_raw_video_frame_unref(in_frame);
		if (err < 0)
			ULOG_ERRNO("mbuf_raw_video_frame_unref:input", -err);
	}
	err = mbuf_mem_unref(in_mem);
	if (err < 0)
		ULOG_ERRNO("mbuf_mem_unref:input", -err);

	if (!self->input.finished)
		return res;

	ULOGI("encoding is finished (input, count=%d)", self->input.count);
	if (self->encoder_timer)
		pomp_timer_clear(self->encoder_timer);
	err = pomp_loop_idle_add(self->loop, &finish_idle, self);
	if (err < 0)
		ULOG_ERRNO("pomp_loop_idle_add", -err);
	return res;
}


static void encoder_timer_cb(struct pomp_timer *timer, void *userdata)
{
	struct venc_prog *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	res = encode_frame(self);
	if (res < 0)
		ULOG_ERRNO("encode_frame", -res);
}


static void encode_frame_idle(void *userdata)
{
	struct venc_prog *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	if (self->finishing)
		return;

	res = encode_frame(self);
	if (res < 0) {
		ULOG_ERRNO("encode_frame", -res);
		return;
	}

	if ((!self->input.finished) && (!self->input.waiting) &&
	    (!self->finishing)) {
		res = pomp_loop_idle_add(self->loop, &encode_frame_idle, self);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add", -res);
	}
}


static void pool_event_cb(struct pomp_evt *evt, void *userdata)
{
	struct venc_prog *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	if (!self->input.waiting)
		return;

	self->input.waiting = 0;

	if ((!self->input.finished) && (!self->finishing)) {
		res = pomp_loop_idle_add(self->loop, &encode_frame_idle, self);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add", -res);
	}
}


static uint64_t get_timestamp(struct mbuf_coded_video_frame *frame,
			      const char *key)
{
	int res;
	struct mbuf_ancillary_data *data;
	uint64_t ts = 0;
	const void *raw_data;
	size_t len;

	res = mbuf_coded_video_frame_get_ancillary_data(frame, key, &data);
	if (res < 0)
		return 0;

	raw_data = mbuf_ancillary_data_get_buffer(data, &len);
	if (!raw_data || len != sizeof(ts))
		goto out;
	memcpy(&ts, raw_data, sizeof(ts));

out:
	mbuf_ancillary_data_unref(data);
	return ts;
}


static int frame_output(struct venc_prog *self,
			struct mbuf_coded_video_frame *out_frame)
{
	int res = 0;
	struct vdef_coded_frame out_info = {};
	const uint8_t *data;
	const void *nalu_data;
	struct vdef_nalu nalu;
	size_t i;
	uint64_t input_time, dequeue_time, output_time;
	size_t nalu_count;

	if (self->input.waiting) {
		res = pomp_evt_signal(self->input.pool_evt);
		if (res < 0)
			ULOG_ERRNO("pomp_evt_signal", -res);
	}

	input_time = get_timestamp(out_frame, VENC_ANCILLARY_KEY_INPUT_TIME);
	dequeue_time =
		get_timestamp(out_frame, VENC_ANCILLARY_KEY_DEQUEUE_TIME);
	output_time = get_timestamp(out_frame, VENC_ANCILLARY_KEY_OUTPUT_TIME);

	res = mbuf_coded_video_frame_get_frame_info(out_frame, &out_info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		return res;
	}

	nalu_count = mbuf_coded_video_frame_get_nalu_count(out_frame);

	ULOGI("encoded frame #%d layer=%u type=%s nalu_count=%zu "
	      "(dequeue: %.2fms, encode: %.2fms, overall: %.2fms)",
	      out_info.info.index,
	      out_info.layer,
	      vdef_coded_frame_type_to_str(out_info.type),
	      nalu_count,
	      (float)(dequeue_time - input_time) / 1000.,
	      (float)(output_time - dequeue_time) / 1000.,
	      (float)(output_time - input_time) / 1000.);

	if (self->output.file == NULL)
		return 0;

	/* Write to file */
	switch (out_info.format.data_format) {
	case VDEF_CODED_DATA_FORMAT_BYTE_STREAM:
	case VDEF_CODED_DATA_FORMAT_JFIF:
	case VDEF_CODED_DATA_FORMAT_UNKNOWN:
		for (i = 0; i < nalu_count; i++) {
			res = mbuf_coded_video_frame_get_nalu(
				out_frame, i, &nalu_data, &nalu);
			if (res < 0) {
				ULOG_ERRNO("mbuf_coded_video_frame_get_nalu",
					   -res);
				return res;
			}
			data = nalu_data;

			switch (self->output.encoding) {
			case VDEF_ENCODING_H264:
				ULOGD("-- NALU type=%s size=%zu importance=%u "
				      "mb_count=%d",
				      h264_nalu_type_str(nalu.h264.type),
				      nalu.size,
				      nalu.importance,
				      nalu.h264.slice_mb_count);
				break;
			case VDEF_ENCODING_H265:
				ULOGD("-- NALU type=%s size=%zu importance=%u",
				      h265_nalu_type_str(nalu.h265.type),
				      nalu.size,
				      nalu.importance);
				break;
			default:
				break;
			}
			res = fwrite(data, nalu.size, 1, self->output.file);
			if (res != 1) {
				ULOG_ERRNO("fwrite", errno);
				break;
			}

			res = mbuf_coded_video_frame_release_nalu(
				out_frame, i, nalu_data);
			if (res < 0) {
				ULOG_ERRNO(
					"mbuf_coded_video_frame_release_nalu",
					-res);
				return res;
			}
		}
		break;
	default:
		break;
	}

	return 0;
}


static void frame_output_cb(struct venc_encoder *enc,
			    int status,
			    struct mbuf_coded_video_frame *out_frame,
			    void *userdata)
{
	struct venc_prog *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(out_frame == NULL, EINVAL);

	if (status != 0) {
		ULOGE("encoder error, resync required");
		return;
	}

	res = frame_output(self, out_frame);
	if (res < 0)
		ULOG_ERRNO("frame_output", -res);

	self->output.count++;

	if ((self->input.finished) &&
	    (self->output.count >=
	     (self->input.count / self->input.decimation))) {
		ULOGI("encoding is finished (output, count=%d)",
		      self->output.count);
		self->output.finished = 1;
	}
}


static void flush_cb(struct venc_encoder *enc, void *userdata)
{
	struct venc_prog *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	ULOGI("encoder is flushed");
	self->flushed = 1;

	/* Stop the encoder */
	res = venc_stop(self->encoder);
	if (res < 0)
		ULOG_ERRNO("venc_stop", -res);
}


static void stop_cb(struct venc_encoder *enc, void *userdata)
{
	struct venc_prog *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	ULOGI("encoder is stopped");
	self->stopped = 1;

	pomp_loop_wakeup(self->loop);
}


static const struct venc_cbs venc_cbs = {
	.frame_output = frame_output_cb,
	.flush = flush_cb,
	.stop = stop_cb,
};


static void sighandler(int signum)
{
	printf("Stopping...\n");
	s_stopping = 1;
	if (s_loop != NULL) {
		int res;
		ULOGI("encoding interrupted");
		res = pomp_loop_idle_add(s_loop, &finish_idle, s_prog);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add", -res);
		res = pomp_loop_wakeup(s_loop);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_wakeup", -res);
	}
	signal(SIGINT, SIG_DFL);
}


enum args_id {
	ARGS_ID_IMPLEM = 256,
	ARGS_ID_MIN_QP,
	ARGS_ID_MAX_QP,
	ARGS_ID_INTRA_QP_DELTA,
	ARGS_ID_CHROMA_QP_DELTA,
	ARGS_ID_IR,
	ARGS_ID_IR_PERIOD,
	ARGS_ID_IR_LENGTH,
	ARGS_ID_BASE_FRAME,
	ARGS_ID_REF_FRAME,
	ARGS_ID_FULL_RANGE,
	ARGS_ID_COLOR,
	ARGS_ID_TRANSFER,
	ARGS_ID_MATRIX,
	ARGS_ID_SAR,
	ARGS_ID_MIN_BUF_COUNT,
	ARGS_ID_INSERT_PIC_TIMING_SEI,
	ARGS_ID_SET_NRI_BITS,
};


static const char short_options[] = "hi:f:W:H:S:F:Ld:s:n:l:o:e:r:b:q:G:";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"implem", required_argument, NULL, ARGS_ID_IMPLEM},
	{"infile", required_argument, NULL, 'i'},
	{"format", required_argument, NULL, 'f'},
	{"width", required_argument, NULL, 'W'},
	{"height", required_argument, NULL, 'H'},
	{"framerate", required_argument, NULL, 'F'},
	{"decimation", required_argument, NULL, 'd'},
	{"start", required_argument, NULL, 's'},
	{"count", required_argument, NULL, 'n'},
	{"loop", required_argument, NULL, 'l'},
	{"fake-live", no_argument, NULL, 'L'},
	{"outfile", required_argument, NULL, 'o'},
	{"encoding", required_argument, NULL, 'e'},
	{"rc", required_argument, NULL, 'r'},
	{"bitrate", required_argument, NULL, 'b'},
	{"qp", required_argument, NULL, 'q'},
	{"min-qp", required_argument, NULL, ARGS_ID_MIN_QP},
	{"max-qp", required_argument, NULL, ARGS_ID_MAX_QP},
	{"intra-qp-delta", required_argument, NULL, ARGS_ID_INTRA_QP_DELTA},
	{"chroma-qp-delta", required_argument, NULL, ARGS_ID_CHROMA_QP_DELTA},
	{"gop", required_argument, NULL, 'G'},
	{"slice-height", required_argument, NULL, 'S'},
	{"ir", required_argument, NULL, ARGS_ID_IR},
	{"ir-period", required_argument, NULL, ARGS_ID_IR_PERIOD},
	{"ir-length", required_argument, NULL, ARGS_ID_IR_LENGTH},
	{"base-frame", required_argument, NULL, ARGS_ID_BASE_FRAME},
	{"ref-frame", required_argument, NULL, ARGS_ID_REF_FRAME},
	{"full-range", no_argument, NULL, ARGS_ID_FULL_RANGE},
	{"color", required_argument, NULL, ARGS_ID_COLOR},
	{"transfer", required_argument, NULL, ARGS_ID_TRANSFER},
	{"matrix", required_argument, NULL, ARGS_ID_MATRIX},
	{"sar", required_argument, NULL, ARGS_ID_SAR},
	{"preferred-min-buf-count",
	 required_argument,
	 NULL,
	 ARGS_ID_MIN_BUF_COUNT},
	{"insert-pic-timing-sei",
	 no_argument,
	 NULL,
	 ARGS_ID_INSERT_PIC_TIMING_SEI},
	{"set-nri-bits", no_argument, NULL, ARGS_ID_SET_NRI_BITS},
	{0, 0, 0, 0},
};


static void welcome(char *prog_name)
{
	printf("\n%s - Video encoding program\n"
	       "Copyright (c) 2017 Parrot Drones SAS\n\n",
	       prog_name);
}


static void usage(char *prog_name)
{
	/* clang-format off */
	printf("Usage: %s [options]\n"
	       "Options:\n"
	       "  -h | --help                          "
		       "Print this message\n"
	       "       --implem <implem_name>          "
		       "Force the implementation to use\n"
	       "  -i | --infile <file_name>            "
		       "YUV input file (*.y4m or *.yuv)\n"
	       "  -f | --format <format>               "
		       "Input file data format (e.g. \"I420\", \"NV12\", "
		       "\"NV21\"...)\n"
	       "  -W | --width <width>                 "
		       "Input width in pixel units "
		       "(unused if input is *.y4m)\n"
	       "  -H | --height <height>               "
		       "Input height in pixel units "
		       "(unused if input is *.y4m)\n"
	       "  -F | --framerate <num/den>           "
		       "Input framerate; format num/den "
		       "(unused if input is *.y4m)\n"
	       "  -d | --decimation <factor>           "
		       "Framerate decimation factor\n"
	       "  -s | --start <i>                     "
		       "Start encoding at frame index i\n"
	       "  -n | --count <n>                     "
		       "Encode at most n frames\n"
	       "       --preferred-min-buf-count <n>   "
		       "Prefered minimum input buffer count\n"
	       "  -l | --loop <dir>                    "
		       "Loop forever, dir=1: loop from beginning, "
		       "dir=-1: loop alternating forward/backward\n"
	       "  -L | --fake-live                     "
		       "Fake live pipeline: schedule the "
		       "encoder at the input framerate\n"
	       "  -o | --outfile <file_name>           "
		       "H.264/H.265 Annex B byte stream output file "
		       "(.264/.h264/.265/.h265) or JPEG/MJPEG file "
		       "(.jpg/.jpeg/.mjpg/.mjpeg) or PNG file (.png)\n"
	       "  -e | --encoding <enc>                "
		       "Output encoding (e.g. \"H264\", \"H265\", "
		       "\"JPEG\", \"MJPEG\", \"PNG\"...)\n"
	       "  -r | --rc <val>                      "
		       "Rate control algorithm (e.g. \"CBR\", \"VBR\" "
		       "or \"CQ\"; default is \"CBR\")\n"
	       "  -b | --bitrate <br>                  "
		       "Bitrate (bit/s) for CBR and VBR rate-control; "
		       "default is 5000000\n"
	       "  -q | --qp <qp>                       "
		       "Based on the encoding this means:\n"
	       "              H.264 or H.265: Quantization parameter for CQ "
						      "rate-control ([1..51])\n"
	       "              JPEG: Quality factor ([1..99])\n"
	       "              PNG : Compression level ([0..9])\n"
	       "       --min-qp <qp>                   "
		       "Minimum quantization parameter ([1..51])\n"
	       "       --max-qp <qp>                   "
		       "Maximum quantization parameter ([1..51])\n"
	       "       --intra-qp-delta <qp>           "
		       "Intra quantization parameter delta ([-50..50])\n"
	       "       --chroma-qp-delta <qp>          "
		       "Chroma quantization parameter delta ([-12..12])\n"
	       "  -G | --gop <gop>                     "
		       "GOP length in seconds (float)\n"
	       "  -S | --slice-height <height>         "
		       "Slice height in macroblock units; "
		       "0 means one slice per frame\n"
	       "       --ir <mode>                     "
		       "Intra refresh mode "
		       "(e.g. \"NONE\", \"VERTICAL_SCAN\" or \"SMART SCAN\"; "
		       "default is \"NONE\")\n"
	       "       --ir-period <n>                 "
		       "Intra refresh period in frames\n"
	       "       --ir-length <n>                 "
		       "Intra refresh length in frames\n"
	       "       --base-frame <interval>         "
		       "Base frame layer interval (default is 1)\n"
	       "       --ref-frame <interval>          "
		       "Reference frame interval (default is 1)\n"
	       "       --full-range                    "
		       "Full range video (default is limited range)\n"
	       "       --color <value>                 "
		       "Color primaries (e.g. \"BT709\" or \"BT2020\")\n"
	       "       --transfer <value>              "
		       "Transfer function (e.g. \"BT709\", \"BT2020\" "
		       "or \"PQ\")\n"
	       "       --matrix <value>                "
		       "Matrix coefficients (e.g. \"BT709\" or "
		       "\"BT2020_NON_CST\")\n"
	       "       --sar <w:h>                     "
		       "Source aspect ratio; format w:h "
		       "(unused if input is *.y4m)\n"
	       "       --insert-pic-timing-sei         "
		       "Insert picture timing SEI before each frame\n"
	       "       --set-nri-bits                  "
		       "Set the H.264 NAL units header NRI bits according "
		       "to RFC6184"
	       "\n",
	       prog_name);
	/* clang-format on */
}


int main(int argc, char **argv)
{
	int res = 0, status = EXIT_SUCCESS;
	int idx, c;
	struct venc_prog *self;
	char *input = NULL, *output = NULL;
	struct vraw_reader_config reader_config;
	struct venc_input_buffer_constraints constraints;
	struct timespec cur_ts = {0, 0};
	ssize_t res1;
	size_t in_capacity;
	uint64_t start_time = 0, end_time = 0;
	int use_timer = 0;
	int auto_implem_by_encoding = 0;
	unsigned int profile = 0; /* TODO */
	unsigned int level = 0; /* TODO */
	enum venc_rate_control rate_control = VENC_RATE_CONTROL_CBR;
	unsigned int min_qp = 0;
	unsigned int max_qp = 0;
	unsigned int qp = 0;
	int intra_qp_delta = -100;
	int chroma_qp_delta = -100;
	unsigned int max_bitrate = 5000000;
	unsigned int target_bitrate = 0; /* TODO */
	unsigned int cpb_size = 0; /* TODO */
	float gop_length_sec = 0.;
	unsigned int base_frame_interval = 0;
	unsigned int ref_frame_interval = 0;
	unsigned int preferred_min_buf_count = 0;
	unsigned int slice_size_mbrows = 0;
	enum venc_entropy_coding entropy_coding =
		VENC_ENTROPY_CODING_CABAC; /* TODO */
	enum venc_intra_refresh intra_refresh = VENC_INTRA_REFRESH_NONE;
	unsigned int intra_refresh_period = 0;
	unsigned int intra_refresh_length = 0;
	int insert_ps = 1;
	int insert_aud = 0; /* TODO */
	int insert_recovery_point_sei = 0; /* TODO */
	int insert_pic_timing_sei = 0;
	int insert_mdcv_sei = 0; /* TODO */
	int insert_cll_sei = 0; /* TODO */
	int streaming_user_data_sei_version = 0; /* TODO */
	int serialize_user_data = 0;
	int rfc6184_nri_bits = 0;
	unsigned int plane_count;

	s_stopping = 0;
	s_loop = NULL;
	s_prog = NULL;

	welcome(argv[0]);

	signal(SIGINT, sighandler);

	/* Context allocation */
	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}
	s_prog = self;

	self->loop = pomp_loop_new();
	if (!self->loop) {
		ULOG_ERRNO("pomp_loop_new", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}
	s_loop = self->loop;

	memset(&self->config, 0, sizeof(self->config));
	self->config.implem = VENC_ENCODER_IMPLEM_AUTO;
	self->config.encoding = VDEF_ENCODING_H264;

	/* Default values */
	self->input.info.color_primaries = VDEF_COLOR_PRIMARIES_BT709;
	self->input.info.transfer_function = VDEF_TRANSFER_FUNCTION_BT709;
	self->input.info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;
	self->input.info.sar.height = 1;
	self->input.info.sar.width = 1;
	self->input.info.framerate.num = 30;
	self->input.info.framerate.den = 1;
	self->input.info.bit_depth = 8;
	self->input.info.full_range = false;

	/* Command-line parameters */
	while ((c = getopt_long(
			argc, argv, short_options, long_options, &idx)) != -1) {
		switch (c) {
		case 0:
			break;

		case 'h':
			usage(argv[0]);
			goto out;

		case ARGS_ID_IMPLEM:
			self->config.implem =
				venc_encoder_implem_from_str(optarg);
			break;

		case 'i':
			input = optarg;
			break;

		case 'f':
			res = vdef_raw_format_from_str(optarg,
						       &self->input.format);
			if (res != 0) {
				ULOG_ERRNO("vdef_raw_format_from_str", -res);
				usage(argv[0]);
				status = EXIT_FAILURE;
				goto out;
			}
			self->input.info.bit_depth =
				self->input.format.pix_size;
			break;

		case 'W':
			sscanf(optarg,
			       "%u",
			       &self->input.info.resolution.width);
			break;

		case 'H':
			sscanf(optarg,
			       "%u",
			       &self->input.info.resolution.height);
			break;

		case 'F':
			sscanf(optarg,
			       "%u/%u",
			       &self->input.info.framerate.num,
			       &self->input.info.framerate.den);
			break;

		case 'd':
			sscanf(optarg, "%u", &self->input.decimation);
			break;

		case 's':
			sscanf(optarg, "%d", &self->input.start_index);
			break;

		case 'n':
			sscanf(optarg, "%d", &self->input.max_count);
			break;

		case 'l':
			sscanf(optarg, "%d", &self->input.loop);
			break;

		case 'L':
			use_timer = 1;
			break;

		case 'o':
			output = optarg;
			break;

		case 'e':
			self->config.encoding = vdef_encoding_from_str(optarg);
			if (self->config.encoding == VDEF_ENCODING_UNKNOWN) {
				ULOGE("unknown encoding: '%s'", optarg);
				usage(argv[0]);
				status = EXIT_FAILURE;
				goto out;
			}
			auto_implem_by_encoding = 1;
			break;

		case 'r':
			rate_control = venc_rate_control_from_str(optarg);
			break;

		case 'b':
			sscanf(optarg, "%d", &max_bitrate);
			break;

		case 'q':
			sscanf(optarg, "%d", &qp);
			break;

		case ARGS_ID_MIN_QP:
			sscanf(optarg, "%d", &min_qp);
			break;

		case ARGS_ID_MAX_QP:
			sscanf(optarg, "%d", &max_qp);
			break;

		case ARGS_ID_INTRA_QP_DELTA:
			sscanf(optarg, "%d", &intra_qp_delta);
			break;

		case ARGS_ID_CHROMA_QP_DELTA:
			sscanf(optarg, "%d", &chroma_qp_delta);
			break;

		case 'G':
			sscanf(optarg, "%f", &gop_length_sec);
			break;

		case 'S':
			sscanf(optarg, "%u", &slice_size_mbrows);
			break;

		case ARGS_ID_IR:
			intra_refresh = venc_intra_refresh_from_str(optarg);
			break;

		case ARGS_ID_IR_PERIOD:
			sscanf(optarg, "%u", &intra_refresh_period);
			break;

		case ARGS_ID_IR_LENGTH:
			sscanf(optarg, "%u", &intra_refresh_length);
			break;

		case ARGS_ID_BASE_FRAME:
			sscanf(optarg, "%u", &base_frame_interval);
			break;

		case ARGS_ID_REF_FRAME:
			sscanf(optarg, "%u", &ref_frame_interval);
			break;

		case ARGS_ID_FULL_RANGE:
			self->input.info.full_range = 1;
			break;

		case ARGS_ID_COLOR:
			self->input.info.color_primaries =
				vdef_color_primaries_from_str(optarg);
			break;

		case ARGS_ID_TRANSFER:
			self->input.info.transfer_function =
				vdef_transfer_function_from_str(optarg);
			break;

		case ARGS_ID_MATRIX:
			self->input.info.matrix_coefs =
				vdef_matrix_coefs_from_str(optarg);
			break;

		case ARGS_ID_SAR:
			sscanf(optarg,
			       "%u:%u",
			       &self->input.info.sar.width,
			       &self->input.info.sar.height);
			break;

		case ARGS_ID_MIN_BUF_COUNT:
			sscanf(optarg, "%u", &preferred_min_buf_count);
			break;

		case ARGS_ID_INSERT_PIC_TIMING_SEI:
			insert_pic_timing_sei = 1;
			break;

		case ARGS_ID_SET_NRI_BITS:
			rfc6184_nri_bits = 1;
			break;

		default:
			usage(argv[0]);
			status = EXIT_FAILURE;
			goto out;
		}
	}

	if (auto_implem_by_encoding &&
	    self->config.implem == VENC_ENCODER_IMPLEM_AUTO)
		self->config.implem =
			venc_get_auto_implem_by_encoding(self->config.encoding);

	/* Input file */
	switch (self->config.implem) {
	case VENC_ENCODER_IMPLEM_FAKEH264:
		printf("Input: none (fake)\n");
		printf("Dimensions: %dx%d\n",
		       self->input.info.resolution.width,
		       self->input.info.resolution.height);
		printf("Framerate: %d/%d\n",
		       self->input.info.framerate.num,
		       self->input.info.framerate.den);
		if (vdef_dim_is_null(&self->input.info.resolution)) {
			ULOGE("invalid width/height");
			status = EXIT_FAILURE;
			goto out;
		}
		if (vdef_frac_is_null(&self->input.info.framerate)) {
			ULOGE("invalid framerate");
			status = EXIT_FAILURE;
			goto out;
		}
		self->input.fake_ts = self->input.info.framerate.den;
		self->input.format = vdef_i420;
		break;

	default:
		if (!input) {
			usage(argv[0]);
			status = EXIT_FAILURE;
			goto out;
		}
		memset(&reader_config, 0, sizeof(reader_config));
		reader_config.start_index = self->input.start_index;
		reader_config.max_count = self->input.max_count;
		reader_config.loop = self->input.loop;
		reader_config.format = self->input.format;
		reader_config.info = self->input.info;

		res = venc_get_input_buffer_constraints(self->config.implem,
							&reader_config.format,
							&constraints);
		if (res < 0) {
			ULOG_ERRNO("venc_get_input_buffer_constraints", -res);
			status = EXIT_FAILURE;
			goto out;
		} else {
			plane_count = vdef_get_raw_frame_plane_count(
				&reader_config.format);
			memcpy(reader_config.plane_stride_align,
			       constraints.plane_stride_align,
			       plane_count *
				       sizeof(*constraints.plane_stride_align));
			memcpy(reader_config.plane_scanline_align,
			       constraints.plane_scanline_align,
			       plane_count *
				       sizeof(*constraints
						       .plane_scanline_align));
			memcpy(reader_config.plane_size_align,
			       constraints.plane_size_align,
			       plane_count *
				       sizeof(*constraints.plane_size_align));
		}

		if ((strlen(input) > 4) &&
		    (strcmp(input + strlen(input) - 4, ".y4m") == 0))
			reader_config.y4m = 1;

		res = vraw_reader_new(
			input, &reader_config, &self->input.reader);
		if (res < 0) {
			ULOG_ERRNO("vraw_reader_new", -res);
			status = EXIT_FAILURE;
			goto out;
		}

		res = vraw_reader_get_config(self->input.reader,
					     &reader_config);
		if (res < 0) {
			ULOG_ERRNO("vraw_reader_get_config", -res);
			status = EXIT_FAILURE;
			goto out;
		}

		self->input.loop = reader_config.loop;
		self->input.format = reader_config.format;
		self->input.info = reader_config.info;

		printf("Input: file '%s'\n", input);
		printf("Format: " VDEF_RAW_FORMAT_TO_STR_FMT "\n",
		       VDEF_RAW_FORMAT_TO_STR_ARG(&self->input.format));
		printf("Bit depth: %d bits, full range: %d\n",
		       reader_config.info.bit_depth,
		       reader_config.info.full_range);
		printf("Color primaries: %s, transfer function: %s, "
		       "matrix coefficients: %s\n",
		       vdef_color_primaries_to_str(
			       reader_config.info.color_primaries),
		       vdef_transfer_function_to_str(
			       reader_config.info.transfer_function),
		       vdef_matrix_coefs_to_str(
			       reader_config.info.matrix_coefs));
		printf("Dimensions: %dx%d\n",
		       self->input.info.resolution.width,
		       self->input.info.resolution.height);
		printf("Framerate: %d/%d\n",
		       self->input.info.framerate.num,
		       self->input.info.framerate.den);
		printf("SAR: %d:%d\n",
		       self->input.info.sar.width,
		       self->input.info.sar.height);
		printf("\n");
		break;
	}

	/* Output file */
	if (output) {
		self->output.file = fopen(output, "wb");
		if (!self->output.file) {
			ULOGE("failed to open file '%s'", output);
			status = EXIT_FAILURE;
			goto out;
		}

		printf("Output: file '%s'\n", output);
	}
	self->output.encoding = self->config.encoding;
	self->config.output.preferred_format =
		VDEF_CODED_DATA_FORMAT_BYTE_STREAM;

	if (self->input.decimation == 0)
		self->input.decimation = 1;

	/* Initialize the encoder */
	self->config.input.format = self->input.format;
	self->config.input.info = self->input.info;
	self->config.input.preferred_min_buf_count = preferred_min_buf_count;

	switch (self->config.encoding) {
	case VDEF_ENCODING_H264:
		self->config.h264.profile = profile;
		self->config.h264.level = level;
		self->config.h264.rate_control = rate_control;
		self->config.h264.min_qp = min_qp;
		self->config.h264.max_qp = max_qp;
		self->config.h264.qp = qp;
		self->config.h264.intra_qp_delta = intra_qp_delta;
		self->config.h264.chroma_qp_delta = chroma_qp_delta;
		self->config.h264.max_bitrate = max_bitrate;
		self->config.h264.target_bitrate = target_bitrate;
		self->config.h264.cpb_size = cpb_size;
		self->config.h264.gop_length_sec = gop_length_sec;
		self->config.h264.decimation = self->input.decimation;
		self->config.h264.base_frame_interval = base_frame_interval;
		self->config.h264.ref_frame_interval = ref_frame_interval;
		self->config.h264.slice_size_mbrows = slice_size_mbrows;
		self->config.h264.entropy_coding = entropy_coding;
		self->config.h264.intra_refresh = intra_refresh;
		self->config.h264.intra_refresh_period = intra_refresh_period;
		self->config.h264.intra_refresh_length = intra_refresh_length;
		self->config.h264.insert_ps = insert_ps;
		self->config.h264.insert_aud = insert_aud;
		self->config.h264.insert_recovery_point_sei =
			insert_recovery_point_sei;
		self->config.h264.insert_pic_timing_sei = insert_pic_timing_sei;
		self->config.h264.streaming_user_data_sei_version =
			streaming_user_data_sei_version;
		self->config.h264.serialize_user_data = serialize_user_data;
		self->config.h264.rfc6184_nri_bits = rfc6184_nri_bits;
		break;
	case VDEF_ENCODING_H265:
		self->config.h265.profile = profile;
		self->config.h265.level = level;
		self->config.h265.rate_control = rate_control;
		self->config.h265.min_qp = min_qp;
		self->config.h265.max_qp = max_qp;
		self->config.h265.qp = qp;
		self->config.h265.intra_qp_delta = intra_qp_delta;
		self->config.h265.chroma_qp_delta = chroma_qp_delta;
		self->config.h265.max_bitrate = max_bitrate;
		self->config.h265.target_bitrate = target_bitrate;
		self->config.h265.cpb_size = cpb_size;
		self->config.h265.gop_length_sec = gop_length_sec;
		self->config.h265.decimation = self->input.decimation;
		self->config.h265.insert_ps = insert_ps;
		self->config.h265.insert_aud = insert_aud;
		self->config.h265.insert_recovery_point_sei =
			insert_recovery_point_sei;
		self->config.h265.insert_time_code_sei = insert_pic_timing_sei;
		self->config.h265.insert_mdcv_sei = insert_mdcv_sei;
		self->config.h265.insert_cll_sei = insert_cll_sei;
		self->config.h265.streaming_user_data_sei_version =
			streaming_user_data_sei_version;
		self->config.h265.serialize_user_data = serialize_user_data;
		break;
	case VDEF_ENCODING_MJPEG:
		self->config.output.preferred_format =
			VDEF_CODED_DATA_FORMAT_JFIF;
		self->config.mjpeg.rate_control = rate_control;
		self->config.mjpeg.quality = qp;
		self->config.mjpeg.max_bitrate = max_bitrate;
		self->config.mjpeg.target_bitrate = target_bitrate;
		break;
	case VDEF_ENCODING_PNG:
		self->config.output.preferred_format =
			VDEF_CODED_DATA_FORMAT_UNKNOWN;
		self->config.png.compression_level = qp;
		break;
	default:
		break;
	}

	res = venc_new(
		self->loop, &self->config, &venc_cbs, self, &self->encoder);
	if (res < 0) {
		ULOG_ERRNO("venc_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Input buffer pool */
	self->input.pool = venc_get_input_buffer_pool(self->encoder);
	if (self->input.pool == NULL) {
		switch (self->config.implem) {
		case VENC_ENCODER_IMPLEM_FAKEH264:
			in_capacity = 0;
			break;
		default:
			res1 = vraw_reader_get_min_buf_size(self->input.reader);
			if (res1 < 0) {
				ULOG_ERRNO("vraw_reader_get_min_buf_size",
					   (int)-res1);
				status = EXIT_FAILURE;
				goto out;
			}
			in_capacity = res1;
			break;
		}

		res = mbuf_pool_new(mbuf_mem_generic_impl,
				    in_capacity,
				    DEFAULT_IN_BUF_COUNT,
				    MBUF_POOL_SMART_GROW,
				    0,
				    "venc_default_pool",
				    &self->input.pool);
		if (res < 0) {
			ULOG_ERRNO("mbuf_pool_new:input", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		self->input.pool_allocated = 1;
	}

	/* Input buffer pool fd event */
	self->input.pool_evt = pomp_evt_new();
	if (self->input.pool_evt == NULL) {
		ULOG_ERRNO("pomp_evt_new", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}
	res = pomp_evt_attach_to_loop(
		self->input.pool_evt, self->loop, &pool_event_cb, self);
	if (res < 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Input buffer queue */
	self->input.queue = venc_get_input_buffer_queue(self->encoder);
	if (self->input.queue == NULL) {
		ULOG_ERRNO("venc_get_input_buffer_queue", EPROTO);
		status = EXIT_FAILURE;
		goto out;
	}

	self->input.frame_info.format = self->input.format;
	self->input.frame_info.info.resolution.height =
		self->input.info.resolution.height;
	self->input.frame_info.info.resolution.width =
		self->input.info.resolution.width;

	/* Start */
	if (use_timer) {
		self->encoder_timer =
			pomp_timer_new(self->loop, encoder_timer_cb, self);
		if (!self->encoder_timer) {
			ULOGE("pomp_timer_new failed");
			status = EXIT_FAILURE;
			goto out;
		}
		uint32_t frame_time_ms = 1000 * self->input.info.framerate.den /
					 self->input.info.framerate.num;
		if (frame_time_ms == 0)
			frame_time_ms = 1;
		res = pomp_timer_set_periodic(
			self->encoder_timer, 1, frame_time_ms);
		if (res != 0) {
			ULOG_ERRNO("pomp_timer_set_periodic", -res);
			status = EXIT_FAILURE;
			goto out;
		}
	} else {
		res = pomp_loop_idle_add(self->loop, encode_frame_idle, self);
		if (res < 0) {
			ULOG_ERRNO("pomp_loop_idle_add", -res);
			status = EXIT_FAILURE;
			goto out;
		}
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &start_time);

	while (!self->stopped) {
		res = pomp_loop_wait_and_process(self->loop, -1);
		if (res)
			ULOG_ERRNO("pomp_loop_wait_and_process", -res);
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &end_time);
	printf("\nOverall time: %.2fs / %.2ffps\n",
	       (float)(end_time - start_time) / 1000000.,
	       self->output.count * 1000000. / (float)(end_time - start_time));

out:
	/* Cleanup */
	if (self) {
		if (self->encoder_timer) {
			pomp_timer_clear(self->encoder_timer);
			res = pomp_timer_destroy(self->encoder_timer);
			if (res < 0)
				ULOG_ERRNO("pomp_timer_destroy", -res);
		}

		if (self->input.reader) {
			res = vraw_reader_destroy(self->input.reader);
			if (res < 0)
				ULOG_ERRNO("vraw_reader_destroy", -res);
		}

		if (self->input.pool_evt != NULL) {
			if (pomp_evt_is_attached(self->input.pool_evt,
						 self->loop)) {
				res = pomp_evt_detach_from_loop(
					self->input.pool_evt, self->loop);
				if (res < 0)
					ULOG_ERRNO("pomp_evt_detach_from_loop",
						   -res);
			}

			res = pomp_evt_destroy(self->input.pool_evt);
			if (res < 0)
				ULOG_ERRNO("pomp_evt_destroy", -res);
		}

		if (self->input.pool_allocated) {
			res = mbuf_pool_destroy(self->input.pool);
			if (res < 0)
				ULOG_ERRNO("mbuf_pool_destroy:input", -res);
		}

		if (self->encoder) {
			res = venc_destroy(self->encoder);
			if (res < 0)
				ULOG_ERRNO("venc_destroy", -res);
		}

		if (self->loop) {
			res = pomp_loop_destroy(self->loop);
			if (res < 0)
				ULOG_ERRNO("pomp_loop_destroy", -res);
		}

		if (self->output.file)
			fclose(self->output.file);
	}

	printf("\n%s\n", (status == EXIT_SUCCESS) ? "Finished!" : "Failed!");
	exit(status);
}
