
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

# API library. This is the library that most programs should use.
LOCAL_MODULE := libvideo-encode
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video encoding library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DVENC_API_EXPORTS -fvisibility=hidden -std=gnu11 -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	src/venc.c
LOCAL_LIBRARIES := \
	libpomp \
	libulog \
	libvideo-defs \
	libvideo-encode-core
LOCAL_CONFIG_FILES := config.in
$(call load-config)
LOCAL_CONDITIONAL_LIBRARIES := \
	CONFIG_VENC_FAKEH264:libvideo-encode-fakeh264 \
	CONFIG_VENC_FFMPEG:libvideo-encode-ffmpeg \
	CONFIG_VENC_HISI:libvideo-encode-hisi \
	CONFIG_VENC_QCOM:libvideo-encode-qcom \
	CONFIG_VENC_QCOM_JPEG:libvideo-encode-qcom-jpeg \
	CONFIG_VENC_MEDIACODEC:libvideo-encode-mediacodec \
	CONFIG_VENC_VIDEOTOOLBOX:libvideo-encode-videotoolbox \
	CONFIG_VENC_TURBOJPEG:libvideo-encode-turbojpeg \
	CONFIG_VENC_X264:libvideo-encode-x264 \
	CONFIG_VENC_X265:libvideo-encode-x265 \
	CONFIG_VENC_PNG:libvideo-encode-png
LOCAL_EXPORT_LDLIBS := -lvideo-encode-core

include $(BUILD_LIBRARY)

include $(CLEAR_VARS)

# Core library, common code for all implementations and structures definitions.
# Used by implementations.
LOCAL_MODULE := libvideo-encode-core
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video encoding library: core files
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/core/include
LOCAL_CFLAGS := -DVENC_API_EXPORTS -fvisibility=hidden -std=gnu11 -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	core/src/venc_core.c \
	core/src/venc_h264.c \
	core/src/venc_h265.c
LOCAL_LIBRARIES := \
	libfutils \
	libh264 \
	libh265 \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpomp \
	libulog \
	libvideo-defs \
	libvideo-metadata \
	libvideo-streaming

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)

include $(CLEAR_VARS)

# ffmpeg implementation. can be enabled in the product configuration.
LOCAL_MODULE := libvideo-encode-ffmpeg
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video encoding library: ffmpeg implementation
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/ffmpeg/include
LOCAL_CFLAGS := -DVENC_API_EXPORTS -fvisibility=hidden -std=gnu99 -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	ffmpeg/src/venc_ffmpeg_convert.c \
	ffmpeg/src/venc_ffmpeg.c
LOCAL_LIBRARIES := \
	ffmpeg-libav \
	libfutils \
	libh264 \
	libh265 \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpomp \
	libulog \
	libvideo-defs \
	libvideo-encode-core \
	libvideo-metadata \
	libvideo-streaming \

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)

ifeq ("${TARGET_OS}", "darwin")
include $(CLEAR_VARS)

LOCAL_MODULE := libvideo-encode-videotoolbox
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video encoding library: Videotoolbox implementation
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/videotoolbox/include
LOCAL_CFLAGS := -DVENC_API_EXPORTS -fvisibility=hidden -std=gnu11
LOCAL_SRC_FILES := \
	videotoolbox/src/venc_videotoolbox.c
LOCAL_LIBRARIES := \
	libfutils \
	libpomp \
	libulog \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-cvpixelbuffer \
	libmedia-buffers-memory-generic \
	libvideo-defs \
	libvideo-encode-core
LOCAL_LDLIBS += \
	-framework Foundation \
	-framework CoreMedia \
	-framework CoreVideo \
	-framework VideoToolbox

include $(BUILD_LIBRARY)
endif

ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-android")
include $(CLEAR_VARS)

LOCAL_MODULE := libvideo-encode-mediacodec
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video encoding library: MediaCodec implementation
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/mediacodec/include
LOCAL_CFLAGS := -DVENC_API_EXPORTS -fvisibility=hidden -std=gnu11
LOCAL_LDLIBS := -lmediandk
LOCAL_SRC_FILES := \
	mediacodec/src/venc_mediacodec.c
LOCAL_LIBRARIES := \
	libfutils \
	libpomp \
	libulog \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libvideo-defs \
	libvideo-encode-core \
	libvideo-metadata


include $(BUILD_LIBRARY)

endif

include $(CLEAR_VARS)

# turbojpeg implementation. can be enabled in the product configuration.
LOCAL_MODULE := libvideo-encode-turbojpeg
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video encoding library: turbojpeg implementation
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/turbojpeg/include
LOCAL_CFLAGS := -DVENC_API_EXPORTS -fvisibility=hidden -std=gnu11 -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	turbojpeg/src/venc_turbojpeg.c
LOCAL_LIBRARIES := \
	libfutils \
	libjpeg-turbo \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpomp \
	libulog \
	libvideo-defs \
	libvideo-encode-core \
	libvideo-metadata \
	libvideo-streaming

include $(BUILD_LIBRARY)

include $(CLEAR_VARS)

# png implementation. can be enabled in the product configuration.
LOCAL_MODULE := libvideo-encode-png
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video encoding library: png implementation
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/png/include
LOCAL_CFLAGS := -DVENC_API_EXPORTS -fvisibility=hidden -std=gnu11 -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	png/src/venc_png.c

LOCAL_LIBRARIES := \
	libfutils \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpomp \
	libpng \
	libulog \
	libvideo-defs \
	libvideo-encode-core \
	libvideo-metadata \
	libvideo-streaming

include $(BUILD_LIBRARY)

include $(CLEAR_VARS)

# x264 implementation. can be enabled in the product configuration.
LOCAL_MODULE := libvideo-encode-x264
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video encoding library: x264 implementation
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/x264/include
LOCAL_CFLAGS := -DVENC_API_EXPORTS -fvisibility=hidden -std=gnu11 -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	x264/src/venc_x264.c
LOCAL_LIBRARIES := \
	libfutils \
	libh264 \
	libpomp \
	libulog \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libvideo-defs \
	libvideo-encode-core \
	libvideo-metadata \
	libvideo-streaming \
	x264

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)

include $(CLEAR_VARS)

# x265 implementation. can be enabled in the product configuration.
LOCAL_MODULE := libvideo-encode-x265
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video encoding library: x265 implementation
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/x265/include
LOCAL_CFLAGS := -DVENC_API_EXPORTS -fvisibility=hidden -std=gnu11 -D_GNU_SOURCE
LOCAL_LDLIBS := -ldl
LOCAL_SRC_FILES := \
	x265/src/venc_x265.c
LOCAL_LIBRARIES := \
	libfutils \
	libh265 \
	libpomp \
	libulog \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libvideo-defs \
	libvideo-encode-core \
	libvideo-metadata \
	libvideo-streaming \
	x265
LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:x265-10bit

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)

ifneq ($(TARGET_OS_FLAVOUR),$(filter %$(TARGET_OS_FLAVOUR),android iphoneos iphonesimulator))
include $(CLEAR_VARS)

LOCAL_MODULE := venc
LOCAL_DESCRIPTION := Video encoding program
LOCAL_CATEGORY_PATH := multimedia
LOCAL_SRC_FILES := tools/venc.c
LOCAL_LIBRARIES := \
	libfutils \
	libh264 \
	libh265 \
	libpomp \
	libulog \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libvideo-defs \
	libvideo-encode \
	libvideo-raw

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_EXECUTABLE)
endif

ifdef TARGET_TEST

include $(CLEAR_VARS)
LOCAL_MODULE := tst-libvideo-encode
LOCAL_CFLAGS += -DTARGET_TEST -D_GNU_SOURCE
# Mirrors the BUILD_LIBVIDEO_ENCODE_<NAME> macros Alchemy auto-derives for
# libvideo-encode's own LOCAL_CONDITIONAL_LIBRARIES (src/venc_priv.h): this
# test module links these backends directly (not through that conditional
# mechanism), so the same defines have to be set by hand from the identical
# CONFIG_VENC_<NAME> Kconfig booleans, already loaded earlier in this file.
# Needed so tests/venc_test_config.c can conditionally #include each
# backend's implementation-specific config header (venc_x264.h/venc_x265.h/
# venc_ffmpeg.h) only when that backend is actually part of the build -
# otherwise the header (only exported transitively when its backend is
# linked into libvideo-encode) is missing and the build fails.
ifeq ("$(CONFIG_VENC_X264)","y")
LOCAL_CFLAGS += -DBUILD_LIBVIDEO_ENCODE_X264
endif
ifeq ("$(CONFIG_VENC_X265)","y")
LOCAL_CFLAGS += -DBUILD_LIBVIDEO_ENCODE_X265
endif
ifeq ("$(CONFIG_VENC_FFMPEG)","y")
LOCAL_CFLAGS += -DBUILD_LIBVIDEO_ENCODE_FFMPEG
endif
LOCAL_SRC_FILES := \
	tests/venc_test.c \
	tests/venc_test_core.c \
	tests/venc_test_h264.c \
	tests/venc_test_h265.c \
	tests/venc_test_config.c \
	tests/venc_test_lifecycle.c
LOCAL_LIBRARIES := \
	libcunit \
	libvideo-encode \
	libvideo-encode-core \
	libh264 \
	libh265 \
	libpomp \
	libulog \
	libvideo-defs \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libvideo-metadata
include $(BUILD_EXECUTABLE)

endif
