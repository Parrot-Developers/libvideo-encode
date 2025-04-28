# libvideo-encode - Video encoding library

_libvideo-encode_ is a C library that provides a common API for encoding video
in H.264 and H.265, and photos in JPEG and PNG across various platforms.

The library uses hardware-accelerated encoding when available.

## Implementations

The following implementations are available:

* MediaCodec on Android 4.2+ (through the NDK API on Android 5.0+
or the Java API through JNI)
* PNG (software encoding)
* TurboJPEG using libjpeg-turbo
* VideoToolbox on iOS 8+ and MacOS X
* x264 (software encoding)
* x265 (software encoding)

The application can force using a specific implementation or let the library
decide according to what is supported by the platform.

## Dependencies

The library depends on the following Alchemy modules:

* libfutils
* libh264
* libh265
* libmedia-buffers
* libmedia-buffers-memory
* libmedia-buffers-memory-generic
* libpomp
* libulog
* libvideo-defs
* libvideo-encode-core
* libvideo-metadata
* libvideo-streaming
* (optional) libjpeg-turbo (for TurboJPEG support)
* (optional) libpng (for PNG support)
* (optional) x264 (for x264 support)
* (optional) x265 (for x265 support)
* (optional) x265-10bit (for x264, 10-bit support)

The library also depends on the following frameworks for iOS and MacOS only:

* Foundation
* CoreMedia
* CoreVideo
* VideoToolbox

## Building

Building is activated by enabling _libvideo-encode_ in the Alchemy build
configuration.

## Operation

Some encoders need the input buffers to originate from their own buffer
pool; when the input buffer pool returned by the library is not _NULL_ it must
be used and input buffers cannot be shared with other video pipeline elements.

The encoding is asynchronous: the application pushes buffers to encode in the
input queue and is notified of encoded frames through a callback function.

### Threading model

The library is designed to run on a _libpomp_ event loop (_pomp_loop_, see
_libpomp_ documentation). All API functions must be called from the _pomp_loop_
thread. All callback functions (frame_output, flush or stop) are called from
the _pomp_loop_ thread.

## Testing

The library can be tested using the provided _venc_ command-line tool which
takes as input a raw YUV file and can can optionally generate an encoded H.264
or H.265 bitstream, as well as JPEG or PNG photos, depending on the encoder
implementation used.

To build the tool, enable _venc_ in the Alchemy build configuration.

For a list of available options, run

    $ venc -h
