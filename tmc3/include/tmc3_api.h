/* The copyright in this software is being made available under the BSD
 * Licence, included below.  This software may be subject to other third
 * party and contributor rights, including patent rights, and no such
 * rights are granted under this licence.
 *
 * Copyright (c) 2026, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the ISO/IEC nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * C interface to the TMC13 encoder and decoder, for embedding the codec in
 * other applications and languages (eg, Python via ctypes or cffi).
 *
 * Point data is described by strided arrays (tmc3_array) that point at the
 * caller's memory, so that arrays from other environments (numpy arrays,
 * including structured arrays, DLPack tensors, ...) are passed without
 * conversion.  Decoded points are returned as arrays that point at the
 * codec's own storage, owned by a frame object that the caller releases.
 *
 * See doc/README.api.md.
 */

#ifndef TMC3_API_H
#define TMC3_API_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#  if defined(TMC3_API_BUILD)
#    define TMC3_API __declspec(dllexport)
#  elif defined(TMC3_API_STATIC)
#    define TMC3_API
#  else
#    define TMC3_API __declspec(dllimport)
#  endif
#else
#  define TMC3_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Version of this interface: changes when the ABI changes */
#define TMC3_API_VERSION 1

/*--------------------------------------------------------------------------*/
/* Status codes */

typedef enum {
  TMC3_OK = 0,
  TMC3_NO_FRAME = 1,          /* tmc3_decoder_next_frame: no frame ready */

  TMC3_ERR_ARGUMENT = -1,     /* invalid argument */
  TMC3_ERR_OPTIONS = -2,      /* invalid codec options */
  TMC3_ERR_UNSUPPORTED = -3,  /* configuration not supported by the API */
  TMC3_ERR_STATE = -4,        /* call not valid in the current state */
  TMC3_ERR_CODEC = -5,        /* encoding or decoding failed */
  TMC3_ERR_MEMORY = -6        /* out of memory */
} tmc3_status;

/*--------------------------------------------------------------------------*/
/* Point data */

typedef enum {
  TMC3_UINT8 = 1,
  TMC3_UINT16 = 2,
  TMC3_INT32 = 3,
  TMC3_FLOAT32 = 4,
  TMC3_FLOAT64 = 5
} tmc3_dtype;

/*
 * A strided array of points, each with one or three components.
 *
 * Component c of point i is at
 *   (char*)data + i * stride + offset[c]
 *
 * - stride == 0 means packed points: the number of components times the
 *   element size.
 * - offset[] all zero means packed components: offset[c] = c * element size.
 *
 * Examples (numpy):
 *   float32 array of shape (N, 3):        stride 0, offsets 0
 *   structured array (x, y, z f4, i u2):  stride = itemsize,
 *                                         offset = field offsets
 *   column-major (N, 3) float32:          stride 4, offset {0, 4N, 8N}
 */
typedef struct {
  void* data;           /* NULL: absent */
  tmc3_dtype dtype;
  int64_t stride;       /* bytes between consecutive points */
  int64_t offset[3];    /* byte offset of each component */
} tmc3_array;

/*
 * A point cloud to encode.  The arrays are only read, during the call.
 *  - position:    x, y, z; any dtype; required
 *  - colour:      r, g, b; TMC3_UINT8 or TMC3_UINT16; when coded
 *  - reflectance: one component; TMC3_UINT8 or TMC3_UINT16; when coded
 */
typedef struct {
  int64_t num_points;
  tmc3_array position;
  tmc3_array colour;
  tmc3_array reflectance;
} tmc3_cloud;

/*--------------------------------------------------------------------------*/
/* Opaque objects */

typedef struct tmc3_encoder tmc3_encoder;
typedef struct tmc3_decoder tmc3_decoder;
typedef struct tmc3_packet tmc3_packet; /* the coded data of one frame */
typedef struct tmc3_frame tmc3_frame;   /* one decoded frame */

/*--------------------------------------------------------------------------*/
/* Library */

/* The software version of the codec */
TMC3_API const char* tmc3_version(void);

/* TMC3_API_VERSION of the library */
TMC3_API int tmc3_api_version(void);

/*
 * The message of the last failed call on the calling thread (errors and
 * warnings from option parsing, codec errors), or "".
 */
TMC3_API const char* tmc3_last_error(void);

/*
 * The number of threads used by the codec in this process, including the
 * calling thread (1: no threads, the default).  The output does not depend
 * on the number of threads.  The "numThreads" option of a codec raises
 * this number when the codec is created.
 */
TMC3_API tmc3_status tmc3_set_num_threads(int num_threads);

/*--------------------------------------------------------------------------*/
/*
 * Encoder
 *
 * Options are the tmc3 command line options, as "name=value" strings
 * (eg, "positionQuantizationScale=0.5", "config=lidar.cfg"); a leading
 * "--" is optional.  The mode and file paths cannot be set.  In addition:
 *   "log=quiet" (default), "log=stdout", "log=stderr":
 *     the destination of the codec's messages.
 *
 * Frames are coded in the order they are passed.  Bi-prediction, which
 * codes frames out of order, is not supported.
 */

TMC3_API tmc3_status tmc3_encoder_create(
  const char* const* options, int num_options, tmc3_encoder** encoder);

/*
 * Encode one frame.
 *  - packet: receives the coded data, in the same format as the files
 *    written by tmc3; packets concatenated in order form the bitstream.
 *  - reconstruction: if not NULL, receives the encoder's reconstruction of
 *    the frame (as decoded).
 */
TMC3_API tmc3_status tmc3_encoder_encode(
  tmc3_encoder* encoder,
  const tmc3_cloud* cloud,
  tmc3_packet** packet,
  tmc3_frame** reconstruction);

TMC3_API void tmc3_encoder_destroy(tmc3_encoder* encoder);

/* The coded data: valid until the packet is released */
TMC3_API const void* tmc3_packet_data(const tmc3_packet* packet);
TMC3_API size_t tmc3_packet_size(const tmc3_packet* packet);
TMC3_API void tmc3_packet_release(tmc3_packet* packet);

/*--------------------------------------------------------------------------*/
/*
 * Decoder
 *
 * Options as for the encoder (decoder options only).
 *
 * Coded data is passed with tmc3_decoder_push in pieces of any size.
 * Decoded frames are queued and returned by tmc3_decoder_next_frame.
 *
 * A frame is complete when the data of the next frame arrives, at the end
 * of the stream (tmc3_decoder_flush), or when the caller marks the end of
 * the frame (tmc3_decoder_end_frame), eg. after pushing a packet produced
 * by tmc3_encoder_encode, which avoids waiting for the next frame.
 */

TMC3_API tmc3_status tmc3_decoder_create(
  const char* const* options, int num_options, tmc3_decoder** decoder);

/* Decode data (the data is only read during the call) */
TMC3_API tmc3_status
tmc3_decoder_push(tmc3_decoder* decoder, const void* data, size_t size);

/*
 * The data pushed so far completes the current frame: output it now.
 * Has no effect with bi-prediction, where frames are output in order.
 */
TMC3_API tmc3_status tmc3_decoder_end_frame(tmc3_decoder* decoder);

/* End of the stream: output any remaining frame */
TMC3_API tmc3_status tmc3_decoder_flush(tmc3_decoder* decoder);

/* The next decoded frame, or TMC3_NO_FRAME */
TMC3_API tmc3_status
tmc3_decoder_next_frame(tmc3_decoder* decoder, tmc3_frame** frame);

TMC3_API void tmc3_decoder_destroy(tmc3_decoder* decoder);

/*--------------------------------------------------------------------------*/
/*
 * Decoded frames
 *
 * The arrays point at the frame's storage and are valid until the frame is
 * released.  The frame may be read from any thread; it is not modified.
 */

typedef struct {
  int64_t num_points;

  /* x, y, z: TMC3_INT32, in units such that output = value * scale + origin */
  tmc3_array position;

  /* r, g, b: TMC3_UINT16, or absent (data == NULL) */
  tmc3_array colour;

  /* TMC3_UINT16, or absent */
  tmc3_array reflectance;

  /* frame number (counted from 0) */
  int32_t frame_num;

  /* the output coordinate system of the tmc3 decoder (outputScaling,
   * outputUnitLength, outputPrecisionBits options) */
  double scale;
  double origin[3];
} tmc3_frame_info;

TMC3_API tmc3_status
tmc3_frame_get_info(const tmc3_frame* frame, tmc3_frame_info* info);

/*
 * Write the positions in output units (value * scale + origin) to dst,
 * which must have frame num_points points: TMC3_FLOAT32 or TMC3_FLOAT64
 * (as written to ply files by tmc3).
 */
TMC3_API tmc3_status
tmc3_frame_get_positions(const tmc3_frame* frame, const tmc3_array* dst);

TMC3_API void tmc3_frame_release(tmc3_frame* frame);

#ifdef __cplusplus
}
#endif

#endif /* TMC3_API_H */
