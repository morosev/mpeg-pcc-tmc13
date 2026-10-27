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

#include "tmc3_api.h"

#include <cstring>
#include <deque>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <vector>

#include "PCCTMC3Decoder.h"
#include "PCCTMC3Encoder.h"
#include "app_params.h"
#include "parallel.h"
#include "pointset_processing.h"
#include "version.h"

using namespace pcc;

static_assert(sizeof(Vec3<int32_t>) == 12, "packed positions");
static_assert(sizeof(Vec3<attr_t>) == 6, "packed colours");

//============================================================================

namespace {

thread_local std::string tlLastError;

tmc3_status
fail(tmc3_status status, const std::string& message)
{
  tlLastError = message;
  return status;
}

//----------------------------------------------------------------------------
// Runs fn, converting exceptions to a status.  fn runs with a stack large
// enough for the codec, possibly on another thread: the error message is
// returned to the calling thread.

template<typename Fn>
tmc3_status
guard(Fn fn)
{
  tmc3_status status = TMC3_OK;
  std::string error;
  auto run = [&] {
    tlLastError.clear();
    try {
      status = fn();
    }
    catch (const std::bad_alloc&) {
      status = fail(TMC3_ERR_MEMORY, "out of memory");
    }
    catch (const std::exception& e) {
      status = fail(TMC3_ERR_CODEC, e.what());
    }
    catch (...) {
      status = fail(TMC3_ERR_CODEC, "unknown error");
    }
    error = tlLastError;
  };

  try {
    runWithCodingStack(run);
  }
  catch (const std::exception& e) {
    status = TMC3_ERR_MEMORY;
    error = e.what();
  }

  tlLastError = error;
  return status;
}

//----------------------------------------------------------------------------
// Process-wide thread count: only raised by codec options

std::mutex gThreadsMutex;

void
raiseNumThreads(int numThreads)
{
  std::lock_guard<std::mutex> lock(gThreadsMutex);
  if (numThreads > pcc::numThreads())
    pcc::setNumThreads(numThreads);
}

//----------------------------------------------------------------------------
// Codec messages: the "log" option

struct CodecLog {
  // NB: a stream without a buffer discards its output
  std::ostream quiet{nullptr};
  std::ostream* stream = &quiet;

  // Consumes the "log" option; returns false if invalid
  bool parse(std::vector<std::string>& options, std::string& error)
  {
    for (auto it = options.begin(); it != options.end();) {
      std::string opt = it->compare(0, 2, "--") ? *it : it->substr(2);
      if (opt.compare(0, 4, "log=")) {
        ++it;
        continue;
      }

      std::string value = opt.substr(4);
      if (value == "quiet")
        stream = &quiet;
      else if (value == "stdout")
        stream = &std::cout;
      else if (value == "stderr")
        stream = &std::cerr;
      else {
        error = "invalid log option: " + value;
        return false;
      }
      it = options.erase(it);
    }
    return true;
  }
};

//----------------------------------------------------------------------------

std::vector<std::string>
toStrings(const char* const* options, int numOptions)
{
  std::vector<std::string> strings;
  for (int i = 0; i < numOptions; i++)
    if (options[i])
      strings.emplace_back(options[i]);
  return strings;
}

//----------------------------------------------------------------------------
// Array access

size_t
dtypeSize(tmc3_dtype dtype)
{
  switch (dtype) {
  case TMC3_UINT8: return 1;
  case TMC3_UINT16: return 2;
  case TMC3_INT32: return 4;
  case TMC3_FLOAT32: return 4;
  case TMC3_FLOAT64: return 8;
  }
  return 0;
}

// The resolved layout of an array with numComp components
struct Layout {
  const char* data;
  int64_t stride;
  int64_t offset[3];
};

bool
resolve(const tmc3_array& a, int numComp, Layout* layout)
{
  size_t size = dtypeSize(a.dtype);
  if (!a.data || !size)
    return false;

  bool packedComp = true;
  for (int c = 0; c < numComp; c++)
    packedComp &= a.offset[c] == 0;

  layout->data = static_cast<const char*>(a.data);
  layout->stride = a.stride ? a.stride : int64_t(size * numComp);
  for (int c = 0; c < numComp; c++)
    layout->offset[c] = packedComp ? int64_t(c * size) : a.offset[c];
  return true;
}

template<typename T>
T
load(const char* p)
{
  T val;
  std::memcpy(&val, p, sizeof(T));
  return val;
}

template<typename T>
void
store(char* p, T val)
{
  std::memcpy(p, &val, sizeof(T));
}

// Calls fn(i, value) for component c of the n points, with the value
// converted to Dst
template<typename Dst, typename Fn>
void
forEachValue(const Layout& l, tmc3_dtype dtype, int c, int64_t n, Fn fn)
{
  const char* p = l.data + l.offset[c];
  switch (dtype) {
  case TMC3_UINT8:
    for (int64_t i = 0; i < n; i++, p += l.stride)
      fn(i, Dst(load<uint8_t>(p)));
    break;
  case TMC3_UINT16:
    for (int64_t i = 0; i < n; i++, p += l.stride)
      fn(i, Dst(load<uint16_t>(p)));
    break;
  case TMC3_INT32:
    for (int64_t i = 0; i < n; i++, p += l.stride)
      fn(i, Dst(load<int32_t>(p)));
    break;
  case TMC3_FLOAT32:
    for (int64_t i = 0; i < n; i++, p += l.stride)
      fn(i, Dst(load<float>(p)));
    break;
  case TMC3_FLOAT64:
    for (int64_t i = 0; i < n; i++, p += l.stride)
      fn(i, Dst(load<double>(p)));
    break;
  }
}

//----------------------------------------------------------------------------
// The position component of the cloud (in coding axis order) that holds
// x, y, z: the ply property names of the axis order.

void
axisComponents(AxisOrder order, int component[3])
{
  auto names = axisOrderToPropertyNames(order);
  for (int k = 0; k < 3; k++)
    component[names[k][0] - 'x'] = k;
}

// Colour components (r, g, b) as stored in a cloud (g, b, r)
const int kColourComponent[3] = {2, 0, 1};

}  // namespace

//============================================================================

struct tmc3_packet {
  std::vector<char> data;
};

//----------------------------------------------------------------------------

struct tmc3_frame {
  CloudFrame frame;
  double scale;
  Vec3<double> origin;

  // Takes the cloud of src and converts the attributes for output
  tmc3_frame(const Parameters& params, CloudFrame& src)
  {
    frame.frameNum = src.frameNum;
    frame.geometry_axis_order = src.geometry_axis_order;
    frame.outputUnitLength = src.outputUnitLength;
    frame.outputUnit = src.outputUnit;
    frame.outputOrigin = src.outputOrigin;
    frame.outputFpBits = src.outputFpBits;
    frame.attrDesc = src.attrDesc;
    frame.cloud.swap(src.cloud);

    // as for ply output by tmc3
    scaleAttributesForOutput(frame.attrDesc, frame.cloud);
    if (params.convertColourspace)
      convertToGbr(frame.attrDesc, frame.cloud);

    scale = outputScale(params, frame) / (1 << frame.outputFpBits);
    origin = outputOrigin(params, frame);
  }
};

//============================================================================

struct tmc3_encoder : PCCTMC3Encoder3::Callbacks {
  Parameters params;
  PCCTMC3Encoder3 encoder;
  CodecLog log;

  // The raw origin used for input sorting (as tmc3)
  Vec3<int> angularOrigin;

  // Position component of the coded cloud holding x, y, z
  int axisComponent[3];

  int frameIndex = 0;

  // The packet being written
  tmc3_packet* packet = nullptr;

  void onOutputBuffer(const PayloadBuffer& buf) override
  {
    // as writeTlv()
    uint32_t length = uint32_t(buf.size());
    auto& out = packet->data;
    out.push_back(char(buf.type));
    out.push_back(char(length >> 24));
    out.push_back(char(length >> 16));
    out.push_back(char(length >> 8));
    out.push_back(char(length >> 0));
    out.insert(out.end(), buf.begin(), buf.end());
  }

  void onPostRecolour(const PCCPointSet3&) override {}

  tmc3_status readCloud(const tmc3_cloud& in, PCCPointSet3& cloud);
};

//----------------------------------------------------------------------------
// Reads the input into cloud, as tmc3 reads a ply file

tmc3_status
tmc3_encoder::readCloud(const tmc3_cloud& in, PCCPointSet3& cloud)
{
  const int64_t n = in.num_points;
  if (n <= 0 || n > INT32_MAX)
    return fail(TMC3_ERR_ARGUMENT, "invalid number of points");

  Layout pos, col, refl;
  if (!resolve(in.position, 3, &pos))
    return fail(TMC3_ERR_ARGUMENT, "invalid position array");

  bool codeColour = params.encoder.attributeIdxMap.count("color");
  bool codeRefl = params.encoder.attributeIdxMap.count("reflectance");

  auto attrOk = [](const tmc3_array& a) {
    return a.dtype == TMC3_UINT8 || a.dtype == TMC3_UINT16;
  };

  if (codeColour && (!resolve(in.colour, 3, &col) || !attrOk(in.colour)))
    return fail(TMC3_ERR_ARGUMENT, "colour is coded: invalid colour array");

  if (
    codeRefl
    && (!resolve(in.reflectance, 1, &refl) || !attrOk(in.reflectance)))
    return fail(
      TMC3_ERR_ARGUMENT, "reflectance is coded: invalid reflectance array");

  cloud.addRemoveAttributes(codeColour, codeRefl);
  cloud.resize(n);

  // NB: as ply::read, positions are scaled in double precision and
  //     truncated
  const double scale = params.inputScale;
  for (int k = 0; k < 3; k++) {
    int c = axisComponent[k];
    forEachValue<double>(
      pos, in.position.dtype, k, n,
      [&](int64_t i, double v) { cloud[i][c] = int32_t(v * scale); });
  }

  if (codeColour)
    for (int k = 0; k < 3; k++) {
      int c = kColourComponent[k];
      forEachValue<attr_t>(
        col, in.colour.dtype, k, n,
        [&](int64_t i, attr_t v) { cloud.getColor(i)[c] = v; });
    }

  if (codeRefl)
    forEachValue<attr_t>(
      refl, in.reflectance.dtype, 0, n,
      [&](int64_t i, attr_t v) { cloud.getReflectance(i) = v; });

  return TMC3_OK;
}

//============================================================================

struct tmc3_decoder : PCCTMC3Decoder3::Callbacks {
  Parameters params;
  std::unique_ptr<PCCTMC3Decoder3> decoder;
  CodecLog log;

  // An incomplete data unit, completed by the next push
  std::vector<char> partial;

  // Decoded frames not yet returned
  std::deque<std::unique_ptr<tmc3_frame>> frames;

  bool flushed = false;

  void onOutputCloud(CloudFrame& frame) override
  {
    frames.emplace_back(new tmc3_frame(params, frame));
  }

  // Decodes the complete data units in [begin, end); returns the first
  // byte not consumed.
  const char* decode(const char* begin, const char* end);
};

//----------------------------------------------------------------------------

const char*
tmc3_decoder::decode(const char* p, const char* end)
{
  // as readTlv(): type, then 32-bit big-endian length
  PayloadBuffer buf;
  while (end - p >= 5) {
    uint32_t length = 0;
    for (int i = 1; i <= 4; i++)
      length = (length << 8) | uint8_t(p[i]);
    if (uint64_t(end - p - 5) < length)
      break;

    buf.type = PayloadType(uint8_t(p[0]));
    buf.assign(p + 5, p + 5 + length);
    if (decoder->decompress(&buf, this))
      throw std::runtime_error("can't decompress point cloud");
    p += 5 + length;
  }
  return p;
}

//============================================================================
// Library

extern "C" {

const char*
tmc3_version(void)
{
  return pcc::version;
}

//----------------------------------------------------------------------------

int
tmc3_api_version(void)
{
  return TMC3_API_VERSION;
}

//----------------------------------------------------------------------------

const char*
tmc3_last_error(void)
{
  return tlLastError.c_str();
}

//----------------------------------------------------------------------------

tmc3_status
tmc3_set_num_threads(int num_threads)
{
  return guard([&] {
    if (num_threads < 1)
      return fail(TMC3_ERR_ARGUMENT, "invalid number of threads");
    std::lock_guard<std::mutex> lock(gThreadsMutex);
    pcc::setNumThreads(num_threads);
    return TMC3_OK;
  });
}

//============================================================================
// Encoder

tmc3_status
tmc3_encoder_create(
  const char* const* options, int num_options, tmc3_encoder** encoder)
{
  return guard([&] {
    if (!encoder || (num_options && !options) || num_options < 0)
      return fail(TMC3_ERR_ARGUMENT, "invalid argument");
    *encoder = nullptr;

    std::unique_ptr<tmc3_encoder> enc(new tmc3_encoder);
    auto args = toStrings(options, num_options);
    std::string messages;
    if (!enc->log.parse(args, messages))
      return fail(TMC3_ERR_OPTIONS, messages);

    if (!parseCodecOptions(args, false, enc->params, messages))
      return fail(TMC3_ERR_OPTIONS, messages);

    auto& params = enc->params;
    if (params.encoder.gps.biPredictionEnabledFlag)
      return fail(TMC3_ERR_UNSUPPORTED, "bi-prediction is not supported");

    raiseNumThreads(params.numThreads);

    enc->angularOrigin = params.encoder.gps.gpsAngularOrigin;
    axisComponents(params.encoder.sps.geometry_axis_order, enc->axisComponent);
    enc->encoder.setMotionVectorFileName(params.motionVectorPath);
    enc->encoder.setLog(enc->log.stream);

    *encoder = enc.release();
    tlLastError = messages;
    return TMC3_OK;
  });
}

//----------------------------------------------------------------------------

tmc3_status
tmc3_encoder_encode(
  tmc3_encoder* enc,
  const tmc3_cloud* in,
  tmc3_packet** packet,
  tmc3_frame** reconstruction)
{
  return guard([&] {
    if (!enc || !in || !packet)
      return fail(TMC3_ERR_ARGUMENT, "invalid argument");
    *packet = nullptr;
    if (reconstruction)
      *reconstruction = nullptr;

    auto& params = enc->params;
    CodingLogRedirect redirect(*enc->log.stream);

    PCCPointSet3 cloud;
    tmc3_status status = enc->readCloud(*in, cloud);
    if (status != TMC3_OK)
      return status;

    // as tmc3: input sorting, then attribute conversions
    if (params.sortInputByAzimuth)
      sortByAzimuth(
        cloud, 0, cloud.getPointCount(), 0., enc->angularOrigin,
        params.encoder.gps.geom_angular_azimuth_scale_log2_minus11 + 12,
        params.encoder.gps.geom_angular_azimuth_speed_minus1 + 1,
        params.encoder.gps.angularTheta, params.encoder.gps.angularZ);

    if (params.convertColourspace)
      convertFromGbr(params.encoder.sps.attributeSets, cloud);

    scaleAttributesForInput(params.encoder.sps.attributeSets, cloud);

    enc->encoder.setInterForCurrPic(
      params.encoder.gps.interPredictionEnabledFlag
      && (enc->frameIndex % params.encoder.randomAccessPeriod));

    CloudFrame recon;
    auto* reconPtr =
      reconstruction || params.encoder.sps.inter_frame_prediction_enabled_flag
      ? &recon
      : nullptr;

    std::unique_ptr<tmc3_packet> out(new tmc3_packet);
    enc->packet = out.get();
    int ret = enc->encoder.compress(cloud, &params.encoder, enc, reconPtr);
    enc->packet = nullptr;
    enc->frameIndex++;
    if (ret)
      return fail(TMC3_ERR_CODEC, "can't compress point cloud");

    if (reconstruction)
      *reconstruction = new tmc3_frame(params, recon);
    *packet = out.release();
    return TMC3_OK;
  });
}

//----------------------------------------------------------------------------

void
tmc3_encoder_destroy(tmc3_encoder* encoder)
{
  delete encoder;
}

//----------------------------------------------------------------------------

const void*
tmc3_packet_data(const tmc3_packet* packet)
{
  return packet ? packet->data.data() : nullptr;
}

size_t
tmc3_packet_size(const tmc3_packet* packet)
{
  return packet ? packet->data.size() : 0;
}

void
tmc3_packet_release(tmc3_packet* packet)
{
  delete packet;
}

//============================================================================
// Decoder

tmc3_status
tmc3_decoder_create(
  const char* const* options, int num_options, tmc3_decoder** decoder)
{
  return guard([&] {
    if (!decoder || (num_options && !options) || num_options < 0)
      return fail(TMC3_ERR_ARGUMENT, "invalid argument");
    *decoder = nullptr;

    std::unique_ptr<tmc3_decoder> dec(new tmc3_decoder);
    auto args = toStrings(options, num_options);
    std::string messages;
    if (!dec->log.parse(args, messages))
      return fail(TMC3_ERR_OPTIONS, messages);

    if (!parseCodecOptions(args, true, dec->params, messages))
      return fail(TMC3_ERR_OPTIONS, messages);

    raiseNumThreads(dec->params.numThreads);

    dec->decoder.reset(new PCCTMC3Decoder3(dec->params.decoder));
    dec->decoder->setMotionVectorFileName(dec->params.motionVectorPath);
    dec->decoder->setLog(dec->log.stream);

    *decoder = dec.release();
    tlLastError = messages;
    return TMC3_OK;
  });
}

//----------------------------------------------------------------------------

tmc3_status
tmc3_decoder_push(tmc3_decoder* dec, const void* data, size_t size)
{
  return guard([&] {
    if (!dec || (size && !data))
      return fail(TMC3_ERR_ARGUMENT, "invalid argument");
    if (dec->flushed)
      return fail(TMC3_ERR_STATE, "the stream has ended");

    CodingLogRedirect redirect(*dec->log.stream);
    const char* p = static_cast<const char*>(data);
    const char* end = p + size;

    // complete a partial data unit first
    if (!dec->partial.empty()) {
      auto& partial = dec->partial;
      size_t length = 0;
      if (partial.size() < 5) {
        size_t take = std::min(size_t(end - p), 5 - partial.size());
        partial.insert(partial.end(), p, p + take);
        p += take;
        if (partial.size() < 5)
          return TMC3_OK;
      }
      for (int i = 1; i <= 4; i++)
        length = (length << 8) | uint8_t(partial[i]);
      size_t take = std::min(size_t(end - p), 5 + length - partial.size());
      partial.insert(partial.end(), p, p + take);
      p += take;
      if (partial.size() < 5 + length)
        return TMC3_OK;

      std::vector<char> unit;
      unit.swap(partial);
      dec->decode(unit.data(), unit.data() + unit.size());
    }

    // decode from the caller's data, keeping any incomplete data unit
    p = dec->decode(p, end);
    dec->partial.assign(p, end);
    return TMC3_OK;
  });
}

//----------------------------------------------------------------------------

tmc3_status
tmc3_decoder_end_frame(tmc3_decoder* dec)
{
  return guard([&] {
    if (!dec)
      return fail(TMC3_ERR_ARGUMENT, "invalid argument");
    if (dec->flushed)
      return fail(TMC3_ERR_STATE, "the stream has ended");
    if (!dec->partial.empty())
      return fail(TMC3_ERR_STATE, "incomplete data unit");

    CodingLogRedirect redirect(*dec->log.stream);
    if (dec->decoder->endFrame(dec))
      return fail(TMC3_ERR_CODEC, "can't decompress point cloud");
    return TMC3_OK;
  });
}

//----------------------------------------------------------------------------

tmc3_status
tmc3_decoder_flush(tmc3_decoder* dec)
{
  return guard([&] {
    if (!dec)
      return fail(TMC3_ERR_ARGUMENT, "invalid argument");
    if (dec->flushed)
      return TMC3_OK;

    dec->flushed = true;
    CodingLogRedirect redirect(*dec->log.stream);
    if (dec->decoder->decompress(nullptr, dec))
      return fail(TMC3_ERR_CODEC, "can't decompress point cloud");
    if (!dec->partial.empty())
      return fail(
        TMC3_ERR_CODEC, "the stream ends with an incomplete data unit");
    return TMC3_OK;
  });
}

//----------------------------------------------------------------------------

tmc3_status
tmc3_decoder_next_frame(tmc3_decoder* dec, tmc3_frame** frame)
{
  return guard([&] {
    if (!dec || !frame)
      return fail(TMC3_ERR_ARGUMENT, "invalid argument");
    *frame = nullptr;
    if (dec->frames.empty())
      return TMC3_NO_FRAME;

    *frame = dec->frames.front().release();
    dec->frames.pop_front();
    return TMC3_OK;
  });
}

//----------------------------------------------------------------------------

void
tmc3_decoder_destroy(tmc3_decoder* decoder)
{
  delete decoder;
}

//============================================================================
// Frames

tmc3_status
tmc3_frame_get_info(const tmc3_frame* frame, tmc3_frame_info* info)
{
  if (!frame || !info)
    return fail(TMC3_ERR_ARGUMENT, "invalid argument");

  // NB: the arrays allow writing; the frame is not modified by the library
  auto& cloud = const_cast<PCCPointSet3&>(frame->frame.cloud);
  const int64_t n = cloud.getPointCount();

  std::memset(info, 0, sizeof(*info));
  info->num_points = n;
  info->frame_num = frame->frame.frameNum;
  info->scale = frame->scale;

  int axis[3];
  axisComponents(frame->frame.geometry_axis_order, axis);

  if (n) {
    info->position.data = &cloud[0];
    info->position.dtype = TMC3_INT32;
    info->position.stride = sizeof(Vec3<int32_t>);
    for (int k = 0; k < 3; k++)
      info->position.offset[k] = axis[k] * sizeof(int32_t);
  }
  for (int k = 0; k < 3; k++)
    info->origin[k] = frame->origin[axis[k]];

  if (n && cloud.hasColors()) {
    info->colour.data = &cloud.getColor(0);
    info->colour.dtype = TMC3_UINT16;
    info->colour.stride = sizeof(Vec3<attr_t>);
    for (int k = 0; k < 3; k++)
      info->colour.offset[k] = kColourComponent[k] * sizeof(attr_t);
  }

  if (n && cloud.hasReflectances()) {
    info->reflectance.data = &cloud.getReflectance(0);
    info->reflectance.dtype = TMC3_UINT16;
    info->reflectance.stride = sizeof(attr_t);
  }

  return TMC3_OK;
}

//----------------------------------------------------------------------------

tmc3_status
tmc3_frame_get_positions(const tmc3_frame* frame, const tmc3_array* dst)
{
  if (!frame || !dst)
    return fail(TMC3_ERR_ARGUMENT, "invalid argument");

  Layout out;
  if (
    !resolve(*dst, 3, &out)
    || (dst->dtype != TMC3_FLOAT32 && dst->dtype != TMC3_FLOAT64))
    return fail(TMC3_ERR_ARGUMENT, "invalid position array");

  const auto& cloud = frame->frame.cloud;
  const int64_t n = cloud.getPointCount();
  int axis[3];
  axisComponents(frame->frame.geometry_axis_order, axis);

  // as ply::write
  char* base = const_cast<char*>(out.data);
  for (int64_t i = 0; i < n; i++) {
    Vec3<double> pos = cloud[i] * frame->scale + frame->origin;
    char* p = base + i * out.stride;
    for (int k = 0; k < 3; k++) {
      if (dst->dtype == TMC3_FLOAT32)
        store(p + out.offset[k], float(pos[axis[k]]));
      else
        store(p + out.offset[k], pos[axis[k]]);
    }
  }
  return TMC3_OK;
}

//----------------------------------------------------------------------------

void
tmc3_frame_release(tmc3_frame* frame)
{
  delete frame;
}

}  // extern "C"
