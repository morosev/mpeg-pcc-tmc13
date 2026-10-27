/* The copyright in this software is being made available under the BSD
 * Licence, included below.  This software may be subject to other third
 * party and contributor rights, including patent rights, and no such
 * rights are granted under this licence.
 *
 * Copyright (c) 2017-2018, ISO/IEC
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

#include "TMC3.h"

#include <future>
#include <memory>

#include "PCCTMC3Encoder.h"
#include "PCCTMC3Decoder.h"
#include "app_params.h"
#include "ply.h"
#include "pointset_processing.h"
#include "program_options_lite.h"
#include "io_tlv.h"
#include "parallel.h"
#include "version.h"

using namespace std;
using namespace pcc;

//============================================================================

class SequenceCodec {
public:
  // NB: params must outlive the lifetime of the decoder.
  SequenceCodec(Parameters* params) : params(params) {}

  // Perform conversions and write output point cloud
  //  \params cloud  a mutable copy of reconFrame.cloud
  void writeOutputFrame(
    const std::string& postInvScalePath,
    const std::string& preInvScalePath,
    const CloudFrame& reconFrame,
    PCCPointSet3& cloud);

protected:
  Parameters* params;
};

//----------------------------------------------------------------------------

class SequenceEncoder
  : public SequenceCodec
  , PCCTMC3Encoder3::Callbacks {
public:
  // NB: params must outlive the lifetime of the decoder.
  SequenceEncoder(Parameters* params);

  int compress(Stopwatch* clock);

protected:
  int compressOneFrame(Stopwatch* clock);
  int compressOneGOF(Stopwatch* clock);

  // Read and prepare the input frame frameFileNum
  bool loadInputFrame(int frameFileNum, PCCPointSet3& pointCloud);

  void onOutputBuffer(const PayloadBuffer& buf) override;
  void onPostRecolour(const PCCPointSet3& cloud) override;

private:
  ply::PropertyNameMap _plyAttrNames;

  // The raw origin used for input sorting
  Vec3<int> _angularOrigin;

  PCCTMC3Encoder3 encoder;

  std::ofstream bytestreamFile;

  int frameNum;

  int gofSizePlusOne;
  // Indicates the file number of current frame
  int currentFrame;
  // Indicates the file number of previous I/P frame
  int preIPFrame;
  bool codedGOF;

  // The next input frame, read while the current frame is coded
  struct InputPrefetch {
    int fileNum = -1;
    bool ok = false;
    PCCPointSet3 cloud;
    std::future<void> done;
  } _prefetch;
};

//----------------------------------------------------------------------------

class SequenceDecoder
  : public SequenceCodec
  , PCCTMC3Decoder3::Callbacks {
public:
  // NB: params must outlive the lifetime of the decoder.
  SequenceDecoder(Parameters* params);

  int decompress(Stopwatch* clock);

protected:
  void onOutputCloud(CloudFrame& cloud) override;

private:
  PCCTMC3Decoder3 decoder;

  std::ofstream bytestreamFile;

  Stopwatch* clock;

  // The output frame being written while the next frame is decoded
  std::future<void> _pendingWrite;
};

//============================================================================

int
main(int argc, char* argv[])
{
  cout << "MPEG PCC tmc3 version " << ::pcc::version << endl;

  Parameters params;

  try {
    if (!ParseParameters(argc, argv, params))
      return 1;
  }
  catch (df::program_options_lite::ParseFailure& e) {
    std::cerr << "Error parsing option \"" << e.arg << "\" with argument \""
              << e.val << "\"." << std::endl;
    return 1;
  }

  pcc::setNumThreads(params.numThreads);

  // Timers to count elapsed wall/user time
  pcc::chrono::Stopwatch<std::chrono::steady_clock> clock_wall;
  pcc::chrono::Stopwatch<pcc::chrono::utime_inc_children_clock> clock_user;

  clock_wall.start();

  int ret = 0;
  if (params.isDecoder) {
    ret = SequenceDecoder(&params).decompress(&clock_user);
  } else {
    ret = SequenceEncoder(&params).compress(&clock_user);
  }

  clock_wall.stop();

  using namespace std::chrono;
  auto total_wall = duration_cast<milliseconds>(clock_wall.count()).count();
  auto total_user = duration_cast<milliseconds>(clock_user.count()).count();
  std::cout << "Processing time (wall): " << total_wall / 1000.0 << " s\n";
  std::cout << "Processing time (user): " << total_user / 1000.0 << " s\n";

  return ret;
}

//============================================================================

SequenceEncoder::SequenceEncoder(Parameters* params) : SequenceCodec(params)
{
  // determine the naming (ordering) of ply properties
  _plyAttrNames.position =
    axisOrderToPropertyNames(params->encoder.sps.geometry_axis_order);

  // NB: this is the raw origin before the encoder tweaks it
  _angularOrigin = params->encoder.gps.gpsAngularOrigin;
}

//----------------------------------------------------------------------------

int
SequenceEncoder::compress(Stopwatch* clock)
{
  bytestreamFile.open(params->compressedStreamPath, ios::binary);
  if (!bytestreamFile.is_open()) {
    return -1;
  }
  this->encoder.setMotionVectorFileName(params->motionVectorPath);
  const int lastFrameNum = params->firstFrameNum + params->frameCount;
  if (!params->encoder.gps.biPredictionEnabledFlag) {
    for (frameNum = params->firstFrameNum; frameNum < lastFrameNum;
         frameNum++) {
      this->encoder.setInterForCurrPic(
        params->encoder.gps.interPredictionEnabledFlag
        && ((frameNum - params->firstFrameNum) % params->encoder.randomAccessPeriod));
      if (compressOneFrame(clock))
        return -1;
    }
  } else {
    preIPFrame = -1;
    currentFrame = -1;
    codedGOF = false;
    for (frameNum = params->firstFrameNum; frameNum < lastFrameNum;
         frameNum += params->encoder.randomAccessPeriod) {
      gofSizePlusOne =
        ((frameNum + params->encoder.randomAccessPeriod) >= lastFrameNum)
        ? (lastFrameNum - frameNum)
        : (params->encoder.randomAccessPeriod + 1);
      if (compressOneGOF(clock)) {
        return -1;
      }
    }
  }

  std::cout << "Total bitstream size " << bytestreamFile.tellp() << " B\n";
  bytestreamFile.close();

  return 0;
}

//----------------------------------------------------------------------------

bool
SequenceEncoder::loadInputFrame(int frameFileNum, PCCPointSet3& pointCloud)
{
  std::string srcName{expandNum(params->uncompressedDataPath, frameFileNum)};
  if (
    !ply::read(srcName, _plyAttrNames, params->inputScale, pointCloud)
    || pointCloud.getPointCount() == 0)
    return false;

  // Some evaluations wish to scan the points in azimuth order to simulate
  // real-time acquisition (since the input has lost its original order).
  // NB: because this is trying to emulate the input order, binning is disabled
  if (params->sortInputByAzimuth)
    sortByAzimuth(
      pointCloud, 0, pointCloud.getPointCount(), 0., _angularOrigin,
      params->encoder.gps.geom_angular_azimuth_scale_log2_minus11 + 12,
      params->encoder.gps.geom_angular_azimuth_speed_minus1 + 1,
      params->encoder.gps.angularTheta,
      params->encoder.gps.angularZ);

  // Sanitise the input point cloud
  // todo(df): remove the following with generic handling of properties
  bool codeColour = params->encoder.attributeIdxMap.count("color");
  if (!codeColour)
    pointCloud.removeColors();
  assert(codeColour == pointCloud.hasColors());

  bool codeReflectance = params->encoder.attributeIdxMap.count("reflectance");
  if (!codeReflectance)
    pointCloud.removeReflectances();
  assert(codeReflectance == pointCloud.hasReflectances());

  return true;
}

//----------------------------------------------------------------------------

int
SequenceEncoder::compressOneFrame(Stopwatch* clock)
{
  const auto frameFileNum = params->encoder.gps.biPredictionEnabledFlag
    ? (encoder.getCurrFrameIndex() + params->firstFrameNum)
    : frameNum;
  PCCPointSet3 pointCloud;
  bool loaded = false;
  if (_prefetch.done.valid()) {
    _prefetch.done.get();
    if (_prefetch.fileNum == frameFileNum && _prefetch.ok) {
      pointCloud.swap(_prefetch.cloud);
      loaded = true;
    }
    _prefetch.cloud = PCCPointSet3();
  }

  if (!loaded && !loadInputFrame(frameFileNum, pointCloud)) {
    cout << "Error: can't open input file!" << endl;
    return -1;
  }

  // With several threads, read the next frame while this one is coded
  // NB: frames are coded in file order unless bi-prediction is used
  const int nextFileNum = frameFileNum + 1;
  if (
    pcc::numThreads() > 1 && !params->encoder.gps.biPredictionEnabledFlag
    && nextFileNum < params->firstFrameNum + params->frameCount) {
    _prefetch.fileNum = nextFileNum;
    _prefetch.done = std::async(std::launch::async, [this, nextFileNum] {
      _prefetch.ok = loadInputFrame(nextFileNum, _prefetch.cloud);
    });
  }

  clock->start();

  if (params->convertColourspace)
    convertFromGbr(params->encoder.sps.attributeSets, pointCloud);

  scaleAttributesForInput(params->encoder.sps.attributeSets, pointCloud);

  // The reconstructed point cloud
  CloudFrame recon;
  auto* reconPtr =
    params->reconstructedDataPath.empty()
    && !params->encoder.sps.inter_frame_prediction_enabled_flag
    ? nullptr : &recon;

  auto bytestreamLenFrameStart = bytestreamFile.tellp();

    int ret = (params->encoder.gps.biPredictionEnabledFlag == 2)
    ? encoder.compressHGOF(pointCloud, &params->encoder, this, reconPtr)
    : encoder.compress(pointCloud, &params->encoder, this, reconPtr);
  if (ret) {
    cout << "Error: can't compress point cloud!" << endl;
    return -1;
  }

  auto bytestreamLenFrameEnd = bytestreamFile.tellp();
  int frameLen = bytestreamLenFrameEnd - bytestreamLenFrameStart;
  std::cout << "Total frame size " << frameLen << " B" << std::endl;

  clock->stop();

  if (!params->reconstructedDataPath.empty())
    writeOutputFrame(params->reconstructedDataPath, {}, recon, recon.cloud);

  return 0;
}

int
SequenceEncoder::compressOneGOF(Stopwatch* clock)
{
  //Generate the indexes for I-frame and P-frame in one GOF
  std::vector<int> IPFrameIndexes;
  for (int i = 0; i < gofSizePlusOne;
       i += params->encoder.gps.biPredictionPeriod)
    IPFrameIndexes.push_back(i);
  if (IPFrameIndexes.back() != gofSizePlusOne - 1)
    IPFrameIndexes.push_back(gofSizePlusOne - 1);

  auto const firstFrameNum = params->firstFrameNum;
  auto const randomAccessPeriod = params->encoder.randomAccessPeriod;
  //First process the I-frame and P-frame
  //If the Bi-prediction is disabled, the other frames are set to P-frame
  //Otherwise, the other frames are set to B-frame
  for (int i = codedGOF ? 1 : 0; i < IPFrameIndexes.size(); i++) {
    int currentIPFrame = IPFrameIndexes[i] + frameNum;

    // First frame is always coded as I-frame
    if (preIPFrame == -1) {
      encoder.setInterForCurrPic(false);
      encoder.setBiPredEncodeParams(false, 0, -1, -1, 0);
      currentFrame = frameNum;
      if (compressOneFrame(clock))
        return -1;
    } else {
      const auto prevIPFrameDelta = preIPFrame - firstFrameNum;
      if (encoder.biPredictionEligibility(
            currentIPFrame - firstFrameNum, prevIPFrameDelta,
            &params->encoder)) {
        // Code the next I-frame/P-frame first
        currentFrame = currentIPFrame;
        encoder.setInterForCurrPic(
          (currentFrame - firstFrameNum) % randomAccessPeriod);
        encoder.setBiPredEncodeParams(
          false, currentFrame - firstFrameNum, prevIPFrameDelta, -1, 1);
        if (compressOneFrame(clock))
          return -1;

        // Code the other frames as B-frames
        // If the hierarchical GOF sturcture is applied
        if (params->encoder.gps.biPredictionEnabledFlag == 2) {
          if (
            currentIPFrame - preIPFrame - 1 != encoder.getCodeOrderListSize())
            encoder.initBiPredEncodeParamsGOF(currentIPFrame - preIPFrame);

          encoder.setRefTimesList();
          for (int i = 0; i < encoder.getCodeOrderListSize(); i++) {
            currentFrame = preIPFrame + encoder.getCodeOrder(i);
            encoder.setInterForCurrPic(true);
            encoder.setBiPredEncodeParams(
              true, currentFrame - firstFrameNum,
              encoder.getRefFrame(i * 2) + prevIPFrameDelta,
              encoder.getRefFrame(i * 2 + 1) + prevIPFrameDelta,
              encoder.getQPshift(i));
            encoder.setCurrFrameIndexInGOF(encoder.getCodeOrder(i));
            if (compressOneFrame(clock))
              return -1;
          }
        } else {
          // IF the IBBP GOF structure is applied
          for (currentFrame = preIPFrame + 1; currentFrame < currentIPFrame;
               currentFrame++) {
            auto const frameDelta = currentFrame - firstFrameNum;
            encoder.setInterForCurrPic(true);
            encoder.setBiPredEncodeParams(
              true, frameDelta, frameDelta - 1, currentIPFrame - firstFrameNum,
              2);
            if (compressOneFrame(clock))
              return -1;
          }
        }
      } else {
        for (currentFrame = preIPFrame + 1; currentFrame < currentIPFrame + 1;
             currentFrame++) {
          auto const frameDelta = currentFrame - firstFrameNum;
          encoder.setInterForCurrPic(frameDelta % randomAccessPeriod);
          encoder.setBiPredEncodeParams(
            false, frameDelta, frameDelta - 1, -1, 1);

          if (compressOneFrame(clock))
            return -1;
        }
      }
    }
    preIPFrame = currentIPFrame;
  }
  codedGOF = true;
  return 0;
}

//----------------------------------------------------------------------------

void
SequenceEncoder::onOutputBuffer(const PayloadBuffer& buf)
{
  writeTlv(buf, bytestreamFile);
}

//----------------------------------------------------------------------------

void
SequenceEncoder::onPostRecolour(const PCCPointSet3& cloud)
{
  if (params->postRecolorPath.empty()) {
    return;
  }

  // todo(df): don't allocate if conversion is not required
  PCCPointSet3 tmpCloud(cloud);
  CloudFrame frame;
  frame.setParametersFrom(params->encoder.sps, params->encoder.outputFpBits);
  frame.cloud = cloud;
  frame.frameNum = frameNum - params->firstFrameNum;

  writeOutputFrame(params->postRecolorPath, {}, frame, tmpCloud);
}

//============================================================================

SequenceDecoder::SequenceDecoder(Parameters* params)
  : SequenceCodec(params), decoder(params->decoder)
{}

//----------------------------------------------------------------------------

int
SequenceDecoder::decompress(Stopwatch* clock)
{
  ifstream fin(params->compressedStreamPath, ios::binary);
  if (!fin.is_open()) {
    return -1;
  }
  decoder.setMotionVectorFileName(params->motionVectorPath);
  this->clock = clock;
  clock->start();

  PayloadBuffer buf;
  while (true) {
    PayloadBuffer* buf_ptr = &buf;
    readTlv(fin, &buf);

    // at end of file (or other error), flush decoder
    if (!fin)
      buf_ptr = nullptr;

    if (decoder.decompress(buf_ptr, this)) {
      cout << "Error: can't decompress point cloud!" << endl;
      return -1;  
    }

    if (!buf_ptr)
      break;
  }

  if (_pendingWrite.valid())
    _pendingWrite.get();

  fin.clear();
  fin.seekg(0, ios_base::end);
  std::cout << "Total bitstream size " << fin.tellg() << " B" << std::endl;

  clock->stop();

  return 0;
}

//----------------------------------------------------------------------------

void
SequenceDecoder::onOutputCloud(CloudFrame& frame)
{
  // With several threads, write the frame while the next one is decoded
  // NB: one frame is written at a time, in order
  if (pcc::numThreads() > 1) {
    if (_pendingWrite.valid())
      _pendingWrite.get();
    auto copy = std::make_shared<CloudFrame>(frame);
    _pendingWrite = std::async(std::launch::async, [this, copy] {
      writeOutputFrame(
        params->reconstructedDataPath, params->preInvScalePath, *copy,
        copy->cloud);
    });
    return;
  }

  clock->stop();

  // copy the point cloud in order to modify it according to the output options
  PCCPointSet3 pointCloud(frame.cloud);
  writeOutputFrame(
    params->reconstructedDataPath, params->preInvScalePath, frame, pointCloud);

  clock->start();
}

//============================================================================

void
SequenceCodec::writeOutputFrame(
  const std::string& postInvScalePath,
  const std::string& preInvScalePath,
  const CloudFrame& frame,
  PCCPointSet3& cloud)
{
  if (postInvScalePath.empty() && preInvScalePath.empty())
    return;

  scaleAttributesForOutput(frame.attrDesc, cloud);

  if (params->convertColourspace)
    convertToGbr(frame.attrDesc, cloud);

  // the order of the property names must be determined from the sps
  ply::PropertyNameMap attrNames;
  attrNames.position = axisOrderToPropertyNames(frame.geometry_axis_order);

  // offset frame number
  int frameNum = frame.frameNum + params->firstFrameNum;

  // Dump the decoded colour using the pre inverse scaled geometry
  if (!preInvScalePath.empty()) {
    std::string filename{expandNum(preInvScalePath, frameNum)};
    ply::write(cloud, attrNames, 1.0, 0.0, filename, !params->outputBinaryPly);
  }

  auto plyScale = outputScale(*params, frame) / (1 << frame.outputFpBits);
  auto plyOrigin = outputOrigin(*params, frame);
  std::string decName{expandNum(postInvScalePath, frameNum)};
  if (!ply::write(
        cloud, attrNames, plyScale, plyOrigin, decName,
        !params->outputBinaryPly)) {
    cout << "Error: can't open output file!" << endl;
  }
}
