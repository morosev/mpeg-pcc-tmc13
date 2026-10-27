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

#pragma once

#include <array>
#include <string>
#include <vector>

#include "PCCTMC3Decoder.h"
#include "PCCTMC3Encoder.h"
#include "frame.h"

//============================================================================

enum class OutputSystem
{
  // Output after global scaling, don't convert to external system
  kConformance = 0,

  // Scale output to external coordinate system
  kExternal = 1,
};

//----------------------------------------------------------------------------

struct Parameters {
  bool isDecoder;

  // command line parsing should adjust dist2 values according to PQS
  bool positionQuantizationScaleAdjustsDist2;

  // Scale factor to apply when loading the ply before integer conversion.
  // Eg, If source point positions are in fractional metres converting to
  // millimetres will allow some fidelity to be preserved.
  double inputScale;

  // Length of the output point clouds unit vectors.
  double outputUnitLength;

  // output mode for ply writing (binary or ascii)
  bool outputBinaryPly;

  // Number of threads used for parallel processing
  int numThreads;

  // Fractional fixed-point bits retained in conformance output
  int outputFpBits;

  // Output coordinate system to use
  OutputSystem outputSystem;

  // when true, configure the encoder as if no attributes are specified
  bool disableAttributeCoding;

  // Frame number of first file in input sequence.
  int firstFrameNum;

  // Number of frames to process.
  int frameCount;

  std::string uncompressedDataPath;
  std::string compressedStreamPath;
  std::string reconstructedDataPath;

  // Filename for saving recoloured point cloud (encoder).
  std::string postRecolorPath;

  // Filename for saving pre inverse scaled point cloud (decoder).
  std::string preInvScalePath;

  pcc::EncoderParams encoder;
  pcc::DecoderParams decoder;

  // perform attribute colourspace conversion on ply input/output.
  bool convertColourspace;

  // resort the input points by azimuth angle
  bool sortInputByAzimuth;

  std::string motionVectorPath;
};

//============================================================================
// Option parsing shared by the tmc3 application and the library interface

// Parse command line arguments for the tmc3 application.
// Prints help or the effective configuration to std::cout.
bool ParseParameters(int argc, char* argv[], Parameters& params);

// Parse codec options given as "name=value" (or "--name=value") strings,
// as accepted on the command line, including "config=<file>".  File paths
// and the mode cannot be set.  Errors and warnings are appended to
// messages; returns false on error.
bool parseCodecOptions(
  const std::vector<std::string>& options,
  bool isDecoder,
  Parameters& params,
  std::string& messages);

//============================================================================
// Conversions applied to clouds read from and written to ply files

std::array<const char*, 3> axisOrderToPropertyNames(pcc::AxisOrder order);

void convertToGbr(
  const std::vector<pcc::AttributeDescription>& attrDescs,
  pcc::PCCPointSet3& cloud);

void convertFromGbr(
  const std::vector<pcc::AttributeDescription>& attrDescs,
  pcc::PCCPointSet3& cloud);

void scaleAttributesForInput(
  const std::vector<pcc::AttributeDescription>& attrDescs,
  pcc::PCCPointSet3& cloud);

void scaleAttributesForOutput(
  const std::vector<pcc::AttributeDescription>& attrDescs,
  pcc::PCCPointSet3& cloud);

// The output position scale factor and origin, according to the
// configured output coordinate system
double outputScale(const Parameters& params, const pcc::CloudFrame& frame);

pcc::Vec3<double>
outputOrigin(const Parameters& params, const pcc::CloudFrame& frame);
