/* The copyright in this software is being made available under the BSD
 * Licence, included below.  This software may be subject to other third
 * party and contributor rights, including patent rights, and no such
 * rights are granted under this licence.
 *
 * Copyright (c) 2020, ISO/IEC
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

#include <cmath>
#include <cstdint>
#include <vector>

#include "constants.h"
#include "entropy.h"
#include "PCCMath.h"
#include "simd.h"
#include "hls.h"
#include "PCCPointSet.h"
#include "geometry_params.h"
#include <map>


namespace pcc {

//============================================================================

struct GPredicter {
  enum Mode
  {
    None,
    Delta,
    Linear2,
    Linear3
  };

  int32_t index[3];
  int pgeom_min_radius;

  bool isValid(Mode mode);

  Vec3<int32_t> predict(const Vec3<int32_t>* points, Mode mode, bool angular);
};

//============================================================================

struct GNode {
  static const int32_t MaxChildrenCount = 3;

  int numDups;
  int32_t parent;
  int32_t childrenCount;
  int32_t children[MaxChildrenCount];
};

//============================================================================

class PredGeomContexts {
public:
  void reset();
  static const uint8_t interFlagBufferMask = 0x1F;

protected:
  AdaptiveBitModel _ctxNumChildren[3];
  AdaptiveBitModel _ctxPredMode[3];
  AdaptiveBitModel _ctxPredIdx[kPTEMaxPredictorIndex];
  AdaptiveBitModel _ctxResGt0[2][3];
  AdaptiveBitModel _ctxSign[2][3];
  AdaptiveBitModel _ctxNumBits[2][5][3][31];
  AdaptiveBitModel _ctxNumDupPointsGt0;
  AdaptiveBitModel _ctxNumDupPoints;
  AdaptiveBitModel _ctxInterFlag[32];
  AdaptiveBitModel _ctxRefNodeIdx[3];
  AdaptiveBitModel _ctxRefDirFlag;

  AdaptiveBitModel _ctxResidual2GtN[2][3];
  AdaptiveBitModel _ctxSign2[3];
  AdaptiveBitModel _ctxEG2Prefix[3][5];
  AdaptiveBitModel _ctxEG2Suffix[3][4];

  AdaptiveBitModel _ctxQpOffsetAbsGt0;
  AdaptiveBitModel _ctxQpOffsetSign;
  AdaptiveBitModel _ctxQpOffsetAbsEgl;

  AdaptiveBitModel _ctxPhiGtN[2][2][2];
  AdaptiveBitModel _ctxSignPhi[2][2];
  AdaptiveBitModel _ctxEGPhi[2][2];
  AdaptiveBitModel _ctxResidualPhi[2][2][7];

  AdaptiveBitModel _ctxEndOfTrees;
  AdaptiveBitModel _ctxResRGTZero[2][4];
  AdaptiveBitModel _ctxResRGTOne[2][4];
  AdaptiveBitModel _ctxResRGTTwo[2][4];
  AdaptiveBitModel _ctxResRExpGolombPre[2][4][10];
  AdaptiveBitModel _ctxResRExpGolombSuf[2][4][10];

  AdaptiveBitModel _ctxResPhiGTZero[2][2];
  AdaptiveBitModel _ctxResPhiSign[2][2+2+1]; 
  AdaptiveBitModel _ctxResPhiGTOne[2][2];
  AdaptiveBitModel _ctxResPhiExpGolombPre[3][4];
  AdaptiveBitModel _ctxResPhiExpGolombSuf[3][4];

  AdaptiveBitModel _ctxResRSign[3][2][8];

  bool _prevInterFlag = false;
  bool _precSignR = false;
  int _resPhiOldSign = 3;
  //int _resPhiOldSign = 2;
  int _precAzimuthStepDelta = 0;
};

//----------------------------------------------------------------------------

inline void
PredGeomContexts::reset()
{
  this->~PredGeomContexts();
  new (this) PredGeomContexts;
}

//============================================================================

template<typename LookupFn>
GPredicter
makePredicter(
  int32_t curNodeIdx,
  GPredicter::Mode mode,
  int minRadius,
  LookupFn nodeIdxToParentIdx)
{
  if (mode == GPredicter::None)
    mode = GPredicter::Delta;

  GPredicter predIdx;
  predIdx.pgeom_min_radius = minRadius;

  switch (mode) {
  default:
  case GPredicter::None:
  case GPredicter::Delta:
  case GPredicter::Linear2:
  case GPredicter::Linear3:
    for (int i = 0; i < int(mode); i++) {
      if (curNodeIdx < 0)
        break;
      predIdx.index[i] = curNodeIdx = nodeIdxToParentIdx(curNodeIdx);
    }
    break;
  }
  return predIdx;
}

//============================================================================

inline bool
GPredicter::isValid(GPredicter::Mode mode)
{
  int numPredictors = int(mode);
  for (int i = 0; i < numPredictors; i++) {
    if (this->index[i] < 0)
      return false;
  }
  return true;
}

//============================================================================

inline Vec3<int32_t>
GPredicter::predict(
  const Vec3<int32_t>* points, GPredicter::Mode mode, bool angular)
{
  Vec3<int32_t> pred;
  switch (mode) {
  case GPredicter::None: {
    pred = 0;

    if (angular)
      pred[0] = pgeom_min_radius;

    if (this->index[0] >= 0 && angular) {
      const auto& p0 = points[this->index[0]];
      pred[1] = p0[1];
      pred[2] = p0[2];
    }
    break;
  }

  case GPredicter::Delta: {
    pred = 0;

    pred[0] = pgeom_min_radius;

    if (this->index[0] >= 0) {
      pred = points[this->index[0]];
    }
    break;
  }

  case GPredicter::Linear2: {
    const auto& p0 = points[this->index[0]];
    const auto& p1 = points[this->index[1]];
    pred = 2 * p0 - p1;
    break;
  }

  default:
  case GPredicter::Linear3: {
    const auto& p0 = points[this->index[0]];
    const auto& p1 = points[this->index[1]];
    const auto& p2 = points[this->index[2]];
    pred = p0 + p1 - p2;
    break;
  }
  }
  return pred;
}

//============================================================================

class SphericalToCartesian {
public:
  SphericalToCartesian(const GeometryParameterSet& gps)
    : log2ScaleRadius(gps.geom_angular_radius_inv_scale_log2)
    , log2ScalePhi(gps.geom_angular_azimuth_scale_log2_minus11 + 12)
    , tanThetaLaser(gps.angularTheta.data())
    , zLaser(gps.angularZ.data())
  {}

  Vec3<int32_t> operator()(Vec3<int32_t> sph)
  {
    int64_t r = radius(sph[0]);

    return Vec3<int32_t>(Vec3<int64_t>{
      planar(r, icos(sph[1], log2ScalePhi)),
      planar(r, isin(sph[1], log2ScalePhi)), height(r, sph[2])});
  }

  // The components of the conversion, for callers that evaluate several
  // positions sharing a radius, azimuth or laser.

  int64_t radius(int32_t sphR) const { return sphR << log2ScaleRadius; }

  int azimuthScaleLog2() const { return log2ScalePhi; }

  // x (or y) for the scaled radius r and the cosine (or sine) of the azimuth
  static int64_t planar(int64_t r, int32_t trig)
  {
    return divExp2RoundHalfInf(r * trig, kLog2ISineScale);
  }

  // z for the scaled radius r and laser laserIdx
  int64_t height(int64_t r, int laserIdx) const
  {
    int64_t z = divExp2RoundHalfInf(
      tanThetaLaser[laserIdx] * r << 2, log2ScaleTheta - log2ScaleZ);
    return divExp2RoundHalfInf(z - zLaser[laserIdx], log2ScaleZ);
  }

private:
  static constexpr int log2ScaleZ = 3;
  static constexpr int log2ScaleTheta = 20;
  int log2ScaleRadius;
  int log2ScalePhi;
  const int* tanThetaLaser;
  const int* zLaser;
};

//============================================================================

class CartesianToSpherical {
public:
  CartesianToSpherical(const GeometryParameterSet& gps)
    : sphToCartesian(gps)
    , log2ScaleRadius(gps.geom_angular_radius_inv_scale_log2)
    , scalePhi(1 << (gps.geom_angular_azimuth_scale_log2_minus11 + 12))
    , numLasers(gps.angularTheta.size())
    , tanThetaLaser(gps.angularTheta.data())
    , zLaser(gps.angularZ.data())
  {
#if defined(PCC_SIMD)
    initLaserSearch();
#endif
  }

  Vec3<int32_t> operator()(Vec3<int32_t> xyz)
  {
    int64_t r0 = int64_t(std::round(hypot(xyz[0], xyz[1])));
    int32_t thetaIdx = findLaser(r0, xyz[2]);

    auto phi0 = std::round((atan2(xyz[1], xyz[0]) / (2.0 * M_PI)) * scalePhi);

    Vec3<int32_t> sphPos{int32_t(divExp2RoundHalfUp(r0, log2ScaleRadius)),
                         int32_t(phi0), thetaIdx};

    // local optmization
    auto minErr = (sphToCartesian(sphPos) - xyz).getNorm1();
    int32_t dt0 = 0;
    int32_t dr0 = 0;
    if (minErr) {
      // All candidates share the laser: z depends only upon the radius,
      // and x, y upon the radius and azimuth.  Evaluate each once.
      const int log2ScalePhi = sphToCartesian.azimuthScaleLog2();
      int64_t rCand[5];
      int32_t zErr[5], cosCand[5], sinCand[5];
      for (int32_t d = -2; d <= 2; ++d) {
        rCand[d + 2] = sphToCartesian.radius(sphPos[0] + d);
        zErr[d + 2] = std::abs(
          int32_t(sphToCartesian.height(rCand[d + 2], thetaIdx)) - xyz[2]);
        cosCand[d + 2] = icos(sphPos[1] + d, log2ScalePhi);
        sinCand[d + 2] = isin(sphPos[1] + d, log2ScalePhi);
      }

      for (int32_t dt = -2; dt <= 2 && minErr; ++dt) {
        for (int32_t dr = -2; dr <= 2; ++dr) {
          const int64_t r = rCand[dr + 2];
          int32_t x = int32_t(SphericalToCartesian::planar(r, cosCand[dt + 2]));
          int32_t y = int32_t(SphericalToCartesian::planar(r, sinCand[dt + 2]));
          auto err = std::abs(x - xyz[0]) + std::abs(y - xyz[1]) + zErr[dr + 2];
          if (err < minErr) {
            minErr = err;
            dt0 = dt;
            dr0 = dr;
          }
        }
      }
    }
    sphPos[0] += dr0;
    sphPos[1] += dt0;

    return sphPos;
  }

private:
  // The first laser whose height at radius r0 is closest to z.
  int32_t findLaser(int64_t r0, int32_t z)
  {
#if defined(PCC_SIMD)
    if (r0 <= laserSearchMaxR && std::abs(z) <= kLaserSearchMaxAbs)
      return findLaserSimd(uint32_t(r0), z);
#endif
    int32_t thetaIdx = 0;
    int32_t minError = std::numeric_limits<int32_t>::max();
    for (int idx = 0; idx < numLasers; ++idx) {
      int64_t z0 = divExp2RoundHalfInf(
        tanThetaLaser[idx] * r0 << 2, log2ScaleTheta - log2ScaleZ);
      int64_t z1 = divExp2RoundHalfInf(z0 - zLaser[idx], log2ScaleZ);
      int32_t err = std::abs(z1 - z);
      if (err < minError) {
        thetaIdx = idx;
        minError = err;
      }
    }
    return thetaIdx;
  }

#if defined(PCC_SIMD)
  // Bound on the magnitude of intermediate values in the 32-bit search
  static constexpr int32_t kLaserSearchMaxAbs = 1 << 28;

  void initLaserSearch()
  {
    // NB: the search is disabled by a negative radius limit
    laserSearchMaxR = -1;
    // NB: padded for whole vectors of up to eight lanes
    const int padded = (numLasers + 7) & ~7;
    if (!numLasers || padded > kMaxLasers)
      return;

    absTan.assign(padded, 0);
    tanSign.assign(padded, 0);
    zOffset.assign(padded, 0);
    // padding lanes never attain the minimum
    errFloor.assign(padded, std::numeric_limits<int32_t>::max());
    std::fill_n(errFloor.begin(), numLasers, 0);

    uint32_t maxAbsTan = 0;
    for (int idx = 0; idx < numLasers; ++idx) {
      if (std::abs(int64_t(zLaser[idx])) > kLaserSearchMaxAbs)
        return;
      absTan[idx] = uint32_t(std::abs(int64_t(tanThetaLaser[idx])));
      tanSign[idx] = tanThetaLaser[idx] < 0 ? -1 : 0;
      zOffset[idx] = zLaser[idx];
      maxAbsTan = std::max(maxAbsTan, absTan[idx]);
    }

    // |z| = |tan| * r0 * 4 / 2^17 must not exceed kLaserSearchMaxAbs
    const int64_t maxProduct = int64_t(kLaserSearchMaxAbs) << 15;
    laserSearchMaxR = maxAbsTan
      ? std::min<int64_t>(maxProduct / maxAbsTan, UINT32_MAX)
      : UINT32_MAX;
  }

  int32_t findLaserSimd(uint32_t r0, int32_t z)
  {
    // NB: r0 >= 0, so rounding tan * r0 * 4 / 2^17 half away from zero is
    //     sign(tan) * ((|tan| * r0 + 2^14) >> 15), a rounding narrow shift.
    //     Every subsequent value fits in 32 bits by construction.
    //     Then z1 = divExp2RoundHalfInf(z0 - zLaser, 3) and the error is
    //     |z1 - z|, padding lanes being raised to errFloor.
    int32_t err[kMaxLasers];
    const int padded = int(absTan.size());
    int32_t minError;
#  if defined(PCC_SIMD_NEON)
    const uint32x4_t vr0 = vdupq_n_u32(r0);
    const int32x4_t vz = vdupq_n_s32(z);
    int32x4_t vmin = vdupq_n_s32(std::numeric_limits<int32_t>::max());
    for (int idx = 0; idx < padded; idx += 4) {
      uint32x4_t t = vld1q_u32(&absTan[idx]);
      uint32x2_t lo =
        vrshrn_n_u64(vmull_u32(vget_low_u32(t), vget_low_u32(vr0)), 15);
      uint32x4_t q = vrshrn_high_n_u64(lo, vmull_high_u32(t, vr0), 15);
      int32x4_t s = vld1q_s32(&tanSign[idx]);
      int32x4_t z0 = vsubq_s32(veorq_s32(vreinterpretq_s32_u32(q), s), s);

      int32x4_t d = vsubq_s32(z0, vld1q_s32(&zOffset[idx]));
      int32x4_t ds = vshrq_n_s32(d, 31);
      uint32x4_t a = vrshrq_n_u32(vreinterpretq_u32_s32(vabsq_s32(d)), 3);
      int32x4_t z1 = vsubq_s32(veorq_s32(vreinterpretq_s32_u32(a), ds), ds);

      int32x4_t e = vabdq_s32(z1, vz);
      e = vmaxq_s32(e, vld1q_s32(&errFloor[idx]));
      vst1q_s32(&err[idx], e);
      vmin = vminq_s32(vmin, e);
    }
    minError = vminvq_s32(vmin);
#  elif defined(PCC_SIMD_AVX2)
    const __m256i vr0 = _mm256_set1_epi32(int32_t(r0));
    const __m256i vz = _mm256_set1_epi32(z);
    const __m256i round = _mm256_set1_epi64x(1 << 14);
    const __m256i four = _mm256_set1_epi32(4);
    __m256i vmin = _mm256_set1_epi32(std::numeric_limits<int32_t>::max());
    auto load = [](const int32_t* v) {
      return _mm256_loadu_si256(reinterpret_cast<const __m256i*>(v));
    };
    for (int idx = 0; idx < padded; idx += 8) {
      __m256i t = load(reinterpret_cast<const int32_t*>(&absTan[idx]));
      __m256i q = simdMulShiftLo(t, vr0, round, 15);
      __m256i s = load(&tanSign[idx]);
      __m256i z0 = _mm256_sub_epi32(_mm256_xor_si256(q, s), s);

      __m256i d = _mm256_sub_epi32(z0, load(&zOffset[idx]));
      __m256i ds = _mm256_srai_epi32(d, 31);
      __m256i a =
        _mm256_srli_epi32(_mm256_add_epi32(_mm256_abs_epi32(d), four), 3);
      __m256i z1 = _mm256_sub_epi32(_mm256_xor_si256(a, ds), ds);

      __m256i e = _mm256_abs_epi32(_mm256_sub_epi32(z1, vz));
      e = _mm256_max_epi32(e, load(&errFloor[idx]));
      _mm256_storeu_si256(reinterpret_cast<__m256i*>(&err[idx]), e);
      vmin = _mm256_min_epi32(vmin, e);
    }
    minError = simdMinLane(vmin);
#  elif defined(PCC_SIMD_SSE41)
    const __m128i vr0 = _mm_set1_epi32(int32_t(r0));
    const __m128i vz = _mm_set1_epi32(z);
    const __m128i round = _mm_set1_epi64x(1 << 14);
    const __m128i four = _mm_set1_epi32(4);
    __m128i vmin = _mm_set1_epi32(std::numeric_limits<int32_t>::max());
    auto load = [](const int32_t* v) {
      return _mm_loadu_si128(reinterpret_cast<const __m128i*>(v));
    };
    for (int idx = 0; idx < padded; idx += 4) {
      __m128i t = load(reinterpret_cast<const int32_t*>(&absTan[idx]));
      __m128i q = simdMulShiftLo(t, vr0, round, 15);
      __m128i s = load(&tanSign[idx]);
      __m128i z0 = _mm_sub_epi32(_mm_xor_si128(q, s), s);

      __m128i d = _mm_sub_epi32(z0, load(&zOffset[idx]));
      __m128i ds = _mm_srai_epi32(d, 31);
      __m128i a = _mm_srli_epi32(_mm_add_epi32(_mm_abs_epi32(d), four), 3);
      __m128i z1 = _mm_sub_epi32(_mm_xor_si128(a, ds), ds);

      __m128i e = _mm_abs_epi32(_mm_sub_epi32(z1, vz));
      e = _mm_max_epi32(e, load(&errFloor[idx]));
      _mm_storeu_si128(reinterpret_cast<__m128i*>(&err[idx]), e);
      vmin = _mm_min_epi32(vmin, e);
    }
    minError = simdMinLane(vmin);
#  endif

    // the first laser attaining the minimum, as the scalar search
    int idx = 0;
    while (err[idx] != minError)
      idx++;
    return idx;
  }

  static constexpr int kMaxLasers = 256;
  std::vector<uint32_t> absTan;
  std::vector<int32_t> tanSign;
  std::vector<int32_t> zOffset;
  std::vector<int32_t> errFloor;
  int64_t laserSearchMaxR;
#endif

  SphericalToCartesian sphToCartesian;
  static constexpr int32_t log2ScaleZ = 3;
  static constexpr int32_t log2ScaleTheta = 20;
  int32_t log2ScaleRadius;
  int32_t scalePhi;
  int numLasers;
  const int* tanThetaLaser;
  const int* zLaser;
};
class CartesianToSphericalSimple {
public:
  CartesianToSphericalSimple(const GeometryParameterSet& gps)
    : sphToCartesian(gps)
    , log2ScaleRadius(gps.geom_angular_radius_inv_scale_log2)
    , scalePhi(1 << (gps.geom_angular_azimuth_scale_log2_minus11 + 12))
    , azimLog2(gps.geom_angular_azimuth_scale_log2_minus11 + 12 - 1)
    , numLasers(gps.angularTheta.size())
    , tanThetaLaser(gps.angularTheta.data())
    , zLaser(gps.angularZ.data())
  {}

  Vec3<int32_t> operator()(Vec3<int32_t> xyz)
  {
    const int64_t xLaser = xyz[0] << 8;
    const int64_t yLaser = xyz[1] << 8;
    const int64_t r0 = isqrt(xLaser * xLaser + yLaser * yLaser) >> 8;
    int32_t thetaIdx = 0;
    int32_t minError = std::numeric_limits<int32_t>::max();
    for (int idx = 0; idx < numLasers; ++idx) {
      int64_t z = divExp2RoundHalfInf(
        tanThetaLaser[idx] * r0 << 2, log2ScaleTheta - log2ScaleZ);
      int64_t z1 = divExp2RoundHalfInf(z - zLaser[idx], log2ScaleZ);
      int32_t err = std::abs(z1 - xyz[2]);
      if (err < minError) {
        thetaIdx = idx;
        minError = err;
      }
    }
    const int64_t tanElevAng = iatan2(yLaser, xLaser);
    const int sh = 44 - azimLog2;
    const int off = 1 << (sh - 1);
    auto phi0 =
      (((tanElevAng + 3294199) * 5340354 + off) >> sh) - (1 << azimLog2);

    Vec3<int32_t> sphPos{
      int32_t(divExp2RoundHalfUp(r0, log2ScaleRadius)), int32_t(phi0),
      thetaIdx};    

    return sphPos;
  }

private:
  SphericalToCartesian sphToCartesian;
  static constexpr int32_t log2ScaleZ = 3;
  static constexpr int32_t log2ScaleTheta = 20;
  int32_t log2ScaleRadius;
  int32_t scalePhi;
  int32_t azimLog2;
  int numLasers;
  const int* tanThetaLaser;
  const int* zLaser;
};
typedef std::vector<Vec3<int>> VecVec3;
typedef std::vector<VecVec3> VecVecVec3;
typedef std::vector<VecVecVec3> VecVecVecVec3;

class PredGeomPredictor {
public:
  PredGeomPredictor() : numLasers(0), azimScaleLog2(0), interEnabled(false) {}

  void init(int azim_scale_log2, int nLasers, const bool globMotionEnabled, const bool resamplingEnabled)
  {
    if (!numLasers) {
      azimScaleLog2 = azim_scale_log2;
      numLasers = nLasers;
      globalMotionEnabled = globMotionEnabled;
      enableResampling = resamplingEnabled;
      refPointVals.resize(nLasers);
      refPointValsGlob.resize(nLasers);
      refPointValsCur.resize(nLasers);
    } else {
    }  // Already initialized
  }

  void insert(const std::vector<Vec3<int32_t>>& refFramePosSph)
  {
    assert(numLasers > 0);
    for (auto const& pt : refFramePosSph) {
      refPointValsCur[pt[2]].insert({computePhiQuantized(pt[1]), pt});
    }
  }

  std::pair<bool, point_t> getInterPred(
    const int currAzim, const int currLaserId, const int refNodeIdx) const
  {
    static const auto INVALID_REF = std::pair<bool, point_t>(false, 0);
    const auto& refPic = (refNodeIdx > 1) ? refPointValsGlob : refPointVals;
    const auto& refPoints = refPic[currLaserId];
    const bool nextPred = !(refNodeIdx & 0x1);
    if (!refPoints.size())
      return INVALID_REF;

    const auto quantizedPhi = computePhiQuantized(currAzim);
    auto idx = refPoints.upper_bound(quantizedPhi);
    if (idx == refPoints.end())
      return INVALID_REF;

    if (nextPred)
      return std::pair<bool, point_t>(true, idx->second);

    // nextnextPred
    idx = refPoints.upper_bound(idx->first);
    if (idx == refPoints.end())
      return INVALID_REF;
    else
      return std::pair<bool, point_t>(true, idx->second);
  }

  int computePhiQuantized(const int val) const
  {
    int offset = azimScaleLog2 ? (1 << (azimScaleLog2 - 1)) : 0;
    return val >= 0 ? (val + offset) >> azimScaleLog2
                    : -((-val + offset) >> azimScaleLog2);
  }

  bool isInterEnabled() { return interEnabled; }
  void setInterEnabled(bool enableFlag) { interEnabled = enableFlag; }
  void updateNextMovingStatus(const bool status)
  {
    motionParams.updateNextMovingStatus(status);
  }
  void updateCurMovingStatue(const int currPicIdx, const int refPicIdx){
    std::pair<int, int> currThresh;
    std::vector<int64_t> currMatrix;
    Vec3<int> currVector;
    motionParams.getMotionParamsMultiple2(currThresh, currMatrix, currVector, currPicIdx, refPicIdx);
    std::vector<int> currMatrix_int;
    currMatrix_int.resize(9);
    for(int i = 0; i < 9; i++)
      currMatrix_int[i] = int(currMatrix[i]);
    bool status = motionParams.checkMovingStatus(currMatrix_int, currVector);
    updateMovingStatus( status, currPicIdx - 1 );
  }

  void updateMovingStatus(const bool status, const int Ctr){
    motionParams.updateMovingStatus(status, Ctr);
  }
  void clearRefFrame()
  {
    for (auto& laserPts : refPointVals)
      laserPts.clear();
  }
  void clearRefFrameCur()
  {
    for (auto& laserPts : refPointValsCur)
      laserPts.clear();
  }
  void advanceFrame() { motionParams.AdvanceFrame();}
  void setFrameCtr(const int frameCtr) { motionParams.setFrameCtr(frameCtr); }
  void setRefFrameCtr(const int refFrameCtr) { motionParams.setRefFrameCtr(refFrameCtr); }
  int getFrameCtr() {return motionParams.getFrameCtr(); }

  void parseMotionParams(std::string fileName, double qs)
  {
    motionParams.parseFile(fileName, qs);
  }
  void updateFrame(const GeometryParameterSet& gps)
  {
    motionParams.AdvanceFrame();
    if (globalMotionEnabled) {
      CartesianToSphericalSimple cartToSpherical(gps);
      SphericalToCartesian sphericalToCart(gps);
      std::pair<int, int> currThresh;
      std::vector<int64_t> currMatrix;
      Vec3<int> currVector;
      for (auto& laserPts : refPointValsGlob)
        laserPts.clear();

      if (!gps.biPredictionEnabledFlag)
        motionParams.getMotionParams(currThresh, currMatrix, currVector);
      else{
        motionParams.getMotionParamsMultiple2(currThresh, currMatrix, currVector, motionParams.getFrameCtr() + 1, motionParams.getRefFrameCtr() + 1);
      }

      for (auto laserId = 0; laserId < numLasers; laserId++) {
        for (auto ptIter : refPointValsCur[laserId]) {
          auto pt = sphericalToCart(ptIter.second);
          if (pt[2] > currThresh.first || pt[2] < currThresh.second) {
            auto p = pt;
            for (auto k = 0; k < 3; k++) {
              int64_t x =
                divExp2RoundHalfInfPositiveShift(
                  currMatrix[3 * k + 0] * p[0] + currMatrix[3 * k + 1] * p[1]
                    + currMatrix[3 * k + 2] * p[2],
                  16, 1 << 15)
                + currVector[k];
              pt[k] = x;
            }

            pt = cartToSpherical(pt);
          } else
            pt = ptIter.second;
          const auto phiQ = computePhiQuantized(pt[1]);
          if (!refPointValsGlob[pt[2]].count(phiQ)){
            refPointValsGlob[pt[2]].insert({phiQ, pt});
          }
          else if (refPointValsGlob[pt[2]].at(phiQ)[0] > pt[0]){
            refPointValsGlob[pt[2]].at(phiQ) = pt;
          }

        }
      }

      if (getFrameMovingState()) {
        if (enableResampling) {
          for (auto laserId = 0; laserId < numLasers; laserId++) {
            auto& ptsZero = refPointValsCur[laserId];
            auto& ptsGlob = refPointValsGlob[laserId];
            for (auto& ptIter : ptsZero) {
              pcc::point_t ptA = 0, ptB = 0;
              auto& pt = ptIter.second;
              auto phiQ = computePhiQuantized(pt[1]);
              if (ptsGlob.count(phiQ)) {
                const auto& colPt = ptsGlob[phiQ];
                ptA = colPt;
                if (colPt[1] < pt[1]) {
                  auto idx = ptsGlob.upper_bound(phiQ);
                  ptB = (idx == ptsGlob.end()) ? ptA : idx->second;
                } else if (colPt[1] > pt[1]) {
                  auto idx = ptsGlob.lower_bound(phiQ);
                  ptB =
                    (idx == ptsGlob.begin()) ? ptA : std::prev(idx)->second;
                } else
                  ptB = ptA;
              } else {
                auto idx = ptsGlob.upper_bound(phiQ);
                auto idx1 = idx;
                if (idx != ptsGlob.begin())
                  idx1 = std::prev(idx);
                if (idx == ptsGlob.end())
                  idx = idx1;
                ptA = idx->second;
                ptB = idx1->second;
              }
              int64_t delAzim = ptA[1] - ptB[1];
              int64_t delRad = (ptA[0] - ptB[0]);
              if (!delAzim || !delRad)
                pt[0] = ptA[0];
              else {
                const auto nr = delRad * (pt[1] - ptA[1]);
                const auto dr = delAzim;
                const bool sign =
                  ((nr > 0 && dr > 0) || (nr < 0 && dr < 0)) ? 0 : 1;
                pt[0] = ptA[0]
                  + (1 - 2 * sign) * (std::abs(nr) + (std::abs(dr) >> 1))
                    / std::abs(dr);
              }
            }
          }
        }
      } 
      else
        for (auto laserId = 0; laserId < numLasers; laserId++)
          refPointValsGlob[laserId] = std::move(refPointVals[laserId]);

      for (auto laserId = 0; laserId < numLasers; laserId++)
        refPointVals[laserId] = std::move(refPointValsCur[laserId]);
    } 
    else {
      for (auto laserId = 0; laserId < numLasers; laserId++)
        refPointVals[laserId] = std::move(refPointValsCur[laserId]);
    }
  }
  template <typename T>
  void getMotionParams(std::pair<int, int> &th, std::vector<T> &mat, Vec3<int> &tr) const
  {
    motionParams.getMotionParams(th, mat, tr);
  }
  template <typename T>
  void getMotionParamsMultiple(std::pair<int, int> &th, std::vector<T> &mat, Vec3<int> &tr, const int _curPicIndex, const int _refPicIndex) const
  {
    motionParams.getMotionParamsMultiple2(th, mat, tr, _curPicIndex, _refPicIndex);
  }
  void setMotionParams(std::pair<int, int> &th, std::vector<int> &mat, Vec3<int> &tr) 
  {
    motionParams.setMotionParams(th, mat, tr);
  }
  
  void setGlobalMotionEnabled(bool GlobalMotionEnabled){
    globalMotionEnabled = GlobalMotionEnabled;
  }
  bool getGlobalMotionEnabled() { return globalMotionEnabled; }
  void updateThresholds(
    const int frameCounter, const int leftThresh, const int rightTresh)
  {
    motionParams.updateThresholds(frameCounter, leftThresh, rightTresh);
  }
  bool getFrameMovingState() { return motionParams.getMovingStatus(); }
  void setFrameMovingState(const bool movsts)
  {
    motionParams.setMovingStatus(movsts);
  }
 private:
  std::vector<std::map<int, pcc::point_t>> refPointVals;
  std::vector<std::map<int, pcc::point_t>> refPointValsGlob;
  std::vector<std::map<int, pcc::point_t>> refPointValsCur;
  MotionParameters motionParams;
  int numLasers;
  int azimScaleLog2;
  bool interEnabled;
  bool globalMotionEnabled;
  bool enableResampling;
};

//============================================================================

}  // namespace pcc
