/* The copyright in this software is being made available under the BSD
 * Licence, included below.  This software may be subject to other third
 * party and contributor rights, including patent rights, and no such
 * rights are granted under this licence.
 *
 * Copyright (c) 2026, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "PCCMath.h"
#include "PCCMisc.h"

#include <cstdint>
#include <limits>
#include <vector>

#if defined(__ARM_NEON)
#  include <arm_neon.h>
#endif
#if defined(_MSC_VER)
#  include <intrin.h>
#endif

namespace pcc {

//============================================================================
// Helpers for the bucketed candidate scans of the LoD nearest neighbour
// search.  The scans compute many L1 distances (to points and to bounding
// boxes) of which only a few lead to a neighbour list update.  The distances
// are computed in bulk; the (order dependent) list updates remain scalar so
// that the result is identical to a purely scalar search.

#if defined(__ARM_NEON)
constexpr bool kLodScanSimd = true;
#else
constexpr bool kLodScanSimd = false;
#endif

//----------------------------------------------------------------------------
// Structure-of-arrays copy of a set of candidate positions.

struct LodScanPoints {
  std::vector<int32_t> x, y, z;

  // Gather pos[idx[i]] for i in [0, count)
  template<typename IdxIt>
  void build(const std::vector<Vec3<int32_t>>& pos, IdxIt idx, int32_t count)
  {
    x.resize(count);
    y.resize(count);
    z.resize(count);
    for (int32_t i = 0; i < count; ++i) {
      const auto& p = pos[idx[i]];
      x[i] = p[0];
      y[i] = p[1];
      z[i] = p[2];
    }
  }
};

//----------------------------------------------------------------------------
// Structure-of-arrays copy of one level of bounding boxes.

struct LodScanBoxes {
  std::vector<int32_t> minX, minY, minZ, maxX, maxY, maxZ;

  void build(const std::vector<Box3<int32_t>>& boxes)
  {
    const int32_t count = int32_t(boxes.size());
    minX.resize(count);
    minY.resize(count);
    minZ.resize(count);
    maxX.resize(count);
    maxY.resize(count);
    maxZ.resize(count);
    for (int32_t i = 0; i < count; ++i) {
      minX[i] = boxes[i].min[0];
      minY[i] = boxes[i].min[1];
      minZ[i] = boxes[i].min[2];
      maxX[i] = boxes[i].max[0];
      maxY[i] = boxes[i].max[1];
      maxZ[i] = boxes[i].max[2];
    }
  }
};

//----------------------------------------------------------------------------
// Clear and return the index of the lowest (or highest if Reverse) set bit
// of the non-zero mask.

template<bool Reverse>
inline int
lodPopBit(uint32_t& mask)
{
#if defined(__GNUC__) || defined(__clang__)
  const int i = Reverse ? 31 - __builtin_clz(mask) : __builtin_ctz(mask);
#elif defined(_MSC_VER)
  unsigned long i;
  if (Reverse)
    _BitScanReverse(&i, mask);
  else
    _BitScanForward(&i, mask);
#else
  const int i = Reverse ? ilog2(mask) : ilog2(mask & (~mask + 1));
#endif
  mask &= ~(uint32_t(1) << i);
  return int(i);
}

//----------------------------------------------------------------------------
// Call f(i) for each set bit i of mask, in ascending (or descending if
// Reverse) order.

template<bool Reverse, typename F>
inline void
lodForEachBit(uint32_t mask, F f)
{
  while (mask)
    f(lodPopBit<Reverse>(mask));
}

#if defined(__ARM_NEON)
//----------------------------------------------------------------------------
// Pack the four lane flags of m into the low bits of the result.

inline uint32_t
lodLaneBits(uint32x4_t m)
{
  static const uint32_t kBits[4] = {1, 2, 4, 8};
  return vaddvq_u32(vandq_u32(m, vld1q_u32(kBits)));
}
#endif

//----------------------------------------------------------------------------
// For b in [b0, b1], with b1 - b0 < 32:
//  - dist[b - b0] = boxes[b].getDist1(p);
//  - bit (b - b0) of the result is set if dist[b - b0] < limit.

inline uint32_t
lodBoxScan(
  const LodScanBoxes& bx,
  int32_t b0,
  int32_t b1,
  const Vec3<int32_t>& p,
  int64_t limit,
  int32_t* dist)
{
  uint32_t mask = 0;
  int32_t b = b0;
#if defined(__ARM_NEON)
  // NB: a limit beyond the int32 range keeps every box
  const bool keepAll = limit > std::numeric_limits<int32_t>::max();
  const int32x4_t lim = vdupq_n_s32(keepAll ? 0 : int32_t(limit));
  const int32x4_t px = vdupq_n_s32(p[0]);
  const int32x4_t py = vdupq_n_s32(p[1]);
  const int32x4_t pz = vdupq_n_s32(p[2]);
  const int32x4_t zero = vdupq_n_s32(0);
  for (; b + 3 <= b1; b += 4) {
    // per axis: max(max(min - p, 0), p - max), as Box3::getDist1
    int32x4_t dx = vmaxq_s32(
      vmaxq_s32(vsubq_s32(vld1q_s32(&bx.minX[b]), px), zero),
      vsubq_s32(px, vld1q_s32(&bx.maxX[b])));
    int32x4_t dy = vmaxq_s32(
      vmaxq_s32(vsubq_s32(vld1q_s32(&bx.minY[b]), py), zero),
      vsubq_s32(py, vld1q_s32(&bx.maxY[b])));
    int32x4_t dz = vmaxq_s32(
      vmaxq_s32(vsubq_s32(vld1q_s32(&bx.minZ[b]), pz), zero),
      vsubq_s32(pz, vld1q_s32(&bx.maxZ[b])));
    int32x4_t d = vaddq_s32(vaddq_s32(dx, dy), dz);
    vst1q_s32(dist + (b - b0), d);
    uint32_t bits = keepAll ? 0xf : lodLaneBits(vcltq_s32(d, lim));
    mask |= bits << (b - b0);
  }
#endif
  for (; b <= b1; ++b) {
    int32_t dx = std::max(std::max(bx.minX[b] - p[0], 0), p[0] - bx.maxX[b]);
    int32_t dy = std::max(std::max(bx.minY[b] - p[1], 0), p[1] - bx.maxY[b]);
    int32_t dz = std::max(std::max(bx.minZ[b] - p[2], 0), p[2] - bx.maxZ[b]);
    dist[b - b0] = dx + dy + dz;
    if (dist[b - b0] < limit)
      mask |= uint32_t(1) << (b - b0);
  }
  return mask;
}

//----------------------------------------------------------------------------
// Call visit(h) for each candidate h in [h0, h1], with h1 - h0 < 32,
// (ascending, or descending if Reverse) that may update the neighbour list,
// ie, whose L1 distance to p is not greater than threshold.
//
// NB: the candidates are filtered using the threshold on entry.  Since it
//     never increases, any candidate skipped here would also have been
//     rejected by the scalar update, which still performs the exact test
//     for every visited candidate.

template<bool Reverse, typename Visit>
inline void
lodScanCandidates(
  const LodScanPoints& c,
  int32_t h0,
  int32_t h1,
  const Vec3<int32_t>& p,
  int64_t threshold,
  Visit visit)
{
#if defined(__ARM_NEON)
  if (h1 < h0)
    return;

  // NB: a threshold beyond the int32 range keeps every candidate
  if (threshold >= std::numeric_limits<int32_t>::max()) {
    lodForEachBit<Reverse>(
      uint32_t(uint64_t(2) << (h1 - h0)) - 1, [&](int i) { visit(h0 + i); });
    return;
  }

  const int32x4_t thr = vdupq_n_s32(int32_t(threshold));
  const int32x4_t px = vdupq_n_s32(p[0]);
  const int32x4_t py = vdupq_n_s32(p[1]);
  const int32x4_t pz = vdupq_n_s32(p[2]);

  uint32_t mask = 0;
  int32_t h = h0;
  for (; h + 3 <= h1; h += 4) {
    // |dx| + |dy| + |dz| <= threshold
    int32x4_t d = vabsq_s32(vsubq_s32(px, vld1q_s32(&c.x[h])));
    d = vaddq_s32(d, vabsq_s32(vsubq_s32(py, vld1q_s32(&c.y[h]))));
    d = vaddq_s32(d, vabsq_s32(vsubq_s32(pz, vld1q_s32(&c.z[h]))));
    mask |= lodLaneBits(vcleq_s32(d, thr)) << (h - h0);
  }
  for (; h <= h1; ++h)
    mask |= uint32_t(1) << (h - h0);

  lodForEachBit<Reverse>(mask, [&](int i) { visit(h0 + i); });
#else
  if (!Reverse) {
    for (int32_t h = h0; h <= h1; ++h)
      visit(h);
  } else {
    for (int32_t h = h1; h >= h0; --h)
      visit(h);
  }
#endif
}

//============================================================================

}  // namespace pcc
