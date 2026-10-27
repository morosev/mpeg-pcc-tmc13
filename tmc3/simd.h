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

//============================================================================
// Selection of the instruction set used by the SIMD kernels.
//
// One of the following is defined, according to the target:
//  - PCC_SIMD_NEON:  AArch64 NEON (always available on AArch64)
//  - PCC_SIMD_SSE41: x86 SSE4.1 (also defined when AVX2 is used)
//  - PCC_SIMD_AVX2:  x86 AVX2, in addition to PCC_SIMD_SSE41
// and PCC_SIMD is defined if any of them is.
//
// SSE4.1 and AVX2 are selected by the compiler target options (eg,
// -msse4.1, -mavx2).  Compilers that do not report SSE4.1 support (eg,
// MSVC) may define PCC_ENABLE_SSE41 instead.
//
// Defining PCC_DISABLE_SIMD selects the scalar code on every target.  All
// SIMD kernels produce results identical to the scalar code.

#if !defined(PCC_DISABLE_SIMD)
#  if (defined(__aarch64__) && defined(__ARM_NEON)) || defined(_M_ARM64)
#    define PCC_SIMD_NEON 1
#  elif defined(__AVX2__)
#    define PCC_SIMD_AVX2 1
#    define PCC_SIMD_SSE41 1
#  elif defined(__SSE4_1__) || defined(PCC_ENABLE_SSE41)
#    define PCC_SIMD_SSE41 1
#  endif
#endif

#if defined(PCC_SIMD_NEON) || defined(PCC_SIMD_SSE41)
#  define PCC_SIMD 1
#endif

#if defined(PCC_SIMD_NEON)
#  include <arm_neon.h>
#elif defined(PCC_SIMD_AVX2)
#  include <immintrin.h>
#elif defined(PCC_SIMD_SSE41)
#  include <smmintrin.h>
#endif

#include <cstdint>

namespace pcc {

#if defined(PCC_SIMD_SSE41)
//----------------------------------------------------------------------------
// Helpers for the x86 kernels.

// The minimum of the four signed lanes of v
inline int32_t
simdMinLane(__m128i v)
{
  v = _mm_min_epi32(v, _mm_shuffle_epi32(v, _MM_SHUFFLE(1, 0, 3, 2)));
  v = _mm_min_epi32(v, _mm_shuffle_epi32(v, _MM_SHUFFLE(2, 3, 0, 1)));
  return _mm_cvtsi128_si32(v);
}

// The four lane masks of m (all ones or zero) packed into the low bits
inline uint32_t
simdLaneBits(__m128i m)
{
  return uint32_t(_mm_movemask_ps(_mm_castsi128_ps(m)));
}

// The low 32 bits of each 64-bit product of the unsigned 32-bit lanes of a
// and b, after adding round and shifting right by shift.  Ie, for each
// lane i: uint32_t((uint64_t(a[i]) * b[i] + round) >> shift).
inline __m128i
simdMulShiftLo(__m128i a, __m128i b, __m128i round, int shift)
{
  // NB: _mm_mul_epu32 multiplies the even lanes only
  __m128i even = _mm_mul_epu32(a, b);
  __m128i odd = _mm_mul_epu32(_mm_srli_epi64(a, 32), _mm_srli_epi64(b, 32));
  even = _mm_srl_epi64(_mm_add_epi64(even, round), _mm_cvtsi32_si128(shift));
  odd = _mm_srl_epi64(_mm_add_epi64(odd, round), _mm_cvtsi32_si128(shift));
  return _mm_blend_epi16(even, _mm_slli_epi64(odd, 32), 0xcc);
}
#endif

#if defined(PCC_SIMD_AVX2)
// The eight lane masks of m (all ones or zero) packed into the low bits
inline uint32_t
simdLaneBits(__m256i m)
{
  return uint32_t(_mm256_movemask_ps(_mm256_castsi256_ps(m)));
}

// As simdMulShiftLo, for eight lanes
inline __m256i
simdMulShiftLo(__m256i a, __m256i b, __m256i round, int shift)
{
  __m256i even = _mm256_mul_epu32(a, b);
  __m256i odd =
    _mm256_mul_epu32(_mm256_srli_epi64(a, 32), _mm256_srli_epi64(b, 32));
  even = _mm256_srl_epi64(
    _mm256_add_epi64(even, round), _mm_cvtsi32_si128(shift));
  odd = _mm256_srl_epi64(
    _mm256_add_epi64(odd, round), _mm_cvtsi32_si128(shift));
  return _mm256_blend_epi32(even, _mm256_slli_epi64(odd, 32), 0xaa);
}

// The minimum of the eight signed lanes of v
inline int32_t
simdMinLane(__m256i v)
{
  return simdMinLane(_mm_min_epi32(
    _mm256_castsi256_si128(v), _mm256_extracti128_si256(v, 1)));
}
#endif

//============================================================================

}  // namespace pcc
