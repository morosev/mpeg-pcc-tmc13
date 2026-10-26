SIMD optimisation of TMC13 (ARM NEON, Intel/AMD x86)
=====================================================

This document records the analysis of TMC13 (`tmc3`) for SIMD
optimisation, the work done so far for ARM NEON, and the status and plan
for Intel/AMD x86 (SSE/AVX2).

The goal is real-time coding of spinning lidar at 10 Hz: 100 ms per frame
for encoding and for decoding, measured as end-to-end wall-clock time
(including ply reading and the input azimuth sort) on a single thread.


General analysis
----------------

### Starting point

- `tmc3` contains no SIMD code: no intrinsics and no platform-specific
  paths.
- Compiler auto-vectorisation contributes nothing measurable.  Builds
  with and without `-fno-vectorize -fno-slp-vectorize` (clang, `-O3`)
  have identical run times.  The hot loops are short, branchy,
  data-dependent and use array-of-structures data, which defeats the
  vectoriser.
- The project's CMake files do not set a build type.  A plain `cmake ..`
  produces an unoptimised build, so use `-DCMAKE_BUILD_TYPE=Release` for
  any performance measurement.

### Constraint: bit-exactness

TMC13 is the reference software: bitstreams and reconstructions must not
change.  Every optimisation here is required to produce byte-identical
output to the scalar code, and SIMD paths must reproduce the exact
integer rounding of the scalar code.  In practice:

- SIMD is used to compute values (distances, errors) in bulk, but
  order-dependent decisions (neighbour list updates, first-minimum
  selection) stay in the original scalar order.
- A vector pre-filter may skip work only when it can be shown that the
  scalar code would have rejected it too.
- Floating-point code (`atan2`, `hypot`, the encoder-side recolouring) is
  left alone, since vector maths libraries do not give identical results.

### Where the time goes

Profiles were taken with macOS `sample` on an Apple M5 Pro, on 30
synthetic Ford-like lidar frames (64 lasers using the Ford laser
parameters, about 143k points per frame) coded with the CTC lidar
settings (angular mode, spherical coordinates for attributes), and on a
dense coloured object.

| Area | Where | Share of time | SIMD suitability |
|---|---|---|---|
| LoD nearest neighbour search | `computeNearestNeighbors` (`PCCTMC3Common.h`) | 23-64% of lidar decode | Good: 32-bit L1 distances to points and bounding boxes |
| Arithmetic coding | `schro_arith_*`, `CtxMapDynamicOBUF` | 10-25% of decode | None: serial bit-by-bit dependency |
| Cartesian to spherical conversion | `CartesianToSpherical` (`geometry_predictive.h`) | ~16% of predgeom encode | Good for the laser search; trigonometry and `atan2` stay scalar |
| Bit-cost estimation | `PredGeomEncoder::estimateBits` | ~8% of predgeom encode (libm `log2`) | Table lookup instead of SIMD |
| Input azimuth sort | `orderByAzimuth` / `findLaserPrecise` | ~8% of end-to-end encode | Good for the laser search |
| RAHT neighbour search | `findNeighbours` (`RAHT.cpp`) | 24-32% of RAHT decode | Mostly memory latency: binary searches over 48-byte nodes; needs a layout change first |
| RAHT transform | `fwd/invTransformBlock222`, `intraDcPred` | ~15% of RAHT decode | Limited: 64-bit fixed point (no 64x64-bit lane multiply in NEON, SSE or AVX2) |
| Encoder recolouring | nanoflann KD-tree (`recolourColour`) | 30-60% of octree encode | Algorithmic first: double precision, dynamic dimension |
| Trisoup ray tracing | `rayTracingAlongdirection` | ~11% of trisoup decode | Good: 32-bit integer rays on a regular grid |

For lidar, the LoD (predicting/lifting) attribute path is the main
target: for sparse data the 27-cell neighbourhood rarely holds three
neighbours, so almost every point falls back to the bucketed range scan,
which is the most vectorisable loop in the codec.

### Instruction set notes

| Need | ARM NEON (AArch64) | x86 |
|---|---|---|
| Baseline availability | Always available on AArch64 | SSE2 on x86-64; SSE4.1/AVX2 need a dispatch or build flag |
| 32-bit lanes per register | 4 | 4 (SSE), 8 (AVX2) |
| Absolute value of int32 | `vabsq_s32` | `_mm_abs_epi32` (SSSE3) |
| Comparison to a bitmask | `vcleq_s32` + lane-bit packing | `_mm_cmpgt_epi32` + `_mm_movemask_ps` |
| Horizontal minimum | `vminvq_s32` | shuffle/`_mm_min_epi32` (SSE4.1) reduction |
| 32x32 to 64-bit multiply | `vmull_u32` / `vmull_high_u32` (4 lanes with 2 instructions) | `_mm_mul_epu32` (2 lanes, even elements only) |
| Rounding narrowing shift | `vrshrn_n_u64` | add, shift, then shuffle to narrow |
| 64x64-bit lane multiply | Not available | AVX-512DQ `vpmullq` only |

The lack of a 64-bit lane multiply below AVX-512 is why the RAHT transform
(64-bit `FixedPoint`) is a poor SIMD target on both architectures unless
its operands can be shown to fit 32 bits.


Work done (ARM NEON)
--------------------

All SIMD code is guarded by `#if defined(__ARM_NEON)` and has a scalar
path that produces identical results, so other targets build and behave
as before.

### LoD nearest neighbour bucket scans (commit 9324a7c)

Files: `tmc3/lod_scan_simd.h` (new), `tmc3/PCCTMC3Common.h`.

- Candidate positions and the bounding boxes of the scan hierarchy are
  copied into structure-of-arrays form (`LodScanPoints`, `LodScanBoxes`)
  once per call.
- `lodBoxScan` computes the L1 distance from the point to up to 32 boxes,
  four at a time, and returns a bitmask of the boxes that cannot be
  pruned.
- `lodScanCandidates` computes `|dx|+|dy|+|dz|` for up to 32 candidates,
  four at a time, and visits only those not beyond the current
  third-nearest distance.
- Only the set bits are visited, which removes most of the unpredictable
  branches in the scan loops.  This mattered as much as the vector
  arithmetic: a version that only vectorised the distances gave no gain
  for the predicting transform.
- Exactness: the existing scalar update functions still test every
  visited box and candidate against the live threshold.  The threshold
  never increases, so anything skipped by the vector filter would also
  have been rejected.

Used by the inter-LoD fallback scans and the intra-LoD scan, ie. by the
predicting and lifting transforms, in the encoder and the decoder.

### Predictive geometry encoder and input (commit 1b07bd5)

Files: `tmc3/geometry_predictive.h`, `tmc3/geometry_predictive_encoder.cpp`,
`tmc3/PCCPointSet.h`, `tmc3/PCCTMC3Common.h`, `tmc3/ply.cpp`.

- **Bit-cost estimation** (scalar): `estimate()` evaluated
  `-log2(p / 128)` with libm for a 7-bit probability.  It now reads a
  128-entry table built at startup with the same expression.
- **Spherical refinement** (scalar): the 5x5 local refinement in
  `CartesianToSpherical` evaluated the full conversion (including
  `icos`/`isin`) for each of 25 candidates.  The candidates share the
  laser, so the 5 radii, heights and sin/cos pairs are evaluated once.
  `SphericalToCartesian` is expressed in terms of the same component
  helpers.
- **Laser search in `CartesianToSpherical`** (NEON): four lasers per
  register in 32-bit lanes.  For `r0 >= 0`, rounding
  `tan * r0 * 4 / 2^17` half away from zero equals
  `sign(tan) * ((|tan| * r0 + 2^14) >> 15)`, which is one widening
  multiply and one rounding narrowing shift (`vrshrn_n_u64`).  The first
  laser with the minimum error is selected as in the scalar loop.  The
  NEON path is used only when all intermediate values are known to fit
  in 32 bits.
- **`findLaserPrecise`** (NEON): used by the input azimuth sort and by
  octree angular coding.  `(|zS3| * rInv) >> 17` is a 32x32-bit product
  truncated to 32 bits, exactly as the scalar `int` conversion; used for
  `rInv < 2^32`.
- **Nearest neighbour search** (scalar): the per-point
  `std::vector<bool>` heap allocation is replaced by a fixed array.
- **`ply::read`** (scalar): binary vertex data is read in one block and
  parsed from memory, instead of one stream read per property per point.
  End-of-data behaviour matches `std::istream::read`.  The block is the
  rest of the file, not a size computed from the property byte counts,
  because some property types (eg. `ushort`) are consumed without a known
  byte count.

### Results

Apple M5 Pro, single thread, 30 frames, per-frame time.  The data is
synthetic Ford-like lidar.

Reflectance (attribute) coding time, LoD scan change only (commit
9324a7c):

| Configuration | Decode | Encode |
|---|---|---|
| predgeom + lifting | 1.56x | 1.54x |
| octree + lifting | 1.47x | 1.46x |
| predgeom + predicting | 1.32x | 1.32x |
| octree + predicting | 1.26x | 1.24x |

End-to-end wall time per frame, predictive geometry (the real-time
target):

| | Original | 9324a7c | 1b07bd5 |
|---|---|---|---|
| Encode, predgeom + predicting | 122.8 ms | 111.6 ms | **91.3 ms** |
| Encode, predgeom + lifting | 145.9 ms | 120.0 ms | **98.7 ms** |
| Decode, predgeom + predicting | 75.1 ms | | **61.3 ms** |
| Decode, predgeom + lifting | 96.9 ms | | **68.6 ms** |

Octree configurations are not yet real-time: their encoder is dominated
by the recolouring KD-tree, and octree + predicting decode by octree
geometry decoding.

### Verification

Each change was checked with:

- Byte-identical bitstreams and decoded frames against the original
  build, for four lidar configurations (predgeom/octree geometry with
  predicting/lifting transforms), with both a NEON build and a scalar
  fallback build (`-U__ARM_NEON`).
- Additional configurations: `predictionWithDistributionEnabled=0`, and
  dense colour content with lifting and predicting transforms.
- Randomised NEON versus scalar comparisons of `CartesianToSpherical`
  (800k conversions over 400 random laser tables, including inputs that
  take the scalar fallback) and `findLaserPrecise` (1M searches,
  including ties, small radii and laser counts not a multiple of four).
- Identical results from the old and new ply readers on lidar, dense
  colour, float64, truncated and trailing-data files.
- AddressSanitizer and UndefinedBehaviorSanitizer runs.  UBSan only
  reports left shifts of negative values that the original code already
  performs and the new code keeps, to stay bit-exact.

Measured but reverted (no measurable gain): calling the inline
arithmetic coder functions directly, and keeping LoD search buffers
between calls.


Intel/AMD (x86) status
----------------------

No x86 SIMD paths exist yet.  On x86 the scalar paths are used, with
these consequences:

- Output is identical to the original software.
- The table lookup, spherical refinement, allocation and ply reader
  changes are scalar and benefit x86 as well.
- The restructured LoD scan loops are 3-5% slower than the original code
  when the scalar path is used (measured on ARM with `-U__ARM_NEON`),
  because the bulk distance and bitmask work is done without SIMD.

### Porting plan

The NEON kernels map onto SSE4.1 (four 32-bit lanes) and AVX2 (eight),
keeping the same exactness arguments:

- `lodBoxScan`, `lodScanCandidates`: `_mm_sub_epi32`, `_mm_abs_epi32`,
  `_mm_max_epi32`, `_mm_add_epi32`; comparison masks packed with
  `_mm_movemask_ps(_mm_castsi128_ps(...))` (four bits per vector, or
  eight with `_mm256_movemask_ps`).  With AVX2, one 32-entry run is four
  vectors.
- Laser searches: the 32x32 to 64-bit products need `_mm_mul_epu32`,
  which multiplies only the even elements, so two multiplies plus
  shuffles produce four lanes; the shift and narrowing are done with
  `_mm_srli_epi64` and a shuffle.  The first minimum is found with a
  `_mm_min_epi32` reduction followed by a scalar scan, as on ARM.
- `lodPopBit` already supports MSVC through `_BitScanForward` and
  `_BitScanReverse`, but that branch has not been compiled yet.

Build considerations: SSE4.1 and AVX2 are not part of the x86-64
baseline, so either build with `-msse4.1`/`-mavx2` (or MSVC
`/arch:AVX2`), or add runtime dispatch.  The same bit-exact verification
(NEON vs scalar becomes SSE/AVX2 vs scalar) applies.


Remaining work
--------------

- Port the kernels to SSE4.1/AVX2, and remove the scalar LoD scan
  overhead.
- Confirm all results on real Ford sequences: the measurements so far use
  synthetic frames (about 143k points, more than typical Ford frames).
- More margin for predgeom + lifting encode (98.7 ms): the remaining time
  is mostly the LoD candidate scans, the arithmetic coder, `atan2`, and
  the per-point laser search in the input azimuth sort.  Reading and
  sorting the next frame on a second thread while the current frame is
  encoded would hide most of the ~10 ms of input work without changing
  the bitstream, at the cost of a second core.
- Octree configurations: replace the double-precision, dynamic-dimension
  KD-tree used for recolouring (encoder), and look at octree geometry
  decoding.
- RAHT: store node positions as a separate array and use branch-free or
  SIMD k-ary search in `findNeighbours`; check whether the transform's
  operands fit 32 bits.
- Trisoup: vectorise `rayTracingAlongdirection` (four rays per vector).
