cmake_minimum_required(VERSION 3.0)
enable_language(C)

if (UNIX)
  add_definitions(-D_POSIX_C_SOURCE=200809L)
elseif (WIN32)
  add_definitions(-D_USE_MATH_DEFINES)
endif()

include(CheckSymbolExists)
check_symbol_exists(getrusage sys/resource.h HAVE_GETRUSAGE)

##
# Instruction set used by the SIMD kernels (see doc/README.simd.md).
# All choices produce identical output.
#  - auto:   NEON on AArch64, SSE4.1 on x86, otherwise none
#  - none:   scalar code only
#  - sse4.1: x86 SSE4.1
#  - avx2:   x86 AVX2
set(TMC3_SIMD "auto" CACHE STRING "SIMD instruction set: auto, none, sse4.1, avx2")
set_property(CACHE TMC3_SIMD PROPERTY STRINGS auto none sse4.1 avx2)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64|x86|i[3-6]86)$")
  set(TMC3_TARGET_X86 TRUE)
endif()

if (TMC3_SIMD STREQUAL "none")
  add_definitions(-DPCC_DISABLE_SIMD)
elseif (TMC3_SIMD STREQUAL "avx2")
  if (MSVC)
    add_compile_options(/arch:AVX2)
  else()
    add_compile_options(-mavx2)
  endif()
elseif (TMC3_SIMD STREQUAL "sse4.1"
        OR (TMC3_SIMD STREQUAL "auto" AND TMC3_TARGET_X86))
  # NB: MSVC has no option to select SSE4.1; the intrinsics are always
  #     available, so the kernels are enabled explicitly.
  if (MSVC)
    add_definitions(-DPCC_ENABLE_SSE41)
  else()
    add_compile_options(-msse4.1)
  endif()
elseif (NOT TMC3_SIMD STREQUAL "auto")
  message(FATAL_ERROR "Unknown TMC3_SIMD value: ${TMC3_SIMD}")
endif()

##
# Determine the software version from VCS
# Fallback to descriptive version if VCS unavailable
#  NB: the "unconfirmed-" part refers to this being a fallback option
#      and the reported version may or may not be accurate.
set(VERSION_FALLBACK "unconfirmed-release-23.0-rc2")
set(VERSION_FILE ${CMAKE_CURRENT_BINARY_DIR}/version.cpp)
set(VERSION_FILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/version.cpp.in)
set(VERSION_CACHE_FILE ${CMAKE_CURRENT_BINARY_DIR}/version.cache)
find_package(Git)
add_custom_target(
  genversion
  COMMAND ${CMAKE_COMMAND} -D OUTPUT=${VERSION_FILE}
                           -D TEMPLATE=${VERSION_FILE_IN}
                           -D VERSION_FALLBACK=${VERSION_FALLBACK}
                           -D VERSION_EXTRA=${VERSION_EXTRA}
                           -D VERSION_CACHE_FILE=${VERSION_CACHE_FILE}
                           -D GIT_EXECUTABLE=${GIT_EXECUTABLE}
                           -P ${CMAKE_SOURCE_DIR}/scripts/genversion.cmake
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

add_custom_command(
  OUTPUT ${VERSION_FILE}
  DEPENDS genversion
)

configure_file (
  "${CMAKE_CURRENT_SOURCE_DIR}/TMC3Config.h.in"
  "${PROJECT_BINARY_DIR}/tmc3/TMC3Config.h"
)

#
# NB: avoid the use of wild cards for source files in this directory,
# since cmake is unable to detect additions or removals in such cases
# resulting in obscure broken builds when switching branches or
# otherwise adding/removing files.
#
# Dependencies are permitted to use the wild card since they are
# not expected to change without modifying this CMakeLists.txt.
#

file(GLOB PROJECT_IN_FILES
  "TMC3Config.h.in"
  "version.cpp.in"
)

file(GLOB PROJECT_INC_FILES
  "Attribute.h"
  "AttributeCommon.h"
  "AttributeDecoder.h"
  "AttributeEncoder.h"
  "BitReader.h"
  "BitWriter.h"
  "DualLutCoder.h"
  "FixedPoint.h"
  "OctreeNeighMap.h"
  "PCCKdTree.h"
  "PCCMath.h"
  "PCCMisc.h"
  "PCCPointSet.h"
  "PCCTMC3Common.h"
  "PCCTMC3Decoder.h"
  "PCCTMC3Encoder.h"
  "RAHT.h"
  "TMC3.h"
  "attribute_raw.h"
  "colourspace.h"
  "coordinate_conversion.h"
  "constants.h"
  "entropy.h"
  "entropychunk.h"
  "entropydirac.h"
  "entropyo3dgc.h"
  "entropyutils.h"
  "frame.h"
  "framectr.h"
  "geometry.h"
  "geometry_intra_pred.h"
  "geometry_octree.h"
  "geometry_params.h"
  "geometry_predictive.h"
  "geometry_trisoup.h"
  "hls.h"
  "io_hls.h"
  "io_tlv.h"
  "lod_scan_simd.h"
  "motionWip.h"
  "osspecific.h"
  "parallel.h"
  "partitioning.h"
  "pcc_chrono.h"
  "ply.h"
  "pointset_processing.h"
  "quantization.h"
  "ringbuf.h"
  "simd.h"
  "tables.h"
  "version.h"
  "../dependencies/nanoflann/*.hpp"
  "../dependencies/nanoflann/*.h"
  "../dependencies/program-options-lite/*.h"
  "../dependencies/schroedinger/schroarith.h"
)

file(GLOB PROJECT_CPP_FILES
  "AttributeCommon.cpp"
  "AttributeDecoder.cpp"
  "AttributeEncoder.cpp"
  "DualLutCoder.cpp"
  "FixedPoint.cpp"
  "OctreeNeighMap.cpp"
  "RAHT.cpp"
  "TMC3.cpp"
  "attribute_raw_decoder.cpp"
  "attribute_raw_encoder.cpp"
  "coordinate_conversion.cpp"
  "decoder.cpp"
  "encoder.cpp"
  "entropydirac.cpp"
  "frame.cpp"
  "geometry_intra_pred.cpp"
  "geometry_octree.cpp"
  "geometry_octree_decoder.cpp"
  "geometry_octree_encoder.cpp"
  "geometry_predictive_decoder.cpp"
  "geometry_predictive_encoder.cpp"
  "geometry_trisoup_decoder.cpp"
  "geometry_trisoup_encoder.cpp"
  "io_hls.cpp"
  "io_tlv.cpp"
  "misc.cpp"
  "motionWip.cpp"
  "osspecific.cpp"
  "parallel.cpp"
  "partitioning.cpp"
  "pcc_chrono.cpp"
  "ply.cpp"
  "pointset_processing.cpp"
  "quantization.cpp"
  "tables.cpp"
  "../dependencies/program-options-lite/*.cpp"
  "../dependencies/schroedinger/schroarith.c"
)

source_group (inc FILES ${PROJECT_INC_FILES})
source_group (input FILES ${PROJECT_IN_FILES})
source_group (cpp FILES ${PROJECT_CPP_FILES})

include_directories(
  "${PROJECT_BINARY_DIR}/tmc3"
  "${PROJECT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/nanoflann"
  "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/program-options-lite"
)

add_executable (tmc3
  ${PROJECT_CPP_FILES}
  ${PROJECT_INC_FILES}
  ${PROJECT_IN_FILES}
  ${VERSION_FILE}
)
add_dependencies(tmc3 genversion)

find_package(Threads REQUIRED)
target_link_libraries(tmc3 Threads::Threads)

add_executable (ply-merge EXCLUDE_FROM_ALL
  "../tools/ply-merge.cpp"
  "misc.cpp"
  "ply.cpp"
  "../dependencies/program-options-lite/program_options_lite.cpp"
  ${VERSION_FILE}
)
add_dependencies(ply-merge genversion)

install (TARGETS tmc3 DESTINATION bin)
