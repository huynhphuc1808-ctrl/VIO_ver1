#!/usr/bin/env bash
set -eo pipefail
# The native entry point sources Jazzy -> ws_px4 -> ws_vio_references, and selects
# project packages only; the unused API-2 interface library is not built.
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j${CMAKE_BUILD_PARALLEL_LEVEL} -l${CMAKE_BUILD_PARALLEL_LEVEL}}"
exec /ws/scripts/build.sh "$@" --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
