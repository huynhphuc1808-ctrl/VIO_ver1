#!/usr/bin/env bash
# Build only the estimator packages; archived reference packages are not overlays.
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
algorithm_ws="${VIO_ALGORITHM_WS:-$(dirname "${workspace_dir}")/ws_vio_references}"
for package in vio_core vio_init vio_msckf; do
  test -f "${algorithm_ws}/VIO_ver1/${package}/package.xml" || {
    echo "Missing estimator package: ${algorithm_ws}/VIO_ver1/${package}" >&2
    exit 1
  }
done
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${algorithm_ws}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j2}"
exec colcon build --base-paths VIO_ver1 --symlink-install --executor sequential \
  --packages-select vio_core vio_init vio_msckf "$@"
