#!/usr/bin/env bash
# Build dependencies in their own workspace; never change upstream versions.
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_ws="${PX4_WS:-$(dirname "${workspace_dir}")/ws_px4}"
packages=(px4_msgs px4_ros2_cpp)
full_build=true
if [[ ${1:-} == --messages-only ]]; then
  full_build=false
  packages=(px4_msgs)
  shift
  echo 'PARTIAL dependency build: px4_msgs only; interface API mismatch is not resolved.' >&2
fi
for path in src/px4_msgs/package.xml src/px4-ros2-interface-lib/px4_ros2_cpp/package.xml; do
  test -f "${px4_ws}/${path}" || { echo "Missing dependency: ${px4_ws}/${path}" >&2; exit 1; }
done
if [[ ${full_build} == true ]]; then
  "${workspace_dir}/scripts/check_px4_compatibility.sh" "${PX4_DIR:-}"
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${px4_ws}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j2}"
exec colcon build --base-paths src --symlink-install --executor sequential \
  --packages-select "${packages[@]}" "$@"
