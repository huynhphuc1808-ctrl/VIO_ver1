#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${1:-${PX4_DIR:-}}"
scope="${2:---all}"
if [[ ${scope} != --all && ${scope} != --application ]]; then
  echo 'usage: check_px4_compatibility.sh PX4_DIR [--all|--application]' >&2
  exit 2
fi
expected_px4_commit="50dabd8faef300a4c408070be60f3ea5089bef08"
expected_px4_msgs_commit="51e66788d6d538c7eabdb6a3becb6eee78167cb2"
expected_interface_commit="cd22f009c666063db26e82b607595ac8313f7ee7"
expected_px4_ros2_api_version="1"
px4_ws="${PX4_WS:-$(dirname "${workspace_dir}")/ws_px4}"
px4_msgs_dir="${px4_ws}/src/px4_msgs"
interface_dir="${px4_ws}/src/px4-ros2-interface-lib"
compatibility_script="${interface_dir}/scripts/check-message-compatibility.py"

if [ -z "${px4_dir}" ]; then
  echo "PX4_DIR is not set; pass the PX4-Autopilot checkout as argument 1" >&2
  exit 2
fi
if [ ! -d "${px4_dir}/msg" ]; then
  echo "PX4 message directory not found: ${px4_dir}/msg" >&2
  exit 1
fi
if ! actual_px4_commit="$(git -C "${px4_dir}" rev-parse HEAD 2>/dev/null)"; then
  echo "PX4 checkout is not a readable Git worktree: ${px4_dir}" >&2
  exit 1
fi
if [ "${actual_px4_commit}" != "${expected_px4_commit}" ]; then
  echo "PX4 is not pinned to the validated commit" >&2
  echo "  expected: ${expected_px4_commit}" >&2
  echo "  actual:   ${actual_px4_commit}" >&2
  exit 1
fi
if [ ! -d "${px4_msgs_dir}/msg" ]; then
  echo "px4_msgs checkout is missing: ${px4_msgs_dir}/msg" >&2
  exit 1
fi
if [ ! -f "${compatibility_script}" ]; then
  echo "Upstream compatibility checker is missing: ${compatibility_script}" >&2
  exit 1
fi
# Verify an independent Git checkout, or every included source file against the
# pinned manifest. The enclosing monorepo HEAD is not an upstream dependency pin.
source_verifier="${workspace_dir}/scripts/verify_dependency_source.py"
actual_interface_commit="$(python3 "${source_verifier}" "${interface_dir}" "${expected_interface_commit}")"
interface_registration="${interface_dir}/px4_ros2_cpp/src/components/registration.cpp"
actual_interface_api="$(sed -nE \
  's/.*kLatestPX4ROS2ApiVersion[[:space:]]*=[[:space:]]*([0-9]+).*/\1/p' \
  "${interface_registration}")"
actual_px4_msgs_commit="$(python3 "${source_verifier}" "${px4_msgs_dir}" "${expected_px4_msgs_commit}")"
echo "PX4: ${actual_px4_commit}"
echo "px4_msgs: ${actual_px4_msgs_commit}"
echo "px4_ros2_cpp: ${actual_interface_commit}, API ${actual_interface_api:-unknown}"
if [[ ${actual_px4_msgs_commit} != "${expected_px4_msgs_commit}" ]]; then
  echo "px4_msgs must match ${expected_px4_msgs_commit}" >&2
  exit 1
fi

# Narrow bridge-only check retained for callers that do not use mission/control.
# The application build/bootstrap use --all because uav_vio_mission consumes the
# pinned interface library. Reuse upstream's hash implementation in both paths.
if [[ ${scope} == --application ]]; then
  python3 - "${compatibility_script}" "${px4_msgs_dir}" "${px4_dir}" <<'PY'
import importlib.util
import sys
spec = importlib.util.spec_from_file_location('px4_hash', sys.argv[1])
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)
for name in ('VehicleOdometry',):
    hashes = [checker.message_hash(name, root) for root in sys.argv[2:]]
    if hashes[0] != hashes[1]:
        raise SystemExit(f'{name}: incompatible source message hashes {hashes}')
    print(f'{name}: matching upstream hashes {hashes[0]}')
PY
  echo 'External-vision message compatible. Bridge-only check; use --all to validate mission/control.'
  if [[ ${actual_interface_api} != ${expected_px4_ros2_api_version} ]]; then
    echo 'Mission/control requires the pinned API 1 triplet; run --all before using the interface library.' >&2
  fi
  exit 0
fi
if [ "${actual_interface_commit}" != "${expected_interface_commit}" ] \
  || [ "${actual_interface_api}" != "${expected_px4_ros2_api_version}" ]; then
  echo "px4_ros2_cpp checkout does not match the validated PX4/message triplet" >&2
  echo "  expected: ${expected_interface_commit}, API ${expected_px4_ros2_api_version}" >&2
  echo "  actual:   ${actual_interface_commit}, API ${actual_interface_api:-unknown}" >&2
  echo "  source:   ${interface_dir}" >&2
  echo "No upstream source/version was changed. Resolve the triplet explicitly before control cut-over." >&2
  exit 1
fi

actual_px4_msgs_commit="$(python3 "${source_verifier}" "${px4_msgs_dir}" "${expected_px4_msgs_commit}")"
if [ "${actual_px4_msgs_commit}" != "${expected_px4_msgs_commit}" ]; then
  echo "px4_msgs is not pinned to the validated commit" >&2
  echo "  expected: ${expected_px4_msgs_commit}" >&2
  echo "  actual:   ${actual_px4_msgs_commit}" >&2
  exit 1
fi

px4_registration_msg="${px4_dir}/msg/versioned/RegisterExtComponentRequest.msg"
if [ ! -f "${px4_registration_msg}" ]; then
  echo "PX4 registration message is missing: ${px4_registration_msg}" >&2
  exit 1
fi
actual_px4_ros2_api_version="$(
  sed -nE \
    's/^uint16[[:space:]]+LATEST_PX4_ROS2_API_VERSION[[:space:]]*=[[:space:]]*([0-9]+).*/\1/p' \
    "${px4_registration_msg}"
)"
if [ "${actual_px4_ros2_api_version}" != "${expected_px4_ros2_api_version}" ]; then
  echo "PX4 ROS 2 API version is not compatible with upstream px4_ros2_cpp" >&2
  echo "  expected: ${expected_px4_ros2_api_version}" >&2
  echo "  actual:   ${actual_px4_ros2_api_version:-unknown}" >&2
  exit 1
fi

python3 "${compatibility_script}" -v "${px4_msgs_dir}" "${px4_dir}"
echo "PX4/px4_msgs/px4_ros2_cpp compatibility validated"
echo "  PX4:            ${actual_px4_commit}"
echo "  px4_msgs:        ${actual_px4_msgs_commit}"
echo "  px4_ros2_cpp:    ${expected_interface_commit}"
echo "  PX4 ROS 2 API:   ${actual_px4_ros2_api_version}"
