#!/usr/bin/env bash
set -euo pipefail

missing=0
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${workspace_dir}/scripts/setup.bash" --underlays-only
for command in ros2 colcon gz; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "missing required command: ${command}" >&2
    missing=1
  fi
done

if [ -z "${PX4_DIR:-}" ]; then
  echo "PX4_DIR is not set; export it before running the simulation" >&2
  missing=1
elif [ ! -x "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]; then
  echo "PX4 SITL binary not found under PX4_DIR=${PX4_DIR}" >&2
  missing=1
fi

if ! command -v MicroXRCEAgent >/dev/null 2>&1 \
  && ! command -v micro-xrce-dds-agent >/dev/null 2>&1; then
  echo "Micro XRCE-DDS Agent is not available in PATH" >&2
  missing=1
fi

if [ "${missing}" -ne 0 ]; then
  exit 1
fi
"${workspace_dir}/scripts/check_px4_compatibility.sh" "${PX4_DIR}" --all
echo "ws_VIO prerequisites detected"
