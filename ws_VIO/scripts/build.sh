#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} == --runtime-only ]]; then
  echo '--runtime-only is deprecated: all application packages are built.' >&2
  shift
fi
source "${workspace_dir}/scripts/setup.bash" --underlays-only
source "${workspace_dir}/scripts/project_packages.bash"
"${workspace_dir}/scripts/check_px4_compatibility.sh" "${PX4_DIR:-}" --all
cd "${workspace_dir}"
exec colcon build --base-paths src --symlink-install \
  --packages-select "${vio_project_packages[@]}" "$@"
