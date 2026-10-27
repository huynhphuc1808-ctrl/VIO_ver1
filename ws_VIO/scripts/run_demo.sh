#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${workspace_dir}/scripts/setup.bash" all
exec ros2 launch uav_vio_bringup x500_d435i.launch.py "$@"
