#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} == --runtime-only ]]; then
  echo '--runtime-only is deprecated: all application packages are tested.' >&2
  shift
fi
cd "${workspace_dir}"
"${workspace_dir}/scripts/build.sh" "$@"
source "${workspace_dir}/scripts/setup.bash" all
source "${workspace_dir}/scripts/project_packages.bash"
colcon test --base-paths src --packages-select "${vio_project_packages[@]}" --executor sequential
for package in "${vio_project_packages[@]}"; do
  colcon test-result --test-result-base "build/${package}" --verbose
done
"${workspace_dir}/scripts/test_vio_runtime.sh"
