#!/usr/bin/env bash
# Source this file from a fresh shell. local_setup preserves the explicit order.
vio_source_environment() {
  local mode="${1:-all}" restore_nounset=false result=0 prefix
  local app_root
  app_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  export VIO_WS="${VIO_WS:-${app_root}}"
  export PX4_WS="${PX4_WS:-$(dirname "${VIO_WS}")/ws_px4}"
  export VIO_ALGORITHM_WS="${VIO_ALGORITHM_WS:-$(dirname "${VIO_WS}")/ws_vio_references}"
  if [[ ${mode} != all && ${mode} != --underlays-only ]]; then
    echo 'usage: source scripts/setup.bash [--underlays-only]' >&2
    return 2
  fi
  local prefixes=(/opt/ros/jazzy "${PX4_WS}/install" "${VIO_ALGORITHM_WS}/install")
  if [[ ${mode} == all ]]; then prefixes+=("${VIO_WS}/install"); fi
  for prefix in "${prefixes[@]}"; do
    if [[ ! -f ${prefix}/local_setup.bash ]]; then
      echo "Required overlay missing: ${prefix}/local_setup.bash" >&2
      return 1
    fi
  done
  [[ $- == *u* ]] && restore_nounset=true
  set +u
  source /opt/ros/jazzy/setup.bash || result=$?
  for prefix in "${prefixes[@]:1}"; do
    if [[ ${result} == 0 ]]; then
      source "${prefix}/local_setup.bash" || result=$?
    fi
  done
  if [[ ${restore_nounset} == true ]]; then set -u; fi
  return "${result}"
}
vio_source_environment "${1:-all}"
