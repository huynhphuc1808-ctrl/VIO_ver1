#!/usr/bin/env bash
# Environment order is shared with native builds, including interactive shells.
vio_container_environment() {
  local mode=--underlays-only
  if [[ -f /ws/install/local_setup.bash ]]; then mode=all; fi
  source /ws/scripts/setup.bash "${mode}"
}
if [[ ${BASH_SOURCE[0]} != "$0" ]]; then
  vio_container_environment
  return $?
fi
set -e
vio_container_environment
if [[ $# == 0 ]]; then set -- bash; fi
exec "$@"
