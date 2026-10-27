#!/usr/bin/env bash
# Read back startup parameters. No parameter writes or estimator restarts.
set -euo pipefail
if [[ $# != 3 ]]; then
  echo 'usage: configure_px4_sitl.sh PX4_DIR VEHICLE_ID PROFILE' >&2
  exit 2
fi
param_bin="$1/build/px4_sitl_default/bin/px4-param"
vehicle_id="$2"
profile="$3"
[[ -f ${profile} ]] || { echo "Missing parameter profile: ${profile}" >&2; exit 1; }

# rcS applies PX4_PARAM_* after selecting the airframe. Poll the complete
# profile: SYS_AUTOSTART alone can be visible before that point.
for _attempt in $(seq 1 100); do
  matches=true
  while read -r parameter value rest; do
    [[ -z ${parameter} || ${parameter} == \#* ]] && continue
    [[ -z ${rest} ]] || { echo "Invalid profile line: ${parameter}" >&2; exit 1; }
    if ! "${param_bin}" --instance "${vehicle_id}" compare "${parameter}" "${value}" >/dev/null 2>&1; then
      matches=false
      break
    fi
  done < "${profile}"
  if [[ ${matches} == true ]]; then
    echo 'PX4 GNSS + VIO startup parameters verified; EKF2 owns fusion and recovery'
    exit 0
  fi
  sleep 0.2
done
echo "PX4 startup parameter verification failed: ${parameter}=${value}" >&2
"${param_bin}" --instance "${vehicle_id}" show "${parameter}" || true
exit 1
