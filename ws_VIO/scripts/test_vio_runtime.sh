#!/usr/bin/env bash
# Bounded, disarmed integration check with real cameras/VIO_ver1 and native EKF2.
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${workspace_dir}/scripts/setup.bash"
: "${PX4_DIR:?Set PX4_DIR to the checked SITL source/build}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/vio_runtime_ros}"
export QT_QPA_PLATFORM=offscreen
# Preserve old PX4 logs: no logger directory rotation during this check.
export PX4_PARAM_SDLOG_DIRS_MAX=0
logs=$(mktemp -d /tmp/vio_runtime.XXXXXX)
launch_pid=
cleanup() {
  if [[ -n ${launch_pid} ]]; then
    kill -TERM "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
  echo "Runtime evidence: ${logs}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
timeout --signal=INT --kill-after=15s 120s \
  ros2 launch uav_vio_bringup x500_d435i.launch.py world:=lawn gui:=false rviz:=false >"${logs}/launch.log" 2>&1 &
launch_pid=$!
ros2 node list --no-daemon --spin-time 5 >"${logs}/nodes.log" 2>&1
ros2 node info /odom_to_px4_visual_odometry --no-daemon --spin-time 5 >"${logs}/node_info.log" 2>&1 \
  || ros2 node info /odom_to_px4_visual_odometry --no-daemon --spin-time 5 >"${logs}/node_info.log" 2>&1
ros2 topic list --no-daemon --spin-time 5 >"${logs}/topics.log" 2>&1
ros2 topic info /fmu/in/vehicle_visual_odometry --verbose --no-daemon --spin-time 5 >"${logs}/qos.log" 2>&1
# Typed echo avoids relying on the CLI daemon's cached discovery.
for item in \
  '/vio/status std_msgs/msg/String status' \
  '/vio/odometry nav_msgs/msg/Odometry odometry' \
  '/fmu/out/estimator_status_flags px4_msgs/msg/EstimatorStatusFlags flags' \
  '/fmu/out/vehicle_local_position_v1 px4_msgs/msg/VehicleLocalPosition local'; do
  read -r topic type name <<<"${item}"
  timeout --signal=INT --kill-after=2s 8s ros2 topic echo "${topic}" "${type}" --once --no-daemon --spin-time 3 --qos-reliability best_effort >"${logs}/${name}.log" 2>&1 || true
done
hz_rc=0
timeout --signal=INT --kill-after=2s 10s ros2 topic hz /vio/status --spin-time 3 >"${logs}/hz.log" 2>&1 || hz_rc=$?
[[ ${hz_rc} == 0 || ${hz_rc} == 124 ]]
rg -q 'average rate:' "${logs}/hz.log"
# A successful launch/timeout is insufficient: require actual native fusion.
ready=false
for _attempt in $(seq 1 20); do
  kill -0 "${launch_pid}"
  "${PX4_DIR}/build/px4_sitl_default/bin/px4-listener" --instance 0 estimator_status_flags -n 1 >"${logs}/native_flags.log" 2>&1 || true
  if rg -q 'cs_gnss_pos: [Tt]rue' "${logs}/native_flags.log" \
    && rg -q 'cs_ev_vel: [Tt]rue' "${logs}/native_flags.log" \
    && rg -q 'cs_ev_hgt: [Tt]rue' "${logs}/native_flags.log"; then
    ready=true
    break
  fi
  sleep 0.5
done
if [[ ${ready} != true ]]; then
  echo 'FAIL: continuous GNSS + genuine VIO fusion not demonstrated.' >&2
  rg 'Rejecting|Rejected|ERROR|Exception' "${logs}/launch.log" | head -n 12 || true
  exit 1
fi
rg -q 'Publisher count: 1' "${logs}/qos.log"
rg -q 'Subscription count: 1' "${logs}/qos.log"
echo 'PASS: live graph, VIO diagnostics and simultaneous GNSS/EV fusion (disarmed).'
