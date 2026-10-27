#!/usr/bin/env bash
# No FMU needed; no flight commands or synthetic odometry are published.
set -euo pipefail
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-84}" ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/vio-smoke-ros}"
logs=$(mktemp -d /tmp/vio_bridge_smoke.XXXXXX)
node_pid=
cleanup() {
  if [[ -n ${node_pid} ]]; then
    kill -TERM "${node_pid}" 2>/dev/null || true
    wait "${node_pid}" 2>/dev/null || true
  fi
  echo "Smoke logs: ${logs}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
timeout --signal=INT --kill-after=5s 60s \
  ros2 run uav_vio_px4_bridge odom_to_px4_visual_odometry --ros-args \
  -r __node:=vio_smoke -r /vio/status:=/vio_smoke/status \
  -p input_topic:=/vio_smoke/input -p output_topic:=/vio_smoke/output \
  -p use_sim_time:=false >"${logs}/launch.log" 2>&1 &
node_pid=$!
ros2 node list --no-daemon --spin-time 3 >"${logs}/nodes.log" 2>&1
ros2 node info /vio_smoke --no-daemon --spin-time 5 >"${logs}/node_info.log" 2>&1 \
  || ros2 node info /vio_smoke --no-daemon --spin-time 5 >"${logs}/node_info.log" 2>&1
ros2 topic list --no-daemon --spin-time 3 >"${logs}/topics.log" 2>&1
ros2 topic info /vio_smoke/status --verbose --no-daemon --spin-time 3 >"${logs}/qos.log" 2>&1
timeout --signal=INT --kill-after=2s 8s ros2 topic echo /vio_smoke/status std_msgs/msg/String --once --no-daemon --spin-time 3 >"${logs}/sample.log" 2>&1
rg -q UNINITIALIZED "${logs}/sample.log"
rg -q 'Publisher count: 1' "${logs}/qos.log"
rc=0
timeout --signal=INT --kill-after=2s 8s ros2 topic hz /vio_smoke/status --spin-time 3 >"${logs}/hz.log" 2>&1 || rc=$?
[[ ${rc} == 0 || ${rc} == 124 ]]
rg -q 'average rate:' "${logs}/hz.log"
kill -0 "${node_pid}"
echo 'PASS: bridge graph/QoS, no-input diagnostics heartbeat; no flight certification.'
