# x500 D435i simulation

The model composes the existing PX4 `x500` with a workspace-owned D435i
payload. The camera is mounted vertically and looks straight down.

```bash
# From the ws_VIO root; PX4_WS/VIO_ALGORITHM_WS default to sibling workspaces.
./scripts/build.sh
source scripts/setup.bash
export PX4_DIR=/path/to/PX4-Autopilot
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
ros2 launch uav_vio_bringup x500_d435i.launch.py
```

Domain 42 and localhost-only DDS are also the launch defaults. Use the same
environment in every terminal that runs ROS CLI commands. This prevents an
unrelated ROS graph on the default domain from becoming a PX4 command owner;
it does not replace the rule that exactly one mission/control backend may run
inside the selected domain. PX4 Navigator now owns the mission; no ROS command backend runs.

For a headless sensor check:

```bash
ros2 launch uav_vio_bringup x500_d435i.launch.py gui:=false rviz:=false
```

ROS topics:

- `/vio/sensors/d435/image_raw`
- `/vio/sensors/d435/camera_info`
- `/vio/sensors/d435/left/image_raw`
- `/vio/sensors/d435/left/camera_info`
- `/vio/sensors/d435/right/image_raw`
- `/vio/sensors/d435/right/camera_info`
- `/vio/sensors/d435/depth/image_raw`
- `/vio/sensors/d435/depth/camera_info`
- `/vio/sensors/d435/imu`
- `/vio/vio_estimator/left/image_raw`
- `/vio/vio_estimator/right/image_raw`
- `/vio_msckf/odomimu`
- `/vio/odometry`
- `/vio/status`
- `/fmu/in/vehicle_visual_odometry`

VIO_ver1 consumes the D435i stereo pair published directly by
`ros_gz_bridge` with reliable QoS on `/vio/vio_estimator/{left,right}/image_raw`, and
`/vio/sensors/d435/imu`. Color and depth stay available for visualization but
are not estimator inputs. PX4 flight control continues to use only the x500
flight-controller IMU; the D435i IMU is not injected into PX4 sensor topics.

The existing VIO_ver1 workspace remains external and is only sourced as an
installed overlay. It is never copied into this workspace. VIO_ver1 raw IMU
odometry is shifted to the vehicle origin as `/vio/odometry` (`odom` →
`base_link`) before conversion to PX4 arbitrary-heading FRD visual odometry.

See the root README for GNSS + VIO parameters, VIO preflight requirements,
VIO_ver1 mixed-output guard and current validation limitations.
# VIO-only JSON missions

`ros2 launch uav_vio_bringup vio_circle.launch.py px4_dir:=$PX4_DIR` is the
circle preset of `vio_mission.launch.py`; both start the single `vio_mission`
executable. There is no separate CircleExecutor/binary. Start one mission launch.

Wait for genuine stationary VIO, native EV fusion and fresh valid local state,
then select **VIO Mission** in PX4/QGroundControl. Default `auto_start=false`
waits for this operator selection; `true` selects the owned mode once ready.
The runtime confirms the same executor owns the mode before arming. Native
Takeoff, Orbit, Land and PX4 controllers remain in charge of actual flight.

With no `mission_file`, circle arguments generate a JSON definition: `radius_m`
(minimum1m), `altitude_m` (minimum2m), `speed_m_s`, `direction`, `laps`,
`hold_seconds`, `radial_tolerance_m` and `gate_stable_seconds`. Pass a JSON path
to run square/hybrid/other supported actions without rebuilding the binary.

The VIO-only profile disables GNSS; GNSS+VIO default bringup remains a separate
native Navigator workflow. Neither composition sends GNSS/ground truth to
VIO_ver1 or ground truth into control. Mission updates require inactive,
landed, disarmed and fresh state. Active edits are rejected without deferred
application; takeover/reset cancels the episode without automatic resume/rearm.

See the [architecture](../../docs/architecture/MISSION_ARCHITECTURE.md) and
[build/run/acceptance guide](../../docs/development/BUILD_AND_RUN.md) for frame,
clock, failure, command ownership and evaluation contracts. Current SITL evidence
is indexed in the [Astra manifest](../../docs/validation/astra_mission_acceptance.json).
