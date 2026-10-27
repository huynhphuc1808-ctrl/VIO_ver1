# GNSS + VIO cho PX4 Mission và mission JSON VIO-only

`ws_VIO` cung cấp VIO_ver1 adapter, frame/covariance/timestamp conversion và
VIO diagnostics cho **PX4 Navigator/Mission có sẵn**. EKF2 fuse GNSS và VIO liên
tục trong cấu hình mặc định. Một mission SITL riêng dùng PX4 API 1 external mode
và native PX4 Orbit với profile VIO-only; nó không thay đổi fusion mặc định.

## Version đã kiểm tra

| Thành phần | Checkout thực tế |
|---|---|
| PX4 | `50dabd8faef300a4c408070be60f3ea5089bef08`, v1.17.0-alpha1-935, ROS API 1 |
| px4_msgs | `51e66788d6d538c7eabdb6a3becb6eee78167cb2`, API 1 |
| px4_ros2_interface_lib | `cd22f009c666063db26e82b607595ac8313f7ee7`, API 1, tương thích với PX4 và `px4_msgs` đã pin |
| VIO_ver1 | `69488123ed9362dd44b6f28e7f4680abbff1442b`, có thay đổi sẵn của người dùng |

Bridge dùng message chuẩn `VehicleOdometry` trên External Vision input; hash
message khớp PX4. Interface library được pin về API 1 tương thích. Cần dùng API
mode/setpoint của library khi implement external mode; `LocalPositionMeasurementInterface`
không thay thế adapter VIO hiện tại vì hợp đồng measurement không chuyển tiếp
đầy đủ `quality`/`reset_counter` cần cho VIO.

## Build và chạy

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic. Project packages are
`uav_vio_sim`, `uav_vio_px4_bridge`, `uav_vio_mission`, and `uav_vio_bringup`.

```bash
export PX4_DIR=/path/to/PX4-Autopilot
export PX4_WS=/path/to/ws_px4
export VIO_ALGORITHM_WS=/path/to/ws_vio_references
./scripts/build_px4.sh  # build px4_msgs + interface library nếu underlay chưa có
./scripts/build_vio.sh  # build vio_core, vio_init, vio_msckf từ VIO_ver1
./scripts/bootstrap.sh
./scripts/build.sh --executor sequential
source scripts/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
./scripts/run_demo.sh world:=lawn gui:=false rviz:=false
```

Các overlay dùng thứ tự Jazzy → ws_px4 → ws_vio_references → ws_VIO. Chỉ ba
package trong `../ws_vio_references/VIO_ver1` được build cho estimator; các
package tham chiếu còn lại không được nạp vào overlay. Build ứng dụng
kiểm tra source/hash message thực dùng. `./scripts/check_px4_compatibility.sh
"$PX4_DIR"` kiểm tra compatibility của PX4/`px4_msgs`/interface library đã pin.
Không dùng install cũ của mission/probe đã xóa. Các artifact cũ được giữ trong
`.artifacts/before_gnss_vio_build_20260914_131133/`.

Nạp GNSS + VIO mission bằng QGroundControl và dùng PX4 Mission. Bringup không tự arm.
Trước arm phải xác nhận VIO thật liên tục, `cs_gnss_pos`, `cs_gnss_vel`,
`cs_ev_vel`, `cs_ev_hgt` cùng true; local position/velocity và global origin hợp
lệ. **PX4 GNSS + VIO workflow vẫn yêu cầu gate vận hành EV trước arm**. VIO-only
circle đăng ký health/arming check riêng cho external mode.
Các service `/uav_vio/mission/start_flight` và `/land` đã được loại bỏ.

## Fusion và timestamp

Profile [gnss_vio.params](src/uav_vio_sim/config/gnss_vio.params) được PX4 `rcS`
áp dụng trước EKF2 startup. Script cấu hình chỉ read-back bằng `param compare`.
Không đổi parameter hay restart/reset EKF2 khi GNSS mất/phục hồi.

| Parameter | Giá trị | Ý nghĩa |
|---|---:|---|
| `EKF2_GPS_MODE` | 1 | EKF2 hạn chế GNSS reset khi nguồn velocity khác còn fuse |
| `EKF2_GPS_CTRL` | 7 | GNSS position, height, velocity |
| `EKF2_EV_CTRL` | 6 | EV height + velocity 3D; không fuse yaw/XY position |
| `EKF2_HGT_REF`, `EKF2_BARO_CTRL` | 0, 1 | Barometer làm height reference |
| `EKF2_EV_NOISE_MD`, `EKF2_EV_QMIN` | 0, 1 | Dùng covariance thực; chỉ mẫu adapter chấp nhận |
| `COM_POS_FS_EPH`, `EKF2_NOAID_TOUT` | 5, 5000000 | Giữ position-loss checks/failsafe chuẩn |
| `SYS_HAS_GPS`, `SIM_GZ_EN_GPS` | 1, 1 | GNSS từ Gazebo hoạt động |
| `UXRCE_DDS_SYNCT` | 0 | **Chỉ SITL:** PX4 HRT và ROS `/clock` cùng clock Gazebo |

`EV_CTRL=7` không phù hợp origin tùy ý của VIO_ver1 ở commit này: khi GNSS
ngừng fuse, `ev_pos_control.cpp` có nhánh reset về EV position. Dùng velocity
VIO để EKF2 duy trì XY và global position từ origin GNSS đã thiết lập. Mission
vẫn cần global position hợp lệ; local position đơn thuần không đủ.
GNSS phục hồi vẫn qua innovation gate; drift lớn có thể khiến fusion chưa
phục hồi. Không ép reset position để vượt gate.

### Mission JSON VIO-only trên SITL

`ros2 launch uav_vio_bringup vio_circle.launch.py px4_dir:=$PX4_DIR` dùng
cùng `vio_mission` với `vio_mission.launch.py`. Circle là preset JSON; executable
`vio_circle`/CircleExecutor cũ đã được bỏ sau nghiệm thu Phase0–4. Một launch chỉ
khởi động một backend. GNSS+VIO bringup mặc định vẫn dùng PX4 Navigator.

Mặc định `auto_start:=false`: đợi VIO đứng yên ổn định, native EV pos/vel/hgt
fusion và local position/velocity hợp lệ, rồi chọn **VIO Mission** qua PX4/QGC.
Runtime xác nhận quyền executor trước khi arm. `auto_start:=true` chủ động chọn
owned mode sau readiness; chỉ sử dụng khi muốn bắt đầu chuyến bay SITL.

Không truyền `mission_file` thì runtime tạo circle từ `radius_m`, `altitude_m`,
`speed_m_s`, `direction`, `laps`, `hold_seconds`, `radial_tolerance_m` và
`gate_stable_seconds`. Takeoff tối thiểu 2 m, radius tối thiểu 1 m. Truyền file
để chạy Takeoff/Goto/Hold/Orbit/Land với cùng binary:

```bash
ros2 launch uav_vio_bringup vio_mission.launch.py \
  px4_dir:="$PX4_DIR" gui:=false rviz:=false \
  mission_file:="$PWD/src/uav_vio_mission/config/square.json"
```

Upstream MissionExecutor điều phối action; native Takeoff dùng altitude/heading
NaN, local observer handoff sau nominal 0.75 m (tolerance 0.25 m), rồi PX4 Goto
đạt độ cao yêu cầu. Frame `mission_ned` khóa sau startup alignment; reset sau
freeze hủy mission. Native Orbit/Land, EKF2, smoothing và controller thuộc PX4.
Deadline dùng ROS time kèm wall watchdog, freshness dùng steady wall clock.

`vio_only.params` tắt GNSS sensor/fusion, bật EV pos/height/velocity
(`EKF2_EV_CTRL=7`), không fuse EV yaw. Headless không có GCS nên
`NAV_DLL_ACT=0`; native position/estimator-loss checks giữ nguyên. Không cấp
GNSS/ground truth cho VIO_ver1 hoặc dữ liệu ground truth cho flight control.
File chỉ thay được khi inactive, landed, disarmed; active edit bị từ chối và
không được áp dụng ngầm sau flight. Không tự resume/rearm sau giành quyền.
Xem [kiến trúc mission](docs/architecture/MISSION_ARCHITECTURE.md).

Hardware phải dùng clock sensor đã đồng bộ với ROS/system time và XRCE time
sync phù hợp; không chép nguyên profile SITL sang flight controller.
`EKF2_EV_DELAY=0` chỉ là baseline simulator, không phải calibration hardware.

## Contract và giới hạn nguồn VIO

D435i stereo + IMU → VIO_ver1; color/depth chỉ để RViz. Không đưa ground truth
hay GNSS vào VIO_ver1. `/vio_msckf/odomimu` (`global` → `imu`) được đổi lever arm
và covariance sang `/vio/odometry` (`odom` → `base_link`), rồi FRD heading tùy
ý trên `/fmu/in/vehicle_visual_odometry`. Không giả là true-north NED.
Color/depth và CameraInfo tương ứng dùng lazy bridge, bắt đầu phát khi có
subscriber (RViz hoặc công cụ kiểm tra). Stereo/IMU luôn hoạt động cho VIO_ver1.

Adapter kiểm tra `/vio_msckf/poseimu` còn tiến timestamp để không tiếp tục cấp
IMU-only propagation khi camera dừng. Đây chưa chứng minh feature updates
thành công khi ảnh mất texture. Converter từ chối NaN, covariance không hợp lệ,
stale/future/non-monotonic time và latch discontinuity không giải thích được
bằng chuyển động. Không tự tạo reset counter hoặc velocity từ pose.
`/vio/status` là heartbeat chẩn đoán (`VALID_ODOMETRY`, `DEGRADED`, `LOST`),
không phải kết luận EKF2 fusion hoặc giấy phép arm.

Checkout VIO_ver1 hiện có một nhánh fallback trên cùng `odomimu` ghi GLOBAL
velocity dưới frame `imu` và covariance cố định 0.01. Adapter chặn signature
này; chỉ chấp nhận nhánh propagated có covariance thật. Không sửa VIO_ver1 để
né lỗi. Xem [review và nghiệm thu](docs/gnss_vio_review.md).

## Kiểm tra

Runtime JSON đã qua các gate SITL trước cutover; trạng thái nghiệm thu cuối,
phạm vi từng ca và các lần thất bại được ghi tại
[acceptance manifest](docs/validation/astra_mission_acceptance.json).
Lệnh chạy, phân tích ULog, fault injection và rollback nằm trong
[build/chạy/nghiệm thu](docs/development/BUILD_AND_RUN.md).

```bash
./scripts/test_smoke.sh --executor sequential
# ULog flight phải có đủ cả mất GNSS, phục hồi, rồi mất cả GNSS/VIO:
python3 scripts/analyze_gnss_vio_log.py /path/to/flight.ulg --output /tmp/result.json
```

Smoke không arm, không publish VIO giả và fail nếu không chứng minh native
GNSS + EV fusion. ULog analyzer chỉ đọc log; cần `pyulog` và `numpy`.
Kết quả flight trước đợt đổi tên và phạm vi áp dụng nằm trong
[báo cáo SITL](docs/validation/ASTRA_ACCEPTANCE_20261021.md) và acceptance manifest.
Đây là bằng chứng lịch sử; các hash và namespace cũ không chứng nhận binary
sau đổi tên. Kiểm chứng build/test và graph mới được báo cáo riêng.
