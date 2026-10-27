# Review GNSS + VIO / PX4 Mission — 2026-09-23

## Cập nhật nghiệm thu — 2026-10-21

Runtime/pins hiện tại đã vượt full compatibility với interface API1 `cd22f009`,
px4_msgs `51e6678`, PX4 `50dabd8`. `gnss_vio02` kiểm lại genuine VIO_ver1 sau
mixed-output guard: joint fusion5.112s native trước arm; mất GNSS vẫn tiến
waypoint3→4; phục hồi fusion; mất cả hai nguồn dẫn tới native Descend rồi
landed/disarmed. Motion residual tối đa0.00604m, không reset XY/Z/heading khi
còn aided Mission. Default lawn-world disarmed graph/fusion smoke cũng PASS.
Xem [manifest và phạm vi](validation/astra_mission_acceptance.json) và
[báo cáo Astra](validation/ASTRA_ACCEPTANCE_20261021.md).

Các mục ghi ngày tháng bên dưới giữ bối cảnh review/lần bay cũ; mismatch API1/2
đã được giải quyết bằng triplet hiện tại. GNSS+VIO vẫn dùng gate EV vận hành
trước arm; không tuyên bố firmware bắt buộc EV. VIO-only JSON runtime có native
health/arming extension riêng và không thay workflow onboard Navigator.

## Source và quyết định

Kiểm tra worktree/diff trước khi sửa; giữ nguyên các sửa đổi có sẵn của project
và source dependency. Version cụ thể được ghi ở README và `dependencies.repos`.

- PX4 `src/modules/ekf2/params_gnss.yaml`: có GPS_MODE=1, GPS_CTRL=7.
- `EKF/aid_sources/gnss/gps_control.cpp`: timeout/quality checks tự stop GNSS;
  bắt đầu lại qua innovation gate; `isGnssPosResetAllowed()`/velocity dùng
  nguồn aiding khác để quyết định reset. Không cần ROS source switcher.
- `EKF/aid_sources/external_vision/ev_pos_control.cpp`: thay đổi trạng thái
  GNSS position bias có thể reset XY về raw EV origin. Chọn EV_CTRL=6 tránh
  nhánh đó; height + 3D velocity vẫn từ VIO, EKF2 tích phân XY.
- `ev_height_control.cpp`: baro height reference giữ altitude datum độc lập
  origin VIO. EV offset do EKF2 bias estimator xử lý.
- `EKF/ekf.h`: global XY hợp lệ khi global origin đã được thiết lập và local
  XY còn hợp lệ. `commander/ModeUtil/mode_requirements.cpp`: Mission yêu cầu
  local/global validity; không thể đổi GNSS mission thành local-only bằng ROS.
- `commander/failsafe/failsafe.cpp`: dùng mode requirements/fallback chuẩn.
  Không giữ custom hover/land timer, không defer/disable position failsafe.
- `ROMFS/.../init.d-posix/rcS`: PX4_PARAM_* áp dụng trước startup EKF2.
  `configure_px4_sitl.sh` chỉ compare toàn bộ profile, không stop/start EKF2.
- `uxrce_dds_client/module.yaml`, `GZBridge.cpp`: clock Gazebo của HRT và ROS
  không được bù Agent OS offset lần nữa; SYNCT=0 chỉ trong SITL này.

Tài liệu đối chiếu chính thức:
[PX4 GNSS degraded/denied](https://docs.px4.io/main/en/advanced_config/gnss_degraded_or_denied_flight),
[EKF2](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf).
Quyết định implementation dựa trên source commit thực tế, không áp dụng mù
parameter/API từ tài liệu main.

## Xóa và giữ

Xóa `uav_vio_mission` (Python Offboard mission/failsafe, service/launch/tests),
`uav_vio_interfaces/StartFlight`, probe/status wrapper của
`uav_vio_px4_control`, skeleton `uav_px4_platform`. Không tạo mission navigator
thay thế. Xóa velocity-from-pose, variance fallback giả và reset counter suy ra
từ position jump; bỏ disable-GNSS và restart EKF2 trong configure script.

Giữ adapter VIO_ver1, transform lever arm/body/world, covariance thực,
sensor timestamp, VIO validity diagnostics, read-only transform phục vụ RViz,
simulator/sensor/launch. PX4 Navigator và failsafe là owner của mission.

Tại review 2026-09-23, library API2 `7e082628` không tương thích message API1.
Đây là trạng thái lịch sử: triplet hiện tại đã pin library API1 `cd22f009` và
build/bootstrap đều kiểm full compatibility. Interface measurement vẫn không
chuyển tiếp đầy đủ quality/reset_counter theo contract adapter nên adapter hẹp
được giữ. Không copy library hoặc dùng translation node để che mismatch.

## Findings review đã xử lý

- HIGH mixed VIO_ver1 output: `ROS2Visualizer.cpp::publish_state()` dùng
  global velocity với child frame `imu` và covariance 0.01I. Nhánh
  `visualize_odometry()` dùng velocity IMU/body và covariance từ propagator.
  Cùng publisher không có tag phân biệt. Adapter reject toàn bộ exact signature
  fallback (angular rate zero, full covariance 0.01I), không đổi covariance hay
  suy ra fusion. False rejection khi mẫu thật trùng signature là chủ ý bảo thủ.
- MEDIUM covariance: validate finite/symmetric/PSD **trước** Jacobian và
  symmetrization, tránh biến corruption thành covariance có vẻ hợp lệ.
- MEDIUM position jump: kiểm tra residual so với velocity trước đó và sensor
  dt, không coi dịch chuyển hợp lý trong transport gap là reset estimator.
- MEDIUM headless: dùng cùng `IfCondition` cho `gui=false` và `gui=0`.
- MEDIUM tài liệu/service cũ: README/build/smoke đã chuyển sang onboard Mission.

## Evidence flight lịch sử 2026-09-14

ULog `06_19_50.ulg`, SHA-256 và kiểm tra tự động tại
[gnss_vio_sitl.json](validation/gnss_vio_sitl.json). Log gốc trên máy:
`$PX4_DIR/build/px4_sitl_default/rootfs/0/log/2026-09-14/06_19_50.ulg`.
Phân tích lại bằng `scripts/analyze_gnss_vio_log.py`.

- Joint GNSS/EV fusion 2.972s simulation trước arm tại 18.988s.
- Native Mission sau takeoff; GNSS stop fusion 33.572s, vẫn valid XY/velocity.
  Waypoint tiến 1 → 2 trong outage; GNSS resume 38.556s.
- Trong aided Mission: không XY/Z/heading reset; bước position lớn nhất 0.00923m,
  motion residual lớn nhất 0.00455m (ngưỡng test 0.5m).
- Mất cả GNSS + EV tại 47.612s; native Descend 51.724s;
  hạ cánh rồi disarm 57.564s. Không ROS failsafe/controller.
- Injection bằng native `SIM_GPS_USED=0/10`, rồi dừng adapter của test.
  Không đổi EKF2_GPS_CTRL/EV_CTRL giữa flight. Mission upload qua MAVLink;
  test tạm dùng COM_RC_IN_MODE=4 vì không có RC và cruise 1m/s.

**Giới hạn quan trọng:** đây là log trước guard mixed-output. Nó chứng minh
hành vi PX4 với pipeline cũ, chưa chứng nhận precision/velocity của mọi mẫu
VIO_ver1 hoặc flight với adapter cuối. Không dùng PASS của analyzer để bỏ qua
review HIGH nói trên. Test lại end-to-end sau khi nguồn odometry thật hoạt động.

## Blocker và phạm vi chưa nghiệm thu

1. Triplet API1 đã tương thích. GNSS+VIO vẫn có thể arm bằng GNSS khi VIO chưa
   fuse; gate EV trước arm là quy trình kiểm tra, chưa cưỡng chế firmware.
   Standard HealthAndArmingChecks được dùng riêng cho VIO-only JSON runtime.
2. Nguồn VIO_ver1 dirty có fallback sai contract. Các ca hiện tại đã chứng minh genuine
   propagated output khi đứng yên và bay trong phạm vi đã ghi; không sửa estimator theo
   ràng buộc người dùng. Guard có thể làm mất preflight EV fusion ở ZUPT.
3. poseimu freshness chỉ chứng minh camera-state timestamp tiến, không phải
   successful visual features. Blackout/featureless images cần test riêng;
   stopped-provider failsafe không chứng nhận tất cả kiểu lỗi VIO.
4. GNSS return sau drift lớn có thể không qua innovation gate. Không manual
   reset; chưa có long-outage/large-drift/hardware acceptance.
5. Docker image build và hardware timing/extrinsic calibration chưa nghiệm thu.
