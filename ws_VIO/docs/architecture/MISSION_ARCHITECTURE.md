# Mission JSON local trên PX4 API 1

Trạng thái: circle và mission JSON dùng chung executable `vio_mission` từ cutover
2026-10-21, sau các gate SITL Phase0–4. `vio_circle.launch.py` là preset include;
không còn executor/binary circle riêng. Regression sau cutover và phạm vi nghiệm
thu nằm trong [acceptance manifest](../validation/astra_mission_acceptance.json).
Workflow GNSS+VIO vẫn dùng onboard PX4 Navigator/Mission.

## Đường dữ liệu và quyền điều khiển

D435i stereo + IMU → VIO_ver1 → adapter `/vio/odometry` → External Vision
`VehicleOdometry` → PX4 EKF2 → local NED feedback. Không có ground truth hoặc
GNSS trong input VIO_ver1. Chỉ PX4 điều khiển vị trí, attitude, motor và failsafe.

`vio_mission` dùng upstream `MissionExecutor`, `Mission`, `ActionInterface` và
`MissionFileMonitor` của interface-library commit `cd22f009`, API 1. PX4 và
px4_msgs tương ứng là `50dabd8` và `51e6678`. Không thay scheduler upstream.

| Thành phần project | Trách nhiệm |
|---|---|
| `mission_definition.cpp` | Kiểm schema/bounds JSON trước khi tạo endpoint; revision dùng upstream checksum |
| `mission_runtime.cpp` | Registration, bootstrap, ownership, status, lifecycle và cập nhật file |
| `mission_context.cpp` | Telemetry freshness, origin/epoch và reusable LocalGoto mode |
| `mission_actions.cpp` | Local Takeoff/Goto/Hold/Orbit và failure handler; Land chuẩn upstream |
| `mission_guards.hpp` | Pure guards cho readiness, epoch, deadline và callback invocation |
| `mission_file_observer.hpp` | Báo lỗi file bị upstream parser từ chối; không áp dụng mission |

Runtime đăng ký LocalGoto, owned mission mode/executor và một health/arming
check độc lập. `auto_start=false` đợi người điều khiển chọn **VIO Mission** khi ready.
ID lấy từ registration/status, không gắn cứng 23 hoặc 1001.
Khi auto_start được yêu cầu, normal command channel chọn owned mode trước;
sau khi raw `executor_in_charge` xác nhận cùng owner, executor mới gửi arm.
Upstream activation chỉ xảy ra khi armed. Mỗi episode có đúng một backend.

## Action và hệ tọa độ

JSON version 1 có `mission.items`, bắt đầu đúng một `localTakeoff`, kết thúc
đúng một `land`. Các action hỗ trợ: `localTakeoff`, `localGoto`, `localHold`,
`orbit`, `land`. Unknown field/action, NaN, thiếu trường hoặc ngoài bounds bị
từ chối. Hai Orbit liên tiếp phải có localHold/localGoto handoff để native
Orbit được activate lại. Xem validator và các fixture để biết schema chính xác.

Native Takeoff dùng altitude/heading NaN. Vì completion upstream cần global
altitude, local observer bàn giao sau nominal 0.75 m climb (tolerance 0.25 m),
rồi LocalGoto đạt độ cao yêu cầu (Takeoff tối thiểu 2 m). LocalGoto gửi high-level
Goto cho PX4 smoothing/controller; completion cần position error <0.5 m và
speed <0.3 m/s đủ dwell. Orbit gọi native PX4 Orbit và đếm vòng từ local EKF.
Land dùng action chuẩn và chỉ thành công sau fresh landed + disarmed.

`mission_ned` là local NED tịnh tiến từ frozen origin, z âm ở phía trên mặt đất.
Trước arm chỉ có provisional ground reference. Trong Takeoff bounded, chỉ chấp
nhận reset-counter tăng liền kề; delta_z cập nhật ground reference. Frame khóa
sau aligned hover, heading-good và ba ROS seconds không reset. Reset sau freeze
làm mission mất hiệu lực; không bù một frozen frame. Logic và giới hạn thực tế
nằm trong `mission_context.cpp` và `mission_guards.hpp` của package mission.

## Freshness, lỗi và lifecycle

VIO/local receipt freshness dùng steady wall clock (0.5 s); VehicleStatus
freshness là 2 s. Progress deadline dùng ROS time cùng wall watchdog hữu hạn.
Mất owner, stale/invalid state hoặc epoch thay đổi hủy invocation; callback cũ
không được tiếp tục action mới. Failure handler chỉ yêu cầu native Land khi
vẫn có quyền; PX4 tiếp tục chịu trách nhiệm failsafe. Không tự resume/rearm.

File chỉ được thay khi inactive, landed, disarmed, telemetry fresh và bootstrap
chưa bắt đầu. Active edit bị từ chối, không xếp hàng áp dụng sau flight.
`MissionFileMonitor` là nguồn duy nhất áp dụng dữ liệu; observer bổ sung chỉ
đưa malformed/strict-validation errors lên `update_error`. File phải được ghi
lại ở trạng thái hợp lệ để tạo episode mới sau terminal.

`/uav_vio/mission/status` gồm revision, phase/action/index, registration IDs,
owner, ready/armed/landed, frame/origin, success/terminal, error/update_error.
Status và EV fusion không chứng minh VIO chính xác: phải đo stationary drift,
xác nhận native aid/fusion và kiểm ULog trong nghiệm thu thực tế.
