# VIO (ver1)

ROS 2 Jazzy workspace cho D435i visual-inertial odometry, chuyển đổi External
Vision sang PX4 và mission JSON dùng các controller/action có sẵn của PX4.

## Cấu trúc

| Thư mục | Nội dung |
| --- | --- |
| [ws_VIO](ws_VIO/README.md) | Bringup, mô hình/cảm biến Gazebo, adapter VIO → PX4 và mission JSON |
| `ws_px4` | `px4_msgs` và PX4 ROS 2 interface library ở commit đã pin |
| [ws_vio_references/VIO_ver1](ws_vio_references/VIO_ver1/MODULES.md) | Thuật toán với package `vio_core`, `vio_init`, `vio_msckf` và calibration baseline |
| `ws_vio_references/px4_ros_com`, `translation_node` | Mã tham chiếu; không thuộc overlay của workflow chính |

VIO_ver1 được đổi tên và tích hợp từ OpenVINS, giữ giấy phép GPL-3.0 và thông
báo bản quyền của tác giả gốc. Phần đổi tên không thay đổi phép tính thuật toán.
Xem [nguồn gốc và giấy phép](THIRD_PARTY_NOTICES.md).

## Lấy mã và build

Các dependency PX4 là Git submodule để giữ chính xác commit và nguồn gốc.
Clone kèm submodule; bản tải ZIP thông thường không chứa source của submodule.

```bash
git clone --recurse-submodules https://github.com/huynhphuc1808-ctrl/VIO_ver1.git
cd VIO_ver1
# Nếu đã clone mà chưa lấy submodule:
git submodule update --init --recursive

export PX4_DIR=/path/to/PX4-Autopilot
cd ws_VIO
./scripts/build_px4.sh
./scripts/build_vio.sh
./scripts/bootstrap.sh
./scripts/build.sh --executor sequential
source scripts/setup.bash
```

Môi trường: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic; cần Eigen, OpenCV,
Boost, Ceres và các ROS dependency được khai báo trong `package.xml`.
PX4-Autopilot, Micro XRCE-DDS Agent và QGroundControl dùng bản cài riêng.
Các script không tự nâng cấp các thành phần này.

| Thành phần | Commit |
| --- | --- |
| PX4-Autopilot, ROS API 1 | `50dabd8faef300a4c408070be60f3ea5089bef08` |
| px4_msgs | `51e66788d6d538c7eabdb6a3becb6eee78167cb2` |
| px4-ros2-interface-lib | `cd22f009c666063db26e82b607595ac8313f7ee7` |
| Nền thuật toán VIO_ver1 | `69488123ed9362dd44b6f28e7f4680abbff1442b` + sửa đổi cục bộ |

Overlay: Jazzy → `ws_px4/install` → `ws_vio_references/install` → `ws_VIO/install`.
`PX4_WS` và `VIO_ALGORITHM_WS` mặc định trỏ đến hai workspace cùng cấp với
`ws_VIO`; có thể đặt lại bằng biến môi trường trước khi build/source.

## Chạy và kiểm tra

```bash
# Từ ws_VIO, sau khi source scripts/setup.bash:
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
./scripts/run_demo.sh world:=lawn gui:=false rviz:=false
```

Bringup mặc định không tự arm. PX4 sở hữu EKF2, flight control, Navigator và
failsafe. Stereo/IMU thật từ cảm biến hoặc Gazebo cấp dữ liệu cho VIO; ground
truth chỉ phục vụ đánh giá. Chỉ một backend được quyền gửi lệnh flight.

Xem [hướng dẫn ứng dụng](ws_VIO/README.md),
[build/chạy/test](ws_VIO/docs/development/BUILD_AND_RUN.md),
[kiến trúc mission](ws_VIO/docs/architecture/MISSION_ARCHITECTURE.md) và
[kiểm chứng phiên bản này](VALIDATION.md).

Repository chỉ chứa source, cấu hình, asset cần thiết, test và tài liệu project.
Build/install/log, dữ liệu runtime và tài liệu trợ lý cục bộ không được đưa vào.
