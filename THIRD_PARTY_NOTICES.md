# Nguồn gốc và giấy phép

Các thành phần giữ giấy phép riêng. Tên repository và tên package mới không
thay đổi tác giả hoặc điều kiện giấy phép của mã nền.

| Thành phần | Nguồn | Giấy phép |
| --- | --- | --- |
| `ws_vio_references/VIO_ver1` | [OpenVINS](https://github.com/rpng/open_vins/tree/69488123ed9362dd44b6f28e7f4680abbff1442b), Kevin Eckenhoff, Guoquan Huang và OpenVINS Contributors | [GNU GPL v3](ws_vio_references/VIO_ver1/LICENSE) và thông báo trong từng file |
| matplotlib C++ header | Header thư viện đi kèm thuật toán | [Giấy phép riêng](ws_vio_references/VIO_ver1/vio_core/src/plot/LICENSE) |
| `ws_px4/src/px4_msgs` | [PX4/px4_msgs](https://github.com/PX4/px4_msgs/tree/51e66788d6d538c7eabdb6a3becb6eee78167cb2) | [BSD 3-Clause](ws_px4/src/px4_msgs/LICENSE) |
| `ws_px4/src/px4-ros2-interface-lib` | [Auterion/px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib/tree/cd22f009c666063db26e82b607595ac8313f7ee7) | [BSD 3-Clause](ws_px4/src/px4-ros2-interface-lib/LICENSE) |
| `ws_vio_references/px4_ros_com` | [PX4/px4_ros_com](https://github.com/PX4/px4_ros_com), commit trong DEPENDENCIES.lock.json | [BSD 3-Clause](ws_vio_references/px4_ros_com/LICENSE) |
| `ws_vio_references/translation_node` | PX4 translation node, giữ khai báo và source header hiện có | BSD 3-Clause theo `package.xml` |
| Các package `uav_vio_*` | Mã tích hợp ứng dụng trong `ws_VIO` | Apache-2.0 theo `package.xml`; xem [LICENSE](ws_VIO/LICENSE) |

## Thay đổi trong VIO_ver1

Bản tham chiếu trước khi đổi tên chứa các sửa đổi cục bộ của project dựa trên
commit OpenVINS nói trên. Ngày 2026-10-28 đổi thư mục, package/library,
namespace C++, header guard, biến có tiền tố tên cũ, adapter, launch và các
tham chiếu topic/environment tương ứng. Các phép tính, tham số, calibration,
điều kiện update/propagation và logic mission được giữ nguyên.

Các thông báo bản quyền chung, URL tài liệu gốc và các bản LICENSE của thành
phần thứ ba được giữ lại. Một số dòng thông tin cá nhân được điều chỉnh theo
sự cho phép riêng mà người duy trì bản tích hợp xác nhận đã nhận; các tác giả
khác và điều khoản giấy phép chung không bị thay đổi. Các kết quả SITL từ trước
đợt đổi tên là bằng chứng lịch sử, không được coi là kiểm chứng binary mới.
