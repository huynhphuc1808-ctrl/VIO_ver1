# Kiểm chứng VIO_ver1 — 2026-10-28

Môi trường kiểm tra: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, với bộ commit
PX4/px4_msgs/interface API 1 được pin trong README. Các bước dưới đây chạy trên
máy phát triển hiện có; chưa phải thử nghiệm cài từ đầu trên máy sạch.

| Kiểm tra | Kết quả |
| --- | --- |
| Build `vio_core`, `vio_init`, `vio_msckf` | PASS, 3 package |
| Build `uav_vio_mission`, `uav_vio_px4_bridge`, `uav_vio_sim`, `uav_vio_bringup` | PASS, 4 package |
| Full PX4/message/interface compatibility, kể cả trong cây source xuất bản | PASS, đúng triplet API 1 |
| Test mission theo `colcon test-result` | 43 tests, 0 errors/failures/skipped |
| Test bridge, gồm frame/covariance/timestamp/adapter và lint package | 33 tests, 0 errors/failures/skipped |
| Test simulation/launch support | 12 tests, 0 errors/failures/skipped |
| Python/shell syntax trong source ứng dụng | PASS |
| Resolve launch arguments sau đổi tên | PASS |
| Headless GNSS + VIO runtime smoke | PASS, luôn disarmed |
| Source export, đường dẫn tài liệu và bộ lọc file xuất bản | PASS |

## Giữ nguyên logic

Đối chiếu 110 file C++/header/giấy phép của thuật toán với bản sao trước đổi
tên: nội dung khớp sau khi áp dụng đúng ánh xạ tên kỹ thuật. Các thông báo bản
quyền chung và điều khoản giấy phép được giữ nguyên; cập nhật thông tin tác giả
cá nhân ở lần đóng gói tiếp theo được ghi bên dưới. Các phép tính, tham số, calibration và điều kiện
thuật toán không được thay đổi. Cả 16 file C++/header của mission giống hệt
bản trước khi sửa, theo byte.

## Runtime sau đổi tên

Chạy `ws_VIO/scripts/test_vio_runtime.sh` với world lawn, headless,
`ROS_DOMAIN_ID=42` và partition Gazebo riêng. Graph có
`/vio_msckf/run_subscribe_msckf` và `/vio_estimator_odometry_adapter`.
Odometry đi từ `/vio_msckf/odomimu` qua `/vio/odometry` tới
`/fmu/in/vehicle_visual_odometry`.

- `/vio/status`: đo được 10 Hz.
- External Vision input: một publisher, một subscriber.
- Native PX4 flags: `cs_gnss_pos=true`, `cs_ev_vel=true`, `cs_ev_hgt=true`.
- ULog của episode chỉ có `VehicleStatus.arming_state=1`, tương ứng disarmed
  trong message definition đã pin.
- Các process do bài kiểm tra khởi động đã được kiểm tra dừng sau smoke.

Adapter tiếp tục từ chối nhánh fallback có covariance giả định như trước;
phép kiểm fusion ở trên dùng odometry propagated được adapter chấp nhận.

## Phạm vi bằng chứng

Đợt này kiểm chứng đổi tên, build, unit/integration tests và luồng dữ liệu
mô phỏng disarmed. Mission bay, camera phần cứng, chất lượng VIO và hiệu năng
chưa được đo lại sau đổi tên. Không có tuyên bố cải thiện thuật toán.

Các báo cáo SITL ngày 2026-10-21 trong `ws_VIO/docs/validation` là bằng chứng
lịch sử của binary trước đổi tên; hash và namespace ghi trong đó được giữ theo
episode gốc. ULog, ảnh và log runtime lớn được giữ cục bộ, không đóng gói trong
repository mã nguồn.

Build estimator có cảnh báo từ header Eigen; bridge lint có cảnh báo Python
về fork trong process nhiều thread. Các lệnh hoàn tất thành công.

## Bổ sung đầy đủ source dependency — 2026-10-28

Ba dependency trước đây là submodule đã được chuyển thành thư mục source thông
thường: PX4 ROS 2 interface library (257 file), `px4_msgs` (251 file) và
`px4_ros_com` (27 file). Tổng cộng 535 file khớp nội dung/quyền thực thi từ
đúng commit đã pin; chỉ hai file cấu hình editor upstream được loại khỏi bản sao.
Không thay đổi logic của các thư viện.

`DEPENDENCIES.lock.json` lưu SHA-256 và mode của từng file. Bộ kiểm tra source
mới hoạt động với cây source độc lập không có `.git`, đồng thời vẫn hỗ trợ
dependency checkout riêng. Tám test kiểm tra source đúng, source bị sửa/thiếu,
file bổ sung, quyền thực thi, revision sai, Git của monorepo và cache sinh ra
đều đạt. Full PX4/message/interface compatibility tiếp tục PASS.

Danh sách C/C++ có 341 file: 16 trong `ws_VIO`, 174 trong `ws_px4` và 151 trong
`ws_vio_references`; xem `CPP_SOURCE_INDEX.md`. Thông tin tác giả cá nhân được
điều chỉnh theo sự cho phép riêng mà người duy trì bản tích hợp xác nhận đã
nhận. Phần thay đổi C/C++ chỉ xóa đúng các dòng thông báo cá nhân được nêu;
code còn lại giống hệt phiên bản trước. Thông báo của các tác giả khác,
GPL-3.0 chung, MIT của matplotlib-cpp và BSD của PX4/Auterion vẫn được giữ.

Đợt bổ sung này kiểm chứng tính đầy đủ của source và build checker; không có
thay đổi thuật toán để tuyên bố cải thiện chất lượng hoặc hiệu năng, và không
chạy thêm flight/hardware acceptance.
