# Build, chạy và kiểm chứng

Từ root `ws_VIO`, dùng PX4 đã cài đúng pin; không cài lại dependency.

```bash
export PX4_DIR=/path/to/PX4-Autopilot
./scripts/check_px4_compatibility.sh "$PX4_DIR" --all
./scripts/build_vio.sh
./scripts/build.sh --executor sequential
source scripts/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
ctest --test-dir build/uav_vio_mission --output-on-failure
colcon test --base-paths src --packages-select uav_vio_mission uav_vio_px4_bridge --executor sequential
```

Overlay: Jazzy → ws_px4 → ws_vio_references → ws_VIO. Không build/sửa upstream để
né mismatch. Build/bootstrap kiểm full triplet và library messages bằng `--all`.
`--application` chỉ là bridge-only VehicleOdometry check, không chứng nhận mission.

Kiểm JSON offline, không tạo ROS flight endpoint:

```bash
ros2 run uav_vio_mission vio_mission --validate-mission src/uav_vio_mission/config/square.json
```

Chạy runtime JSON mặc định (không tự bắt đầu chuyến bay):

```bash
ros2 launch uav_vio_bringup vio_mission.launch.py \
  px4_dir:="$PX4_DIR" gui:=false rviz:=false \
  mission_file:="$PWD/src/uav_vio_mission/config/local_takeoff_land.json"
```

VIO-only dùng `vio_only.params`; GNSS+VIO mặc định dùng bringup và PX4 Navigator.
`vio_circle.launch.py` là preset của cùng runtime; chỉ mở một launch mission. Chỉ chọn mission mode sau khi
VIO đứng yên ổn định, native EV pos/vel/hgt đang fuse, local position/velocity
valid và graph có đúng backend đã chọn. Không dùng `VALID_ODOMETRY` làm bằng
chứng accuracy. Đọc [kiến trúc](../architecture/MISSION_ARCHITECTURE.md).

Thu diagnosis khi không có mission backend (terminal thứ hai, cùng overlay/domain):

```bash
python3 scripts/diagnose_vio_stationary.py --output /tmp/vio-stationary-new --duration 90
```

Tool chỉ subscribe; nếu phát hiện armed hoặc flight-command publisher sẽ fail.
Nó giữ stereo PNG, raw IMU/odometry, camera-updated pose, local state và fusion.

Một episode nghiệm thu bay SITL được yêu cầu rõ bằng `--fly`:

```bash
python3 scripts/run_mission_sitl.py --fly \
  --mission src/uav_vio_mission/config/local_takeoff_land.json \
  --output .artifacts/minimal-new --timeout 360
```

Tool khởi động fresh headless stack, chờ stationary/native-fusion gate, xác nhận
không có backend trước đó rồi chạy một runtime. Collector, graph, events, result
và ULog lưu cùng episode; tool chỉ dừng process group do chính nó tạo. Directory
output phải mới. Runtime success chưa thay thế phân tích ULog/fault matrix và
review; không dùng tool như quy trình hardware flight.

Analyzer circle nhận ID thực từ status/collector:

```bash
python3 scripts/analyze_vio_circle_log.py /path/to/flight.ulg \
  --executor-id 1 --local-mode-id 24 --groundtruth --output /tmp/circle-analysis.json
```

Hai ID trong ví dụ phải thay bằng registration của chính episode. Fixture
CCW/multilap cần truyền direction/laps tương ứng. Các kết quả cũ không chứng nhận
binary/config mới; phạm vi bằng chứng lịch sử nằm trong acceptance manifest.

Với hai Orbit, chạy analyzer hai lần, `--orbit-index 0` rồi `--orbit-index 1`
với direction/laps tương ứng; không dùng bằng chứng vòng đầu cho vòng thứ hai.
`analyze_mission_sitl.py` kiểm nominal episode và `analyze_mission_fault.py`
kiểm cancellation/recovery cho fault episode. Manual intervention làm ca lỗi
thất bại dù cuối cùng drone đã landed/disarmed.


`--groundtruth` chỉ đọc dữ liệu vật lý trong ULog sau flight để đối chiếu radius,
speed/direction/laps/altitude; tuyệt đối không đưa dữ liệu đó vào flight pipeline.
Không có collision avoidance trong runtime: người viết mission phải chừa khoảng
trống cho cả quỹ đạo và kích thước rotor. Hybrid fixture dùng Gotos (1,0),(1,1)
để vòng radius2m tránh tường ở x=5m của scene.

Các ca lifecycle được yêu cầu riêng trong SITL, ví dụ:

```bash
python3 scripts/run_mission_sitl.py --fly \
  --mission /path/to/takeoff-goto-hold12-land.json \
  --output .artifacts/lifecycle-new --fault active-edit --screenshot
python3 scripts/analyze_mission_sitl.py \
  --run .artifacts/lifecycle-new --output .artifacts/lifecycle-new/analysis.json
```

Các lựa chọn fault khác: `vio-loss`, `stale-local`, `takeover`, `reselection`,
`process-death`, `progress-timeout`, `frame-reset`. `--fault-action` chọn bước;
frame-reset chỉ đổi reset counter trên alias telemetry vào mission, không sửa
state PX4 hoặc EV. Hold deadline test cần fixture riêng có target không thể đạt
trong deadline; runner hạ climb limit cho đúng ca đó và khôi phục ở ca bình thường.
Screenshot yêu cầu GUI hoạt động và Hold ít nhất8s; PNG được ghép bằng hash,
clock và telemetry/ULog cùng episode, không phải ảnh dựng.

Regression GNSS+VIO tạo fresh stack và dùng một test driver MAVLink cho native
Navigator (không chạy ROS mission backend):

```bash
python3 scripts/run_gnss_vio_sitl.py --fly --output .artifacts/gnss-vio-new
python3 scripts/analyze_gnss_vio_log.py .artifacts/gnss-vio-new/flight.ulg \
  --output .artifacts/gnss-vio-new/analysis.json
```

Driver kiểm joint fusion trước arm, mất GNSS vẫn tiến mission, GNSS phục hồi,
rồi mất cả hai nguồn dẫn tới native Descend/landed/disarmed. Fixture square2m
nhiều vòng giữ khoảng cách với feature wall. Đây là regression có fault injection,
không phải hướng dẫn bay hardware hay mission tự vận hành ngoài thực tế.

Rollback bằng commit đã kiểm chứng hoặc bản sao lưu cục bộ, rồi build lại
project. Chỉ khôi phục những file của thay đổi cần lùi; không giải nén đè cả
workspace đang có thay đổi. Không chạy đồng thời backend cũ và mới.
