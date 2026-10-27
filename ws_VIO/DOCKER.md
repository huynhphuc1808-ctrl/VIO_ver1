> Build ứng dụng cần mount PX4_DIR bằng compose.external.yaml để kiểm
> source/hash của triplet API 1 đã pin. Build các overlay PX4 và VIO_ver1
> trên máy chủ trước. Docker là tùy chọn; xem README cho workflow native và
> kết quả kiểm chứng mới nhất.

# Chạy workspace VIO bằng Docker

## 1. Docker dùng để làm gì?

Docker đóng gói Ubuntu, ROS 2 và thư viện vào một **image**. **Container** là
một phiên chạy của image đó. Bạn có thể mở nhiều container từ cùng image,
giống như mở nhiều terminal có cùng bộ công cụ đã cài.

Project này dùng Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic qua `ros_gz`.
Docker giúp máy khác có môi trường build giống nhau, tránh cài toàn bộ thư viện
ROS lên máy chủ. Jazzy và Harmonic là cặp phiên bản được Gazebo hỗ trợ.
Xem [tài liệu Gazebo/ROS](https://gazebosim.org/docs/harmonic/ros_installation/).

**Phạm vi hiện tại:** image chứa dependency và bản sao source trong `src/`;
Compose mount workspace để phát triển. PX4, Micro XRCE-DDS Agent, QGroundControl
và overlay VIO_ver1 đang dùng được tái sử dụng từ máy chủ. Dockerfile không tự
tải lại hay nâng cấp các thành phần này. Máy mới vẫn phải chuẩn bị chúng để
chạy demo đầy đủ.

Application build chọn `uav_vio_sim`, `uav_vio_px4_bridge`, `uav_vio_mission`, `uav_vio_bringup`.
`px4_msgs`/interface library ở `ws_px4`; VIO_ver1 ở `ws_vio_references`. Source theo
Jazzy → ws_px4 → ws_vio_references → ws_VIO. Dùng compose.external.yaml và PX4_DIR
cho source/hash checks. `vio-build` chỉ build application, không provision các
dependency overlay; interface library phải đúng commit API 1 được ghi ở README.

## 2. Các file

| File | Chức năng |
|---|---|
| `docker/Dockerfile` | Cài dependency qua APT/rosdep; target `dev` để phát triển; `runtime-built` tạo snapshot application runtime với named dependency contexts. |
| `docker/compose.yaml` | Chạy service `vio`, mount workspace tại `/ws`, giữ build/install/log trong Docker volume riêng. |
| `docker/compose.external.yaml` | Mount PX4 tại đúng đường dẫn máy chủ; overlay đã mount trong compose.yaml; thấy Agent có sẵn qua PID/network của host. |
| `docker/compose.gui.yaml` | Cho Gazebo/RViz dùng X11 hoặc XWayland và cookie xác thực của desktop. |
| `docker/.env.example` | Mẫu cấu hình UID/GID, domain DDS, số job build và đường dẫn từng máy. |
| `docker/entrypoint.sh` | Source Jazzy → ws_px4 → ws_vio_references → workspace đã build; chuyển tín hiệu đến lệnh đang chạy bằng `exec`. |
| `docker/build.sh` | Lệnh `vio-build`: chạy `colcon build --symlink-install`, mặc định 2 compiler job và từng package tuần tự. |
| `docker/smoke.sh` | Lệnh `vio-smoke`: kiểm bridge read-only, graph, QoS, heartbeat UNINITIALIZED khi không có VIO. |
| `.dockerignore` | Loại Git, build/install/log trên host, rosbag, cache và cấu hình cá nhân khỏi build context. |

## 3. Chuẩn bị máy và build image

Các lệnh dưới đây chạy bằng **Bash tại thư mục gốc `ws_VIO` trên máy chủ**,
trừ khi ghi rõ là lệnh trong container. Cấu hình mô phỏng đầy đủ nhắm tới
Docker Engine trên Linux, ưu tiên Ubuntu 24.04. Docker Desktop trên Windows/macOS
không được kiểm chứng với USB, PID host và đồ họa của project này.
Cấu hình `external` giả định daemon Docker thông thường trên host; chưa kiểm
chứng network/PID host khi chạy Docker rootless.

Cài Docker Engine cùng Compose plugin và Buildx theo
[hướng dẫn chính thức cho Ubuntu](https://docs.docker.com/engine/install/ubuntu/).
Nếu đã có Docker thì dùng bản hiện có. Xác nhận cả Client và Server đều hoạt động:

```bash
docker version
docker compose version
docker run --rm hello-world
```

Chuẩn bị source application và hai workspace dependency, rồi cấu hình:

```bash
# Dependency source nằm ngoài repo; không dùng submodule update tại ws_VIO.
export PX4_WS=/path/to/ws_px4
export VIO_ALGORITHM_WS=/path/to/ws_vio_references
cp docker/.env.example docker/.env
export VIO_UID=$(id -u)
export VIO_GID=$(id -g)
docker compose --env-file docker/.env -f docker/compose.yaml config --quiet
docker compose --env-file docker/.env -f docker/compose.yaml build
```

Chỉ chạy `cp` khi chưa có `docker/.env`. Điền UID/GID từ `id -u`/`id -g` vào
file này để các terminal sau dùng cùng giá trị. Không chạy build bằng UID 0.
Image tạo user `vio` có cùng UID/GID với bạn để file trong mount không thành
file của root. `docker/.env` được Git bỏ qua.

`PX4_WS` và `VIO_ALGORITHM_WS` phải là đường dẫn tuyệt đối thật, đã build trên
Ubuntu 24.04/Jazzy và đúng CPU architecture. `dependencies.repos` ghi revision
thực tế của PX4 dependencies; không tự import/đổi version trong Docker build.
Chuẩn bị messages bằng `./scripts/build_px4.sh --messages-only` trên host.

## 4. Vào container, mount và build ROS 2

```bash
docker compose --env-file docker/.env -f docker/compose.yaml run --rm vio
```

Trong container:

```bash
pwd                         # /ws
vio-build
source /ws/scripts/setup.bash
ros2 pkg list | grep uav_vio
```

Hoặc build bằng một lệnh từ host:

```bash
docker compose --env-file docker/.env -f docker/compose.yaml -f docker/compose.external.yaml run --rm -T vio vio-build
docker compose --env-file docker/.env -f docker/compose.yaml run --rm -T \
  -e ROS_DOMAIN_ID=84 vio vio-smoke
```

`vio-build` gọi `/ws/scripts/build.sh` với compiler jobs giới hạn và executor
sequential. Kiểm tra source/hash ứng dụng cần `PX4_DIR` cùng
compose.external.yaml. Build ứng dụng:

```bash
vio-build
source /ws/scripts/setup.bash
```

`vio-smoke` chạy bridge Python trên topic riêng và kiểm heartbeat, không cần
PX4 hay library API 2. Không dùng executable cũ từ install trước restructure.
Dependency mounts là read-only; `vio-build` không build hoặc sửa `ws_px4` hay
VIO_ver1. rosdep chỉ bỏ các key upstream ở bước cài system packages; runtime
setup vẫn bắt buộc có overlays, không tạo implementation thay thế.

Thư mục host được bind mount vào `/ws`: sửa source trên host sẽ thấy ngay trong
container. Ba đường dẫn `/ws/build`, `/ws/install`, `/ws/log` được phủ bằng
volume `ws-vio_build`, `ws-vio_install`, `ws-vio_log`; dữ liệu build native trên
host không bị dùng lại. Container `--rm` bị xóa khi thoát, các volume vẫn còn.
Đây là hành vi của [bind mount](https://docs.docker.com/engine/storage/bind-mounts/)
và [Compose volume](https://docs.docker.com/reference/compose-file/volumes/).

Nếu cần container chạy lâu để mở thêm terminal:

```bash
docker compose --env-file docker/.env -f docker/compose.yaml up -d
docker compose --env-file docker/.env -f docker/compose.yaml exec vio bash
docker compose --env-file docker/.env -f docker/compose.yaml down
```

`exec vio bash` tự source qua `.bashrc`. Với `exec` chạy thẳng một lệnh ROS,
dùng `exec vio vio-entrypoint ros2 ...` để source môi trường trước.

Snapshot **runtime một phần** có thể chứa cả hai workspace dependency qua
named build contexts. Truyền toàn bộ workspace để giữ các symlink-install;
đường dẫn đích phải đúng đường dẫn tuyệt đối host:

```bash
docker build -f docker/Dockerfile --target runtime-built \
  --build-context px4_workspace="$PX4_WS" \
  --build-context px4_source="$PX4_DIR" \
  --build-context vio_estimator_workspace="$VIO_ALGORITHM_WS" \
  --build-arg PX4_WS="$PX4_WS" --build-arg VIO_ALGORITHM_WS="$VIO_ALGORITHM_WS" \
  --build-arg VIO_UID="$(id -u)" --build-arg VIO_GID="$(id -g)" \
  -t ws-vio:runtime .
docker save -o /tmp/ws-vio-runtime.tar ws-vio:runtime
```

`runtime-built` build các package ứng dụng và cần context `px4_source`
để kiểm source/hash. Target `built` cũ đã được thay bằng target này, không có image full
build được chứng nhận. PX4/Agent ngoài vẫn cần cho simulation. Chỉ chuyển
artifact giữa các máy có cùng kiến trúc/ABI. Docker hiện không có executable
trong môi trường kiểm thử restructure nên build image chưa được xác minh.

## 5. Chạy mô phỏng và VIO hiện có

Trên host, đặt đường dẫn **tuyệt đối thực tế**, không dùng `/path/to/...` nguyên văn:

```bash
export PX4_DIR=/path/to/PX4-Autopilot
export PX4_WS=/path/to/ws_px4
export VIO_ALGORITHM_WS=/path/to/ws_vio_references
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
export MICRO_XRCE_PORT=8888
./scripts/check_px4_compatibility.sh "$PX4_DIR"
test -x "$PX4_DIR/build/px4_sitl_default/bin/px4"
test -f "$VIO_ALGORITHM_WS/install/local_setup.bash"
touch "/tmp/uav_vio_xrce_${MICRO_XRCE_PORT}.lock"
```

PX4 cần có binary, `rootfs/gz_env.sh`, Gazebo plugins và models của bản
`px4_sitl_default`. VIO_ver1 cần được build trên Ubuntu 24.04/Jazzy, cùng CPU
architecture, có executable `vio_msckf/run_subscribe_msckf`. Mount **toàn bộ**
workspace VIO_ver1, vì `--symlink-install` trỏ về `build/` và `src/`.
Giữ cùng đường dẫn host/container để symlink, RPATH và `gz_env.sh` hoạt động.
Khi chuyển checkout sang đường dẫn khác, build lại dependency đó tại đường dẫn
mới; chỉ copy thư mục `install/` thường không đủ.

Kiểm tra Agent trên host:

```bash
ps -eo pid,comm,args | grep '[M]icroXRCEAgent'
ss -lunp | grep ':8888'
```

Chỉ khi chưa có Agent và cổng chưa được dùng, mở terminal host riêng chạy
**executable đã cài**:

```bash
MicroXRCEAgent udp4 -p "$MICRO_XRCE_PORT"
```

Nếu Agent đang chạy đúng cổng thì tái sử dụng. Không mở Agent thứ hai. Không
chạy đồng thời một phiên PX4/Gazebo khác dùng cùng instance/cổng. QGroundControl
chạy trên desktop host nếu cần quan sát, không phải điều kiện build.

Mở container tích hợp:

```bash
docker compose --env-file docker/.env \
  -f docker/compose.yaml -f docker/compose.external.yaml run --rm vio
```

Trong container, build như mục 4 nếu chưa build, rồi kiểm tra overlay:

```bash
ros2 pkg prefix vio_msckf
ldd "$(ros2 pkg prefix vio_msckf)/lib/vio_msckf/run_subscribe_msckf" | grep 'not found'
ldd "$PX4_DIR/build/px4_sitl_default/bin/px4" | grep 'not found'
```

Hai lệnh `ldd | grep` không in gì và trả 1 là bình thường: không có thư viện
bị thiếu. Nếu có dòng `not found`, xử lý ABI/dependency trước khi chạy.

Chạy mô phỏng cảm biến trước, dừng bằng Ctrl-C:

```bash
xvfb-run -a ros2 launch uav_vio_sim simulation.launch.py \
  world:=lawn gui:=false agent_mode:=existing micro_xrce_port:=8888
```

Sau đó chạy bringup VIO hiện có:

```bash
xvfb-run -a ros2 launch uav_vio_bringup x500_d435i.launch.py \
  world:=lawn gui:=false rviz:=false agent_mode:=existing micro_xrce_port:=8888
```

Đổi `8888` trong cả hai lệnh nếu dùng cổng khác. `xvfb-run` cung cấp display ảo
cho renderer khi host không có màn hình. Đây là hướng chạy software rendering;
FPS phụ thuộc CPU và tải máy. Để xem GUI, dùng mục 6 và bỏ `xvfb-run`.

Compose dùng network/IPC host để ROS DDS, Gazebo transport và UDP Agent giao
tiếp trên Linux. File `external` thêm PID host để launch nhận ra Agent, đồng
thời chia sẻ đúng file khóa cổng. PX4 mount có quyền ghi vì SITL ghi trạng thái
runtime; VIO_ver1 mount chỉ đọc. Xem
[Docker host networking](https://docs.docker.com/engine/network/drivers/host/).

Trong terminal container thứ hai, dùng cùng các file Compose và biến môi trường:

```bash
ros2 node list --no-daemon --spin-time 5
ros2 node info /d435i_gz_bridge --no-daemon --spin-time 5
ros2 topic list --no-daemon --spin-time 5
ros2 topic info /vio/sensors/d435/imu --verbose --no-daemon --spin-time 5
timeout 10s ros2 topic hz /vio/sensors/d435/imu
timeout 10s ros2 topic echo /vio/sensors/d435/imu --once --qos-reliability best_effort
timeout 10s ros2 topic echo /vio/odometry --once
```

`timeout` trả 124 khi hết thời gian đo là bình thường; vẫn phải kiểm tra đầu ra
có message/rate. Nếu CLI mâu thuẫn, lặp lại với `--no-daemon --spin-time 5`.
Mọi terminal host/container và PX4 phải dùng cùng domain, localhost flag.

Đầu vào VIO vẫn là stereo D435i và `/vio/sensors/d435/imu`; color/depth chỉ
để quan sát. Không nối ground truth/GPS vào estimator hoặc controller. Các lệnh
ở đây không yêu cầu arm/takeoff. Chỉ thử nhiệm vụ bay sau khi VIO TRACKING ổn định
và PX4 local position hợp lệ theo preflight/fail-safe sẵn có của project.

## 6. RViz, GUI và GPU

### Desktop X11 hoặc XWayland

Trên host có `xauth`, tạo cookie riêng, không mở quyền bằng `xhost +`:

```bash
export DOCKER_XAUTHORITY="$(pwd)/docker/session.xauth"
touch "$DOCKER_XAUTHORITY"
chmod 600 "$DOCKER_XAUTHORITY"
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$DOCKER_XAUTHORITY" nmerge -
docker compose --env-file docker/.env \
  -f docker/compose.yaml -f docker/compose.external.yaml \
  -f docker/compose.gui.yaml run --rm vio
```

Trong container:

```bash
ros2 launch uav_vio_bringup x500_d435i.launch.py \
  world:=lawn gui:=true rviz:=true agent_mode:=existing
```

RViz config của project có color và depth. Kiểm tra hai display Image hiển thị,
đúng optical frame. GUI mặc định dùng Mesa software rendering, không cần GPU.

### Intel/AMD

Nếu host có `/dev/dri`, tạo `docker/gpu.local.yaml`:

```yaml
services:
  vio:
    devices:
      - /dev/dri:/dev/dri
    group_add:
      - "${VIDEO_GID}"
      - "${RENDER_GID}"
    environment:
      LIBGL_ALWAYS_SOFTWARE: "0"
```

Trên host:

```bash
export VIDEO_GID=$(stat -c %g /dev/dri/card0)
export RENDER_GID=$(stat -c %g /dev/dri/renderD128)
```

Chọn tên device thực tế nếu máy dùng `card1`/`renderD129`. Thêm
`-f docker/gpu.local.yaml` vào lệnh Compose GUI. Dùng `glxinfo -B` trong
container để xem renderer; `llvmpipe` nghĩa là đang render bằng CPU.

### NVIDIA

Host cần driver NVIDIA hoạt động (`nvidia-smi`) và NVIDIA Container Toolkit
được cấu hình cho Docker. Không cài driver kernel vào image. Tạo
`docker/gpu.local.yaml` với nội dung sau **thay cho** mẫu Intel/AMD:

```yaml
services:
  vio:
    environment:
      LIBGL_ALWAYS_SOFTWARE: "0"
      NVIDIA_DRIVER_CAPABILITIES: graphics,utility,compute,display
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
```

Thêm file này vào lệnh Compose và kiểm tra `nvidia-smi`, `glxinfo -B` bên trong.
Theo [Docker GPU Compose](https://docs.docker.com/compose/how-tos/gpu-support/),
GPU reservation cần driver/toolkit trên host. GPU giúp Gazebo/RViz render;
VIO_ver1 hiện không bắt buộc CUDA.

## 7. RealSense D435i thật, USB

Mô phỏng D435i không cần camera USB. Chỉ bật phần này khi dùng camera thật.
Sửa `INSTALL_REALSENSE=true` trong `docker/.env`, rồi build lại image. Dockerfile
cài ROS wrapper/SDK từ repository ROS; udev và driver kernel thuộc host.

Host phải nhận camera qua USB 3 và có udev rules phù hợp của librealsense.
Xem [hướng dẫn RealSense ROS](https://github.com/realsenseai/realsense-ros).
Nếu chọn backend V4L2, cần cấp thêm đúng `/dev/video*` và các thiết bị HID cần
dùng; mount USB bus chủ yếu phục vụ backend libusb/RSUSB, không bảo đảm mọi
backend thấy camera.

Tạo `docker/usb.local.yaml` cho USB bus:

```yaml
services:
  vio:
    volumes:
      - /dev/bus/usb:/dev/bus/usb
      - /run/udev:/run/udev:ro
    device_cgroup_rules:
      - 'c 189:* rwm'
    group_add:
      - "${USB_GID}"
```

Dùng `lsusb` trên host để tìm Bus/Device của camera, ví dụ `001/004`:

```bash
export USB_GID=$(stat -c %g /dev/bus/usb/001/004)
docker compose --env-file docker/.env \
  -f docker/compose.yaml -f docker/usb.local.yaml run --rm vio
```

Trong container, chỉ kiểm tra stream camera:

```bash
lsusb
ros2 launch realsense2_camera rs_launch.py \
  enable_infra1:=true enable_infra2:=true \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2 \
  enable_color:=true enable_depth:=true enable_sync:=true
```

Các topic camera thật thường nằm dưới `/camera/camera/...`, khác topic simulator.
Cần calibration thật, extrinsics camera–IMU, remap và clock phù hợp trước khi
nối VIO. Không dùng calibration Gazebo cho camera thật; phần tích hợp phần cứng
này chưa được kiểm chứng. Không chạy camera thật với `use_sim_time=true` khi
không có `/clock`.

## 8. Dependency và cấu hình cần kiểm tra khi đổi máy

| Thành phần | Cần giữ/cập nhật |
|---|---|
| Base image | Ubuntu 24.04/Jazzy, ghim digest trong Dockerfile; không đổi sang Humble để né lỗi. |
| Thư viện hệ thống/ROS | `package.xml` là đầu vào rosdep. Thêm dependency thì khai báo ở đó rồi build lại image; công cụ chỉ dùng trong container khai báo tại Dockerfile. |
| PX4 | Commit `50dabd8faef300a4c408070be60f3ea5089bef08`, build SITL/Gazebo tương thích và đường dẫn `PX4_DIR`. |
| `px4_msgs` | Commit `51e66788d6d538c7eabdb6a3becb6eee78167cb2`; giữ đồng bộ với PX4. |
| `px4_ros2_cpp` | Checkout thực tế `7e082628`, API v2, đang không tương thích. Baseline lịch sử là 2.1.1 / `cd22f009` / API v1; cleanup không đổi upstream. |
| `px4_ros_com` | Checkout hiện tại `86e9aeb20e55a4673fa8a9f1c29ea06a6c5ad1af`; lấy qua checkout reference bên ngoài application, không build mặc định. |
| VIO_ver1 ngoài | Đúng source/version và thay đổi đã kiểm thử của người dùng, Ubuntu/Jazzy/CPU ABI phù hợp; cập nhật `VIO_ALGORITHM_WS`. Repo chưa khóa đầy đủ revision/diff của overlay này. |
| Agent/QGroundControl | Tái sử dụng bản tương thích trên host; cổng Agent đồng nhất với PX4, không chạy trùng. |
| Máy chủ | UID/GID, đường dẫn, ROS domain, X11 cookie, GPU driver/toolkit và quyền USB. |
| CPU architecture | Build lại PX4/VIO_ver1/plugin khi đổi x86_64 ↔ ARM64; không tái sử dụng binary sai kiến trúc. |

PX4 dependencies hiện được đọc từ `${PX4_WS}/src`; VIO_ver1 không còn source trong application. Kiểm tra bộ phiên bản trên host bằng
`./scripts/check_px4_compatibility.sh "$PX4_DIR"`. Không chạy checker Git này
trong image snapshot vì `.git` đã bị loại khỏi image.

Base image đã ghim digest, nhưng APT/rosdep vẫn lấy cập nhật trong series
Noble/Jazzy/Harmonic; đây chưa phải bản build tái lập từng byte. Image ghi danh
sách package thực tế tại `/opt/vio/apt-packages.tsv`. Muốn giữ nguyên môi trường
đã nghiệm thu, lưu/chuyển image bằng `docker save/load`, lưu revision/diff của
dependency ngoài và giữ các artifact kiểm thử. Không tự động cập nhật PX4 hay
VIO_ver1 khi rebuild image.

## 9. Lỗi thường gặp

| Lỗi/hiện tượng | Cách xử lý |
|---|---|
| `docker: command not found` | Cài Docker Engine/CLI và Compose plugin theo mục 3 trên host. |
| `Cannot connect to the Docker daemon` | Chạy `docker version`, `docker context show`, `systemctl status docker`; khởi động đúng daemon bằng `sudo systemctl start docker` nếu service đã cài. CLI đơn lẻ không chạy được container. |
| Permission denied socket | Dùng cấu hình quyền nhóm Docker theo tài liệu chính thức, đăng nhập lại. Không `chmod 666 /var/run/docker.sock`. |
| `sudo: a password is required` | Chủ máy thực hiện bước cài/start service từ terminal host; không đưa mật khẩu vào Dockerfile hay `.env`. |
| Compose không hiểu thuộc tính | Dùng `docker compose` plugin v2, nên từ 2.30 trở lên; không dùng `docker-compose` Python v1 cũ. |
| APT/rosdep lỗi DNS, TLS, 429 | Kiểm tra mạng/DNS/proxy Docker, đồng hồ hệ thống; đăng nhập Docker Hub nếu bị giới hạn pull, rồi build lại. Không tắt kiểm tra TLS. |
| `vio_msckf` không tìm thấy | Dùng file `external`, kiểm tra mount/`VIO_ALGORITHM_WS` và binary; `vio_msckf` là tên khác. `rosdep --skip-keys vio_msckf` chỉ bỏ dependency ngoài ở bước cài, không tạo estimator thay thế. |
| PX4 lỗi `libgz-*.so` hoặc plugin | Kiểm tra `ldd`, bản Harmonic, kiến trúc CPU, `gz_env.sh` và đầy đủ build artifacts tại cùng đường dẫn. |
| `Agent ... not found` hoặc UDP 8888 bận | Chạy/tái sử dụng Agent trên host, dùng `compose.external.yaml` và `agent_mode:=existing`; thống nhất cổng. Không sửa sang `auto` để né lỗi. |
| Bind mount không tồn tại | Tạo file lock/X11 cookie, sửa các đường dẫn thật; `create_host_path: false` giúp phát hiện lỗi thay vì tạo nhầm thư mục. |
| `Killed` khi compile, máy hết RAM | Đặt `VIO_BUILD_JOBS=1` trong `.env`; đóng bớt ứng dụng, chạy lại `vio-build`. Không tăng job chỉ theo số CPU. |
| Cache CMake chứa đường dẫn host, quyền volume sai | Sau khi chắc không cần artifact Docker cũ, dùng `docker compose --env-file docker/.env -f docker/compose.yaml down -v`, rồi build ROS lại. Lệnh này xóa ba volume build/install/log Docker, source host vẫn còn. |
| ROS thấy topic nhưng không nhận dữ liệu | Kiểm tra domain/localhost trên mọi terminal và lúc khởi động PX4; kiểm tra QoS bằng `topic info --verbose`; thử CLI `--no-daemon --spin-time 5`. |
| GUI không mở / Qt xcb | Kiểm tra `DISPLAY`, cookie không rỗng, quyền đọc cookie cùng UID; dùng `xvfb-run -a` cho headless. |
| Ảnh đen/chậm hoặc GPU không hiện | Dùng `glxinfo -B`; thử Mesa software trước, sau đó sửa quyền `/dev/dri` hoặc NVIDIA Toolkit. |
| USB hiện trong `lsusb`, driver không mở được | Kiểm tra udev/quyền device/backend SDK; V4L2 có thể cần `/dev/video*`/HID riêng. Không bật `privileged` cho toàn bộ container chỉ để né lỗi. |
| `Waiting for Gazebo world` | Đọc log server; dùng launch của project để xử lý BOM trên bản world tạm, kiểm tra resource/plugin paths. Không sửa world nguồn để chữa lỗi Docker. |

## 10. Bằng chứng kiểm thử của lần đóng gói này

Các kết quả dưới đây chỉ áp dụng cho checkout hiện tại; chưa phải chứng nhận
bay hoặc clone-to-run trên máy sạch. Xem README ở root repository cho các giới hạn
kiểm chứng sau đổi tên. Docker image chưa được build/test
lại trong lần cleanup này; kết quả native không chứng nhận container.
