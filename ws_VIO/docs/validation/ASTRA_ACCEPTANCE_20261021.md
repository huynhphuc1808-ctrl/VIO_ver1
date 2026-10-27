# Astra VIO + mission JSON — nghiệm thu SITL 2026-10-21

Phase 0–5 đã hoàn tất **trong phạm vi các ca SITL được ghi nhận**. Khi tiếp tục,
runtime đã có source nhưng còn thiếu nghiệm thu bay và chưa cutover. Hiện circle,
square và hybrid chạy bằng một runtime JSON; public circle launch là preset của
`vio_mission`. Workflow GNSS+VIO vẫn dùng PX4 Navigator.

Nguồn bằng chứng: [manifest với hash/bằng chứng từng ca](astra_mission_acceptance.json).
Báo cáo này mô tả các episode trước đợt đổi tên 2026-10-28; các artifact lớn
được lưu cục bộ và không đi kèm repository mã nguồn. Nó không chứng nhận binary
mới sau đổi tên; kiểm chứng của phiên bản mới được báo cáo riêng ở root repository.

| Phase | Kết quả | Bằng chứng chính |
|---|---|---|
| 0 Baseline/contract | PASS | Pins/checker, legacy ULog regression, ID động, rollback, ADR frame/clock |
| 1 Runtime/bootstrap | PASS | ground04 delayed clock/negative native arm/reload; minimal03 ownership trước arm |
| 2 Local actions/failure | PASS theo matrix | Minimal flight; VIO/stale/takeover ở Takeoff/Goto/Hold; Hold timeout; process death |
| 3 Native Orbit | PASS | CW, CCW, hai vòng, hai Orbit riêng; native + physical log |
| 4 JSON/lifecycle | PASS theo matrix | Square/hybrid cùng binary; invalid/active edit/reload/reset/reselection/no rearm |
| 5 Cutover | PASS | Public manual-entry flight, bỏ legacy, GNSS+VIO regression, build/tests/docs/review |

26 hồ sơ nghiệm thu đạt, gồm flight, fault, ground và stationary/smoke; 7 cửa sổ
Orbit được phân tích riêng. Manifest liệt kê chính xác phạm vi và hash mỗi hồ sơ.
Circle/CCW/multilap/two-Orbit/square/hybrid/active-edit dùng cùng binary
`26ef8ef…`; binary cuối `757aa316…` bổ sung sửa kiểu số trong circle generator,
được kiểm lại qua public entrypoint. Không tuyên bố mọi ca dùng binary cuối.

## Kết quả đo tiêu biểu

- Stationary90 wall seconds, 59 snapshot trong phút cuối: VIO speed tối đa
  0.0292m/s, local speed0.0317m/s, displacement dưới0.094m; disarmed, không
  command backend, EV pos/vel/hgt hoạt động và không GNSS.
- Public_circle02: chọn thủ công khi ready, xác nhận raw same-executor ownership
  trước arm, Takeoff → Orbit → Hold → Land/disarm. Physical log:1.00124 vòng,
  radiusRMSE0.10866m, median speed0.96235m/s, altitude1.98148m, zpp0.06067m.
- GNSS+VIO02: joint fusion5.112native seconds trước arm; khi GNSS mất fusion,
  waypoint3→4 vẫn tiến; GNSS phục hồi; mất cả hai nguồn dẫn tới native Descend
  rồi landed/disarmed. Cả6 kiểm tra ULog đạt, motion residual tối đa0.00604m,
  không XY/Z/heading reset khi còn aided Mission. Outage đầu đo được2.748s;
  đây không phải nghiệm thu outage dài.
- Default lawn-world disarmed smoke đạt graph/QoS/VIO diagnostics/native joint
  fusion. Color/depth lazy vẫn bật khi có subscriber; stereo/IMU giữ nguyên
  từng sensor block so với archive trước thay đổi.

PNG từ Gazebo GUI của active_edit01 được lưu cùng artifact cục bộ của episode.
Hash, clock, telemetry trước/sau và ULog xác nhận armed Hold ở khoảng2.009m.
Artifact ảnh không được đóng gói trong repository mã nguồn này.

## Các sửa lỗi đã kiểm chứng

Missing companion local telemetry từng ngăn recovery Land trong native Takeoff.
Failure handler giờ yêu cầu native Land một lần khi fresh status còn xác nhận
armed ownership; PX4 giữ mode requirements/failsafe. API1 wrapper được chặn tới
khi lastTime>0 và mission clock hoạt động. File monitor bỏ notification ban đầu
đã nạp trước, và rewrite cùng revision hợp lệ xóa rejection mà không tự reset/start.

Generated signed `laps` và parsed unsigned `laps` từng khác upstream checksum.
Test serialize/parse qua text đã tái hiện lỗi trước fix; preset giờ tạo số nguyên
dương cùng kiểu với parser. Không thay hash/scheduler upstream hoặc nội dung bay.

Hybrid fixture chuyển Gotos từ(2,0),(2,2) sang(1,0),(1,1) để chừa clearance cho
native Orbit radius2m trong scene có tường tại worldx5m; speed và ngưỡng giữ nguyên.
Hybrid02 đạt radiusRMSE vật lý0.0996m và zpp0.0205m. Runtime chưa tránh vật cản.
Chi tiết kỹ thuật: [learning notes](../learning/MISSION_JSON_RUNTIME_VI.md).

## Kiểm tra và các lần thất bại được giữ lại

Full pinned checker và build4 package đạt; toàn bộ package tests đạt. Mission
có12 CTest entries, bridge33 pytest cases, launch-support11 cases và analyzer7
regressions. Các con số có phần chồng lặp; không cộng thành tổng test duy nhất.
Public entrypoint chỉ có executable `vio_mission`; graph từng flight có một
backend. Native GNSS+VIO regression không chạy ROS mission backend.

Các lần thất bại được giữ nguyên trong manifest: prearm timeout, zero-clock
startup crash, freshness abort, checksum mismatch và lỗi logging của test runner.
Đặc biệt stale_local_localtakeoff01 có raw success=true từ harness cũ nhưng cần
operator Land: manual recovery và analyzer khiến ca này vẫn FAIL. Hybrid01 có
physical height gần5.96m, Orbit geometry lỗi và Hold timeout. Swept rotor envelope
ước tính vượt mặt tường khoảng0.019m; đây là suy luận hình học vì không có contact
event. Không tăng deadline hay nới freshness/geometry để biến ca lỗi thành PASS.

## Rollback và giới hạn

Artifact root: `.artifacts/astra_continuation_20261021_105007`. Source trước công
việc và trước cutover được lưu kèm SHA256; `cutover_rollback.json` lưu hash source,
launch, CMake và binary legacy. Legacy executable, symlink install và object target
cũ đã chuyển khỏi build/install, giữ bản lưu trong `legacy_before_cutover`.
Hướng dẫn [build/run/rollback](../development/BUILD_AND_RUN.md).

Không đổi pin/dependency, không stage/commit/reset Git; các thay đổi sẵn của người
dùng được giữ. Các process mô phỏng do phiên tạo đã dừng. Ground truth chỉ đọc
sau flight để đo; không cấp vào OpenVINS, EKF2 hoặc control.

Matrix không bao phủ mọi tổ hợp fault/action. Timeout flight đo ở Hold; các
deadline khác có offline boundary tests. Frame-reset là fault injection trên
alias telemetry vào mission, không phải một EKF reset thật. VIO heartbeat và
fusion không chứng minh chất lượng feature. Hardware, calibration thực, outage
dài/drift lớn, ảnh thiếu feature, Docker và obstacle avoidance chưa được chứng
nhận trong nghiệm thu này.
