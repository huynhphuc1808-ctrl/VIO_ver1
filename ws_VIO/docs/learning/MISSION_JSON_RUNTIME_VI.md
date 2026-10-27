# Đọc runtime mission JSON

Mục tiêu của migration là thay chuỗi callback riêng cho từng quỹ đạo bằng dữ
liệu mission và các action dùng lại được. PX4 vẫn giữ bộ điều khiển và EKF2.
Đọc [kiến trúc](../architecture/MISSION_ARCHITECTURE.md) cùng source.

1. Bắt đầu ở `mission_definition.cpp`: một file square khác circle bằng danh
   sách action; cùng executable đọc chúng. Validator chạy trước registration,
   nên JSON hỏng không tạo command/setpoint endpoint.
2. Đọc `MissionContext` và `StableReadiness`: một heartbeat mới không xóa được
   transport gap trước đó. Steady clock đo tuổi nhận message; ROS clock đo tiến
   độ mô phỏng. Nếu simulator chậm, hai clock cho hai ý nghĩa khác nhau.
3. Đọc bootstrap trong `mission_runtime.cpp`: selection và quyền executor phải
   có trước arm; activation upstream cần armed. Không dùng hai backend để phá
   vòng phụ thuộc này. Các ID đăng ký là dữ liệu runtime, không phải hằng số.
4. Đọc localTakeoff rồi `InitialFrameState`: magnetic alignment có thể reset
   local state lúc cất cánh. Provisional reference được sửa trong cửa sổ startup
   hữu hạn; frozen mission frame không được tự động bù reset.
5. Đọc invocation guard: callback của action trước có thể đến muộn. Token bị
   hủy khi deactivate/failure; callback cũ không được advance action hiện tại.
6. Đọc MissionFileMonitor và error observer: parse error có thể bị upstream giữ
   trong log, nên cần một observer chỉ báo lỗi. Nó không được trở thành nguồn
   thứ hai áp dụng mission hoặc làm mất rejection của active edit.

Bài học đã đo ngày 2026-10-21: fresh ordinary headless bringup giữ VIO/local
drift dưới 0.094 m trong phút cuối của cửa sổ 90 wall seconds, trong khi attempt5
cũ báo `VALID_ODOMETRY` và EV fusion dù VIO trôi gần 40 m. Chưa đủ dữ liệu để kết
luận nguyên nhân khác biệt. Vì vậy phải giữ prearm stationary measurement và
kiểm ULog; không điều chỉnh threshold cho một flight lỗi trở thành PASS.

Offline tests chứng minh parser/guard và một số hợp đồng riêng. Flight success,
stale/takeover handling, live reload và rollback/cutover cần bằng chứng runtime
riêng. Xem execution plan để phân biệt phần đã có mã và phần đã nghiệm thu.

## Hai lỗi integration đã tìm được

Collector ban đầu lưu `latest.json` mỗi 1 s. Một runner đọc file này để kiểm
freshness 0.5 s liên tục sẽ tự reset cửa sổ ổn định, dù message ROS thật vẫn
đều. Sửa đúng là tăng tần suất snapshot phục vụ runner lên 0.1 s; giữ nguyên
receipt freshness của flight code. Không sửa threshold để làm test xanh.

`stale_local_localtakeoff01` chứng minh một lỗi khác: runtime đã abort khi ROS
local telemetry mất, nhưng failure handler lại đòi chính local telemetry fresh
để yêu cầu Land. Native Takeoff vẫn tiếp tục vì estimator PX4 còn tốt. Ca này
cần operator Land và được giữ là FAIL. ADR-005 bỏ điều kiện ROS-local đó;
`stale_local_localtakeoff02` xác nhận một native Land từ cùng executor, hủy
setpoint cũ, landed/disarmed và không resume/rearm. Điều kiện fresh VehicleStatus,
armed ownership và native Commander checks vẫn giữ nguyên. Đây là phân biệt
giữa dữ liệu companion cần để lập target và dữ liệu PX4 dùng để tự hạ cánh.


## Clock, chuyển động vật lý và clearance

API1 `Subscription::last()` coi lastTime=0 là chưa nhận message. ROS có thể nhận
telemetry trước mẫu `/clock` đầu tiên; vì vậy callback tới chưa đảm bảo wrapper
đọc được. Guard dùng public lastTime>0 và chờ clock mới. Ground04 cố tình trễ
clock của mission12s đã xác nhận không crash/arm rồi phục hồi bình thường.

Hybrid01 nhắc rằng EKF local hợp lệ và EV fusion chưa chứng minh drone thật đi
đúng quỹ đạo. Log vật lý cho thấy cao5.96m trong khi EKF tối đa khoảng2.15m.
Swept rotor envelope ước tính chạm tường tại worldx5m; đây là suy luận hình học,
không có contact event để kết luận va chạm trực tiếp. Mission timeout rồi native
Land đã kết thúc chuyến bay lỗi. Dời waypoint vào(1,0),(1,1), giữ radius2m/speed1m/s,
cho hybrid02 đạt; không kéo dài deadline để che lỗi. Planner/người viết mission
phải chừa clearance cho cả rotor và sai số. Runtime hiện không tránh vật cản.
Ground truth chỉ được đọc sau flight để đo, không dùng sửa VIO hoặc điều khiển.

Revision của MissionExecutor API1 hash cả kiểu số của nlohmann JSON. Số dương
`laps` do parser text tạo là unsigned, còn C++ int tạo signed. Hai JSON so sánh
bằng nhau vẫn có thể khác checksum. Test export qua text đã tái hiện trước fix;
preset giờ tạo laps unsigned như file. Không sửa checksum upstream hay tự duy
trì scheduler khác. Khi kiểm round-trip, cần thực sự serialize/parse qua text,
không chỉ chuyển giữa hai object trong bộ nhớ.
