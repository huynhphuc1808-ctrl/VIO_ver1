# VIO_ver1: mapping source và module

VIO_ver1 là bản tích hợp được đổi tên từ [OpenVINS](https://github.com/rpng/open_vins),
commit nền `69488123ed9362dd44b6f28e7f4680abbff1442b` (package version `2.7.0`),
bao gồm các sửa đổi cục bộ đã có trong bản tham chiếu. Đợt đổi tên 2026-10-28
giữ nguyên thuật toán, phép tính, tham số và calibration; đổi package/library,
namespace C++, header guard, biến mang tiền tố cũ, tên adapter, namespace/topic
ROS và tham chiếu build/launch tương ứng. Tên class/hàm mô tả thuật toán như
`VioManager`, `TrackKLT`, `Propagator` được giữ nguyên.

| Thư mục trong `ws_vio_references/VIO_ver1/` | Vai trò |
| --- | --- |
| `vio_core/` | Camera, feature tracking, triangulation, quaternion và kiểu dữ liệu |
| `vio_init/` | Khởi tạo quán tính tĩnh/động và Ceres factors |
| `vio_msckf/` | Propagation, state, MSCKF/SLAM update và ROS adapter |
| `vio_config/` | Cấu hình và calibration baseline |

Build ba package bằng `../../ws_VIO/scripts/build_vio.sh`; ứng dụng dùng
calibration D435i tại `ws_VIO/src/uav_vio_bringup/config/d435i_gazebo` và
namespace `/vio_msckf`. Các launch ROS 1/dataset tùy chọn là tài liệu tham chiếu;
workflow được tích hợp là ROS 2 Jazzy. Xem báo cáo kiểm chứng ở root repository.

MIT của [matplotlib-cpp](vio_core/src/plot/LICENSE) được giữ nguyên; phần thông
báo OpenVINS được bỏ theo quyền riêng mà người duy trì xác nhận. Không copy `.git`, cache,
`vio_eval`, `vio_data`; các script/launch ROS 1 đánh giá và config `rpng_sim` vẫn
cần các phần đánh giá/dataset tùy chọn đó. Cấu hình T265 được giữ làm baseline.

Các mục “tùy chọn” có trong source, được chọn theo cấu hình.

| Algorithm/module | File → class/function chính |
| --- | --- |
| Feature detection: FAST, grid, subpixel | [Grider_GRID.h](vio_core/src/track/Grider_GRID.h), [Grider_FAST.h](vio_core/src/track/Grider_FAST.h) → `Grider_GRID::perform_griding`, `Grider_FAST::perform_griding` |
| Image preprocessing: histogram/CLAHE | [TrackKLT.cpp](vio_core/src/track/TrackKLT.cpp) → `feed_new_camera`; [TrackDescriptor.cpp](vio_core/src/track/TrackDescriptor.cpp) → `feed_monocular`, `feed_stereo` |
| KLT tracking | [TrackKLT.cpp](vio_core/src/track/TrackKLT.cpp) → `TrackKLT::feed_monocular`, `perform_matching` |
| ORB descriptor matching (tùy chọn) | [TrackDescriptor.cpp](vio_core/src/track/TrackDescriptor.cpp) → `TrackDescriptor::perform_detection_monocular`, `robust_match`, `robust_ratio_test`, `robust_symmetry_test` |
| Stereo matching | [TrackKLT.cpp](vio_core/src/track/TrackKLT.cpp), [TrackDescriptor.cpp](vio_core/src/track/TrackDescriptor.cpp) → `feed_stereo`, `perform_detection_stereo` |
| Geometric outlier rejection / RANSAC | [TrackKLT.cpp](vio_core/src/track/TrackKLT.cpp) → `perform_matching`; [TrackDescriptor.cpp](vio_core/src/track/TrackDescriptor.cpp) → `robust_match` |
| Feature history / selection / cleanup | [FeatureDatabase.cpp](vio_core/src/feat/FeatureDatabase.cpp) → `FeatureDatabase::update_feature`, `features_not_containing_newer`, `features_containing_older`, `cleanup` |
| Disparity / motion detection | [FeatureHelper.h](vio_core/src/feat/FeatureHelper.h) → `FeatureHelper::compute_disparity` |
| Camera projection / distortion | [CamRadtan.h](vio_core/src/cam/CamRadtan.h), [CamEqui.h](vio_core/src/cam/CamEqui.h) → `undistort_f`, `distort_f`, `compute_distort_jacobian` |
| IMU initialization selection | [InertialInitializer.cpp](vio_init/src/init/InertialInitializer.cpp) → `InertialInitializer::initialize` |
| Static IMU initialization | [StaticInitializer.cpp](vio_init/src/static/StaticInitializer.cpp) → `StaticInitializer::initialize` |
| Dynamic initialization (tùy chọn) | [DynamicInitializer.cpp](vio_init/src/dynamic/DynamicInitializer.cpp) → `DynamicInitializer::initialize`; [helper.h](vio_init/src/utils/helper.h) → `InitializerHelper::gram_schmidt` |
| IMU preintegration cho dynamic init (tùy chọn) | [CpiV1.cpp](vio_core/src/cpi/CpiV1.cpp) → `CpiV1::feed_IMU` |
| Ceres refinement / factors cho dynamic init (tùy chọn) | [DynamicInitializer.cpp](vio_init/src/dynamic/DynamicInitializer.cpp) → `DynamicInitializer::initialize`; [Factor_ImuCPIv1.cpp](vio_init/src/ceres/Factor_ImuCPIv1.cpp), [Factor_ImageReprojCalib.cpp](vio_init/src/ceres/Factor_ImageReprojCalib.cpp), [Factor_GenericPrior.cpp](vio_init/src/ceres/Factor_GenericPrior.cpp) → `Evaluate` |
| IMU buffering / interval selection / interpolation | [Propagator.h](vio_msckf/src/state/Propagator.h), [Propagator.cpp](vio_msckf/src/state/Propagator.cpp) → `Propagator::feed_imu`, `select_imu_readings`, `interpolate_data` |
| IMU propagation | [Propagator.cpp](vio_msckf/src/state/Propagator.cpp) → `propagate_and_clone`, `predict_and_compute`, `predict_mean_discrete`, `predict_mean_rk4`, `predict_mean_analytic`, `fast_state_propagate` |
| State transition / process-noise Jacobians | [Propagator.cpp](vio_msckf/src/state/Propagator.cpp) → `compute_F_and_G_discrete`, `compute_F_and_G_analytic` |
| Covariance propagation / EKF correction | [StateHelper.cpp](vio_msckf/src/state/StateHelper.cpp) → `StateHelper::EKFPropagation`, `EKFUpdate` |
| Triangulation | [FeatureInitializer.cpp](vio_core/src/feat/FeatureInitializer.cpp) → `FeatureInitializer::single_triangulation`, `single_triangulation_1d` |
| Feature refinement | [FeatureInitializer.cpp](vio_core/src/feat/FeatureInitializer.cpp) → `FeatureInitializer::single_gaussnewton`, `compute_error` |
| Landmark representation / anchor change | [LandmarkRepresentation.h](vio_core/src/types/LandmarkRepresentation.h) → `LandmarkRepresentation`; [UpdaterSLAM.cpp](vio_msckf/src/update/UpdaterSLAM.cpp) → `change_anchors`, `perform_anchor_change` |
| MSCKF update | [UpdaterMSCKF.cpp](vio_msckf/src/update/UpdaterMSCKF.cpp) → `UpdaterMSCKF::update` |
| SLAM landmark update / delayed init (tùy chọn) | [UpdaterSLAM.cpp](vio_msckf/src/update/UpdaterSLAM.cpp) → `UpdaterSLAM::update`, `delayed_init`; [StateHelper.cpp](vio_msckf/src/state/StateHelper.cpp) → `initialize`, `initialize_invertible` |
| Jacobian / residual | [UpdaterHelper.cpp](vio_msckf/src/update/UpdaterHelper.cpp) → `UpdaterHelper::get_feature_jacobian_full`, `get_feature_jacobian_representation` |
| Nullspace projection | [UpdaterHelper.cpp](vio_msckf/src/update/UpdaterHelper.cpp) → `UpdaterHelper::nullspace_project_inplace` |
| Chi-square gating | [UpdaterMSCKF.cpp](vio_msckf/src/update/UpdaterMSCKF.cpp), [UpdaterSLAM.cpp](vio_msckf/src/update/UpdaterSLAM.cpp) → `update`; [UpdaterZeroVelocity.cpp](vio_msckf/src/update/UpdaterZeroVelocity.cpp) → `try_update`; [StateHelper.cpp](vio_msckf/src/state/StateHelper.cpp) → `initialize` |
| Measurement compression / QR (Givens) | [UpdaterHelper.cpp](vio_msckf/src/update/UpdaterHelper.cpp) → `UpdaterHelper::measurement_compress_inplace` |
| FEJ | [UpdaterHelper.cpp](vio_msckf/src/update/UpdaterHelper.cpp) → `get_feature_jacobian_full`, `get_feature_jacobian_representation`; [Propagator.cpp](vio_msckf/src/state/Propagator.cpp) → `compute_F_and_G_discrete`, `compute_F_and_G_analytic`; [StateOptions.h](vio_msckf/src/state/StateOptions.h) → `StateOptions::do_fej` |
| Clone augmentation / marginalization | [StateHelper.cpp](vio_msckf/src/state/StateHelper.cpp) → `StateHelper::clone`, `augment_clone`, `marginalize`, `marginalize_old_clone`, `marginalize_slam` |
| Camera calibration / time offset | [VioManagerOptions.h](vio_msckf/src/core/VioManagerOptions.h) → `print_and_load_state`; [State.h](vio_msckf/src/state/State.h) → `_cam_intrinsics`, `_calib_IMUtoCAM`, `_calib_dt_CAMtoIMU`; [UpdaterHelper.cpp](vio_msckf/src/update/UpdaterHelper.cpp) → `get_feature_jacobian_full`; [StateHelper.cpp](vio_msckf/src/state/StateHelper.cpp) → `augment_clone`; [Propagator.cpp](vio_msckf/src/state/Propagator.cpp) → `propagate_and_clone` |
| IMU intrinsic calibration (tùy chọn) | [State.h](vio_msckf/src/state/State.h) → `State::Dm`, `State::Tg`; [Propagator.cpp](vio_msckf/src/state/Propagator.cpp) → `compute_H_Dw`, `compute_H_Da`, `compute_H_Tg` |
| Zero-velocity update (tùy chọn) | [UpdaterZeroVelocity.cpp](vio_msckf/src/update/UpdaterZeroVelocity.cpp) → `UpdaterZeroVelocity::try_update` |
| ArUco tracking (tùy chọn) | [TrackAruco.cpp](vio_core/src/track/TrackAruco.cpp) → `TrackAruco::feed_new_camera`, `perform_tracking` |
| Quaternion / state retraction | [JPLQuat.h](vio_core/src/types/JPLQuat.h), [PoseJPL.h](vio_core/src/types/PoseJPL.h), [IMU.h](vio_core/src/types/IMU.h) → `update`; [State_JPLQuatLocal.cpp](vio_init/src/ceres/State_JPLQuatLocal.cpp) → `State_JPLQuatLocal::Plus` |
| Estimator orchestration | [VioManager.cpp](vio_msckf/src/core/VioManager.cpp), [VioManagerHelper.cpp](vio_msckf/src/core/VioManagerHelper.cpp) → `VioManager::track_image_and_update`, `do_feature_propagate_update`, `try_to_initialize` |
| ROS 2 stereo/IMU input và odometry output | [ROS2Visualizer.cpp](vio_msckf/src/ros/ROS2Visualizer.cpp) → `ROS2Visualizer::setup_subscribers`, `callback_stereo`, `callback_inertial`, `visualize_odometry`, `publish_state` |
