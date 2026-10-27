"""Tests for the fixed IMU-to-base transform used after VIO_ver1."""

import math
import os

from nav_msgs.msg import Odometry
import pytest
import rclpy

from uav_vio_px4_bridge.vio_estimator_odometry_adapter import VioOdometryAdapter

os.environ.setdefault("ROS_LOG_DIR", "/tmp/uav_vio_px4_bridge_test_logs")


@pytest.fixture
def adapter():
    """Create the adapter without spinning an executor."""
    rclpy.init()
    node = VioOdometryAdapter()
    node.require_visual_updates = False
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_identity_imu_pose_becomes_expected_base_pose(adapter):
    """Invert the simulation's base-to-IMU mount exactly."""
    source = source_odometry()
    source.header.stamp.sec = 1
    source.pose.pose.orientation.w = 1.0
    source.pose.pose.position.x = 1.0
    source.pose.pose.position.y = 2.0
    source.pose.pose.position.z = 3.0

    captured = []
    adapter.publisher.publish = captured.append
    adapter._on_odometry(source)
    output = captured[0]

    assert output.header.frame_id == "odom"
    assert output.child_frame_id == "base_link"
    assert output.pose.pose.position.x == pytest.approx(0.9)
    assert output.pose.pose.position.y == pytest.approx(2.0)
    assert output.pose.pose.position.z == pytest.approx(2.92)
    assert output.pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))
    assert output.pose.pose.orientation.y == pytest.approx(-math.sqrt(0.5))


def test_duplicate_and_out_of_order_timestamps_are_dropped(adapter):
    """Publish only strictly monotonic odometry from VIO_ver1' two paths."""
    captured = []
    adapter.publisher.publish = captured.append

    first = source_odometry()
    first.header.stamp.sec = 10
    first.pose.pose.orientation.w = 1.0
    adapter._on_odometry(first)

    duplicate = source_odometry()
    duplicate.header.stamp.sec = 10
    duplicate.pose.pose.orientation.w = 1.0
    adapter._on_odometry(duplicate)

    older = source_odometry()
    older.header.stamp.sec = 9
    older.pose.pose.orientation.w = 1.0
    adapter._on_odometry(older)

    newer = source_odometry()
    newer.header.stamp.sec = 10
    newer.header.stamp.nanosec = 1
    newer.pose.pose.orientation.w = 1.0
    adapter._on_odometry(newer)

    assert len(captured) == 2
    assert captured[0].header.stamp.sec == 10
    assert captured[1].header.stamp.nanosec == 1


def test_zero_timestamp_is_dropped(adapter):
    """Never replace missing sensor time with wall or ROS now."""
    captured = []
    adapter.publisher.publish = captured.append
    source = source_odometry()
    source.pose.pose.orientation.w = 1.0
    adapter._on_odometry(source)
    assert captured == []


def test_velocity_is_shifted_from_imu_to_base_origin(adapter):
    """Apply omega cross lever-arm when moving velocity to base_link."""
    source = source_odometry()
    source.header.stamp.sec = 1
    source.pose.pose.orientation.w = 1.0
    source.twist.twist.angular.z = 1.0
    captured = []
    adapter.publisher.publish = captured.append

    adapter._on_odometry(source)

    assert captured[0].twist.twist.linear.x == pytest.approx(0.0)
    assert captured[0].twist.twist.linear.y == pytest.approx(-0.1)
    assert captured[0].twist.twist.linear.z == pytest.approx(0.0)


def test_pose_covariance_includes_lever_arm_and_axis_rotation(adapter):
    """Propagate attitude uncertainty into the shifted base position."""
    source = source_odometry()
    source.header.stamp.sec = 1
    source.pose.pose.orientation.w = 1.0
    source.pose.covariance[21] = 0.04
    source.pose.covariance[28] = 0.09
    source.pose.covariance[35] = 0.16
    captured = []
    adapter.publisher.publish = captured.append

    adapter._on_odometry(source)

    covariance = captured[0].pose.covariance
    assert covariance[0] == pytest.approx(0.000576)
    assert covariance[7] == pytest.approx(0.001856)
    assert covariance[14] == pytest.approx(0.0009)
    assert covariance[21] == pytest.approx(0.16)
    assert covariance[28] == pytest.approx(0.09)
    assert covariance[35] == pytest.approx(0.04)


def test_twist_covariance_includes_angular_lever_uncertainty(adapter):
    """Transform angular uncertainty into base-origin velocity uncertainty."""
    source = source_odometry()
    source.header.stamp.sec = 1
    source.pose.pose.orientation.w = 1.0
    source.twist.covariance[35] = 0.25
    captured = []
    adapter.publisher.publish = captured.append

    adapter._on_odometry(source)

    covariance = captured[0].twist.covariance
    assert covariance[7] == pytest.approx(0.0025)
    assert covariance[21] == pytest.approx(0.25)


def source_odometry():
    """Create an odometry sample using the checked VIO_ver1 frame contract."""
    message = Odometry()
    message.header.frame_id = "global"
    message.child_frame_id = "imu"
    return message


def test_imu_only_propagation_expires_when_visual_state_stops(adapter):
    """High-rate IMU propagation alone is not healthy visual odometry."""
    from geometry_msgs.msg import PoseWithCovarianceStamped
    adapter.require_visual_updates = True
    captured = []
    adapter.publisher.publish = captured.append
    sample = source_odometry()
    sample.header.stamp.sec = 10
    sample.pose.pose.orientation.w = 1.0
    adapter._on_odometry(sample)
    assert not captured
    visual = PoseWithCovarianceStamped()
    visual.header.stamp = sample.header.stamp
    adapter._on_visual_update(visual)
    adapter._on_odometry(sample)
    assert len(captured) == 1
    # Input is not modified by the lever-arm transform.
    assert sample.pose.pose.position.x == 0.0
    sample.header.stamp.sec = 11
    adapter._on_odometry(sample)
    assert len(captured) == 1


def test_invalid_upstream_quaternion_is_not_normalized_to_identity(adapter):
    """Do not turn invalid orientation into an apparently valid measurement."""
    sample = source_odometry()
    sample.header.stamp.sec = 1
    sample.pose.pose.orientation.w = 0.0
    captured = []
    adapter.publisher.publish = captured.append
    adapter._on_odometry(sample)
    assert not captured


def test_mislabeled_fallback_with_fabricated_covariance_is_rejected(adapter):
    """Never fuse the installed VIO_ver1 publish_state fallback as body velocity."""
    source = source_odometry()
    source.header.stamp.sec = 1
    source.pose.pose.orientation.w = 1.0
    source.twist.twist.linear.x = 3.0
    source.twist.covariance = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]
    captured = []
    adapter.publisher.publish = captured.append
    adapter._on_odometry(source)
    assert not captured
    assert adapter.last_stamp_ns is None
    assert adapter.rejected_fallback_samples == 1


def test_asymmetric_covariance_is_rejected_before_symmetrization(adapter):
    """Do not hide corrupted off-diagonals by averaging them to zero."""
    source = source_odometry()
    source.header.stamp.sec = 1
    source.pose.pose.orientation.w = 1.0
    source.pose.covariance[1] = 100.0
    source.pose.covariance[6] = -100.0
    captured = []
    adapter.publisher.publish = captured.append
    adapter._on_odometry(source)
    assert not captured
