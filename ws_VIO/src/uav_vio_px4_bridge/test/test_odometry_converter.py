"""Unit tests for external-odometry validation and covariance mapping."""

import math
import os

from nav_msgs.msg import Odometry
import pytest
import rclpy

from uav_vio_px4_bridge.ros_odometry_to_px4_visual_odometry import (
    RosOdometryToPx4VisualOdometry,
)

os.environ.setdefault("ROS_LOG_DIR", "/tmp/uav_vio_px4_bridge_test_logs")


@pytest.fixture
def converter():
    """Create a converter node without spinning an executor."""
    rclpy.init()
    node = RosOdometryToPx4VisualOdometry()
    node.max_velocity_variance = 20.0
    yield node
    node.destroy_node()
    rclpy.shutdown()


def valid_odometry(node):
    """Build one valid, current odom-to-base_link sample."""
    message = Odometry()
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.pose.orientation.w = 1.0
    for index in (0, 7, 14, 21, 28, 35):
        message.pose.covariance[index] = 0.01
        message.twist.covariance[index] = 0.01
    return message


def test_valid_sample_and_covariance_axis_mapping(converter):
    """Accept a healthy sample and preserve FLU/FRD variance axes."""
    message = valid_odometry(converter)
    sample_us = converter._stamp_us(message)
    assert converter._validate_odometry(
        message, sample_us, (0.0, 0.0, 0.0)
    )
    covariance = [0.0] * 36
    covariance[0] = 1.0
    covariance[7] = 2.0
    covariance[14] = 3.0
    assert converter._variance3_from_pose_covariance(
        covariance
    ) == pytest.approx([1.0, 2.0, 3.0])


def test_invalid_quaternion_and_nonfinite_covariance_are_rejected(converter):
    """Reject zero quaternion and non-finite covariance."""
    message = valid_odometry(converter)
    message.pose.pose.orientation.w = 0.0
    sample_us = converter._stamp_us(message)
    assert not converter._validate_odometry(
        message, sample_us, (0.0, 0.0, 0.0)
    )

    message = valid_odometry(converter)
    message.pose.covariance[0] = math.nan
    sample_us = converter._stamp_us(message)
    assert not converter._validate_odometry(
        message, sample_us, (0.0, 0.0, 0.0)
    )


def test_jump_is_quarantined_without_reset(converter):
    """Reject an unannounced discontinuity without changing the EKF reset counter."""
    message = valid_odometry(converter)
    sample_us = converter._stamp_us(message)
    converter.last_position_flu = (0.0, 0.0, 0.0)
    converter.last_sample_us = sample_us - 1
    assert not converter._validate_odometry(
        message, sample_us, (2.0, 0.0, 0.0)
    )
    assert converter.reset_counter == 0
    assert converter.discontinuity_latched
    assert not converter._validate_odometry(
        message, sample_us + 1, (2.1, 0.0, 0.0)
    )
    assert converter.last_position_flu == (0.0, 0.0, 0.0)


def test_zero_sensor_timestamp_is_rejected(converter):
    """Never substitute ROS now for a missing estimator timestamp."""
    message = valid_odometry(converter)
    message.header.stamp.sec = 0
    message.header.stamp.nanosec = 0
    assert converter._stamp_us(message) == 0
    assert not converter._validate_odometry(message, 0, (0.0, 0.0, 0.0))


def test_output_uses_arbitrary_heading_frd_and_vio_estimator_velocity(converter):
    """Rotate base-frame estimator velocity into arbitrary-heading FRD."""
    message = valid_odometry(converter)
    message.pose.pose.position.x = 1.0
    message.pose.pose.position.y = 2.0
    message.pose.pose.position.z = 3.0
    message.twist.twist.linear.x = 4.0
    message.twist.twist.linear.y = 5.0
    message.twist.twist.linear.z = 6.0
    message.twist.covariance[0] = 0.1
    message.twist.covariance[7] = 0.2
    message.twist.covariance[14] = 0.3
    captured = []
    converter.visual_odom_pub.publish = captured.append

    converter._on_odometry(message)

    output = captured[0]
    assert output.pose_frame == output.POSE_FRAME_FRD
    assert output.position == pytest.approx([1.0, -2.0, -3.0])
    assert output.velocity_frame == output.VELOCITY_FRAME_FRD
    assert output.velocity == pytest.approx([4.0, -5.0, -6.0])
    assert output.velocity_variance == pytest.approx([0.1, 0.2, 0.3])


def test_body_twist_and_covariance_rotate_through_odometry_attitude(converter):
    """Respect the nav_msgs child-frame convention before publishing PX4 EV."""
    message = valid_odometry(converter)
    half_sqrt = math.sqrt(0.5)
    message.pose.pose.orientation.w = half_sqrt
    message.pose.pose.orientation.z = half_sqrt
    message.twist.twist.linear.x = 1.0
    message.twist.twist.angular.x = 1.0
    message.twist.twist.angular.y = 2.0
    message.twist.twist.angular.z = 3.0
    message.twist.covariance[0] = 1.0
    message.twist.covariance[7] = 4.0
    message.twist.covariance[14] = 9.0
    captured = []
    converter.visual_odom_pub.publish = captured.append

    converter._on_odometry(message)

    output = captured[0]
    assert output.velocity == pytest.approx([0.0, -1.0, 0.0], abs=1e-7)
    assert output.velocity_variance == pytest.approx([4.0, 1.0, 9.0])
    assert output.angular_velocity == pytest.approx([1.0, -2.0, -3.0])


@pytest.mark.parametrize("kind", ["zero", "negative", "indefinite", "velocity_nan", "uncertain"])
def test_invalid_measurements_are_never_published(converter, kind):
    """Never replace unknown covariance with fabricated precision."""
    message = valid_odometry(converter)
    if kind == "zero":
        message.twist.covariance = [0.0] * 36
    elif kind == "negative":
        message.pose.covariance[0] = -0.1
    elif kind == "indefinite":
        message.twist.covariance[1] = 1.0
        message.twist.covariance[6] = 1.0
    elif kind == "velocity_nan":
        message.twist.twist.linear.x = math.nan
    else:
        message.twist.covariance[0] = converter.max_velocity_variance + 1
    captured = []
    converter.visual_odom_pub.publish = captured.append
    converter._on_odometry(message)
    assert captured == []


def test_sensor_time_is_preserved_and_stale_samples_cannot_refresh_health(converter):
    """Use sensor time and let stale data expire without resetting EKF2."""
    message = valid_odometry(converter)
    captured = []
    converter.visual_odom_pub.publish = captured.append
    converter._on_odometry(message)
    assert captured[0].timestamp_sample == converter._stamp_us(message)
    assert captured[0].reset_counter == 0
    accepted_at = converter.last_accept_monotonic
    message.header.stamp.sec -= 2
    converter._on_odometry(message)
    assert len(captured) == 1
    assert converter.last_accept_monotonic == accepted_at


def test_clock_pause_expires_diagnostics(converter):
    """A paused ROS clock cannot leave a healthy heartbeat latched forever."""
    message = valid_odometry(converter)
    converter.visual_odom_pub.publish = lambda msg: None
    converter._on_odometry(message)
    converter.last_accept_monotonic -= 1.0
    statuses = []
    converter.status_pub.publish = statuses.append
    converter._publish_status()
    assert statuses[-1].data.startswith("LOST")


def test_motion_during_transport_gap_is_not_an_estimator_reset(converter):
    """Accept 1.2m of expected motion at 3m/s over a 0.4s transport gap."""
    sample = valid_odometry(converter)
    sample_us = converter._stamp_us(sample)
    converter.last_sample_us = sample_us - 400_000
    converter.last_position_flu = (0.0, 0.0, 0.0)
    converter.last_velocity_flu = (3.0, 0.0, 0.0)
    assert converter._validate_odometry(sample, sample_us, (1.2, 0.0, 0.0))
    assert not converter.discontinuity_latched


def test_old_rejection_cannot_mask_stale_stream(converter):
    """Report LOST once the last good data expires, even after a rejected sample."""
    sample = valid_odometry(converter)
    converter.visual_odom_pub.publish = lambda msg: None
    converter._on_odometry(sample)
    converter._reject("bad sample")
    converter.last_accept_monotonic -= 1.0
    statuses = []
    converter.status_pub.publish = statuses.append
    converter._publish_status()
    assert statuses[-1].data.startswith("LOST")
