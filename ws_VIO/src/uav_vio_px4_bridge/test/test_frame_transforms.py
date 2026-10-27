"""Unit tests for centralized ROS/PX4 frame conversions."""

import math

import pytest

from uav_vio_px4_bridge.frame_transforms import (
    quaternion_from_euler,
    quaternion_get_yaw,
    quaternion_to_rotation_matrix,
    rotate_vector,
    transform_orientation_local_flu_to_frd,
    transform_orientation_px4_to_ros,
    transform_orientation_ros_to_px4,
    transform_vector_enu_to_ned,
    transform_vector_local_flu_to_frd,
    transform_vector_ned_to_enu,
)


def test_quaternion_rotation_matrix_matches_vector_rotation():
    """Keep covariance and mean rotations on the same convention."""
    quaternion = quaternion_from_euler(0.3, -0.2, 0.7)
    vector = (1.0, -2.0, 0.5)
    matrix = quaternion_to_rotation_matrix(quaternion)
    via_matrix = tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )
    assert via_matrix == pytest.approx(rotate_vector(quaternion, vector))


def test_enu_ned_vector_mapping_and_inverse():
    """Map east/north/up into north/east/down and back."""
    ned = transform_vector_enu_to_ned(1.0, 2.0, 3.0)
    assert ned == pytest.approx((2.0, 1.0, -3.0))
    assert transform_vector_ned_to_enu(*ned) == pytest.approx(
        (1.0, 2.0, 3.0)
    )


def test_arbitrary_local_flu_to_frd_mapping():
    """Preserve forward and flip left/up for PX4's arbitrary-heading frame."""
    assert transform_vector_local_flu_to_frd(1.0, 2.0, 3.0) == pytest.approx(
        (1.0, -2.0, -3.0)
    )
    identity = transform_orientation_local_flu_to_frd(
        quaternion_from_euler(0.0, 0.0, 0.0)
    )
    assert abs(identity.w) == pytest.approx(1.0)
    assert identity.x == pytest.approx(0.0, abs=1e-12)
    yaw = transform_orientation_local_flu_to_frd(
        quaternion_from_euler(0.0, 0.0, math.pi / 2.0)
    )
    assert quaternion_get_yaw(yaw) == pytest.approx(-math.pi / 2.0)


@pytest.mark.parametrize(
    "roll,pitch,yaw",
    [
        (0.0, 0.0, 0.0),
        (0.2, -0.1, math.pi / 2.0),
        (-0.4, 0.3, -2.0),
    ],
)
def test_orientation_round_trip(roll, pitch, yaw):
    """Preserve orientation through ROS-to-PX4-to-ROS conversion."""
    original = quaternion_from_euler(roll, pitch, yaw)
    recovered = transform_orientation_px4_to_ros(
        transform_orientation_ros_to_px4(original)
    )
    dot = (
        original.w * recovered.w
        + original.x * recovered.x
        + original.y * recovered.y
        + original.z * recovered.z
    )
    assert abs(dot) == pytest.approx(1.0, abs=1e-7)
    assert quaternion_get_yaw(recovered) == pytest.approx(yaw, abs=1e-7)
