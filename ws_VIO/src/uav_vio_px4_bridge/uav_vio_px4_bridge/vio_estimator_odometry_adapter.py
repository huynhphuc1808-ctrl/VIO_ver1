"""Convert VIO_ver1 IMU odometry into the ws_VIO base-link contract."""

import copy
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .frame_transforms import (
    Quaternion,
    quaternion_conjugate,
    quaternion_from_euler,
    quaternion_to_rotation_matrix,
    rotate_vector,
)


def _matrix_multiply(
    left: list[list[float]], right: list[list[float]]
) -> list[list[float]]:
    """Multiply two small dense matrices without an extra runtime dependency."""
    return [
        [
            sum(left[row][inner] * right[inner][column] for inner in range(len(right)))
            for column in range(len(right[0]))
        ]
        for row in range(len(left))
    ]


def _matrix_transpose(matrix: list[list[float]]) -> list[list[float]]:
    """Transpose a small dense matrix."""
    return [list(column) for column in zip(*matrix)]


def _skew(vector: tuple[float, float, float]) -> list[list[float]]:
    """Return the cross-product matrix for a 3-vector."""
    x, y, z = vector
    return [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]]


def _covariance_transform(
    covariance: list[float], jacobian: list[list[float]]
) -> list[float]:
    """Apply a first-order six-state covariance transform."""
    source = [list(covariance[row * 6:(row + 1) * 6]) for row in range(6)]
    transformed = _matrix_multiply(
        _matrix_multiply(jacobian, source), _matrix_transpose(jacobian)
    )
    # Remove tiny numerical asymmetry before handing the covariance to PX4.
    for row in range(6):
        for column in range(row):
            symmetric = 0.5 * (
                transformed[row][column] + transformed[column][row]
            )
            transformed[row][column] = symmetric
            transformed[column][row] = symmetric
    return [value for row in transformed for value in row]


def _block_jacobian(
    upper_left: tuple[tuple[float, float, float], ...],
    upper_right: list[list[float]],
    lower_right: tuple[tuple[float, float, float], ...],
) -> list[list[float]]:
    """Build a 6x6 upper-triangular block Jacobian."""
    result = [[0.0] * 6 for _ in range(6)]
    for row in range(3):
        for column in range(3):
            result[row][column] = upper_left[row][column]
            result[row][column + 3] = upper_right[row][column]
            result[row + 3][column + 3] = lower_right[row][column]
    return result


def _negative_product(
    left: tuple[tuple[float, float, float], ...],
    right: list[list[float]],
) -> list[list[float]]:
    """Return the negative product of two 3x3 matrices."""
    return [
        [
            -sum(left[row][inner] * right[inner][column] for inner in range(3))
            for column in range(3)
        ]
        for row in range(3)
    ]


class VioOdometryAdapter(Node):
    """Apply the fixed base-to-D435i transform and normalize frame names."""

    def __init__(self) -> None:
        super().__init__("vio_estimator_odometry_adapter")
        self.declare_parameter("input_topic", "/vio_msckf/odomimu")
        self.declare_parameter("output_topic", "/vio/odometry")
        self.declare_parameter("visual_update_topic", "/vio_msckf/poseimu")
        self.declare_parameter("require_visual_updates", True)
        self.declare_parameter("visual_update_timeout_s", 0.5)
        self.declare_parameter("base_to_imu_xyz", [0.08, 0.0, -0.10])
        self.declare_parameter("base_to_imu_rpy", [0.0, 1.57079632679, 0.0])

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        visual_topic = str(self.get_parameter("visual_update_topic").value)
        self.require_visual_updates = bool(
            self.get_parameter("require_visual_updates").value
        )
        self.visual_update_timeout = float(
            self.get_parameter("visual_update_timeout_s").value
        )
        if not math.isfinite(self.visual_update_timeout) or self.visual_update_timeout <= 0:
            raise ValueError("visual_update_timeout_s must be finite and positive")
        self.last_visual_stamp_ns: int | None = None
        self.last_visual_receive: float | None = None
        xyz = tuple(float(v) for v in self.get_parameter("base_to_imu_xyz").value)
        rpy = tuple(float(v) for v in self.get_parameter("base_to_imu_rpy").value)
        if len(xyz) != 3 or len(rpy) != 3:
            raise ValueError("base_to_imu_xyz and base_to_imu_rpy need 3 values")

        # T_GB = T_GI * inverse(T_BI). Cache inverse(T_BI).
        q_base_imu = quaternion_from_euler(*rpy)
        self.q_base_imu = q_base_imu.normalized()
        self.q_imu_base = quaternion_conjugate(q_base_imu).normalized()
        neg_base_imu = (-xyz[0], -xyz[1], -xyz[2])
        self.p_base_in_imu = rotate_vector(self.q_imu_base, neg_base_imu)

        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.subscription = self.create_subscription(
            Odometry, input_topic, self._on_odometry, qos_profile_sensor_data
        )
        self.visual_subscription = self.create_subscription(
            PoseWithCovarianceStamped, visual_topic, self._on_visual_update,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Adapting VIO_ver1 IMU odometry: {input_topic} -> {output_topic}"
        )
        self.last_stamp_ns: int | None = None
        self.rejected_fallback_samples = 0

    def _on_visual_update(self, message: PoseWithCovarianceStamped) -> None:
        # This checkout also emits IMU-only high-rate odomimu. poseimu carries
        # the camera-updated state timestamp. Repeated/stalled visual states
        # must not keep that propagation stream eligible as external vision.
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns > 0 and (
            self.last_visual_stamp_ns is None or stamp_ns > self.last_visual_stamp_ns
        ):
            self.last_visual_stamp_ns = stamp_ns
            self.last_visual_receive = time.monotonic()

    def _on_odometry(self, source: Odometry) -> None:
        stamp_ns = (
            int(source.header.stamp.sec) * 1_000_000_000
            + int(source.header.stamp.nanosec)
        )
        if stamp_ns <= 0:
            return
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return
        if source.header.frame_id != "global" or source.child_frame_id != "imu":
            return
        # The installed checkout has a publish_state() fallback on the same
        # topic: GLOBAL velocity mislabeled as body IMU, fixed 0.01 variances,
        # and zero angular rate. There is no real velocity covariance to recover.
        # Reject its entire signature; never infer precision or reinterpret it
        # as the genuine fast_state_propagate() output. A matching legitimate
        # sample is conservatively rejected too.
        angular = source.twist.twist.angular
        fallback_covariance = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]
        if (angular.x == angular.y == angular.z == 0.0
                and list(source.twist.covariance) == fallback_covariance):
            self.rejected_fallback_samples += 1
            if self.rejected_fallback_samples == 1:
                self.get_logger().error(
                    "Rejecting mixed VIO_ver1 fallback: global velocity labeled imu "
                    "with fabricated covariance; require genuine propagated odometry"
                )
            return
        # Do this BEFORE the Jacobian helper removes numerical asymmetry.
        # Symmetrizing arbitrary bad input could otherwise manufacture a valid
        # covariance that the downstream converter can no longer reject.
        for values in (source.pose.covariance, source.twist.covariance):
            covariance = np.asarray(values).reshape(6, 6)
            if (not np.isfinite(covariance).all()
                    or not np.allclose(covariance, covariance.T, atol=1e-8)
                    or np.linalg.eigvalsh(covariance).min() < -1e-8):
                return
        if self.require_visual_updates:
            if self.last_visual_stamp_ns is None:
                return
            visual_age = (stamp_ns - self.last_visual_stamp_ns) / 1e9
            if (not -0.1 <= visual_age <= self.visual_update_timeout
                    or time.monotonic() - self.last_visual_receive > self.visual_update_timeout):
                return

        orientation = source.pose.pose.orientation
        values = (orientation.w, orientation.x, orientation.y, orientation.z)
        if not all(math.isfinite(value) for value in values):
            return
        if abs(math.sqrt(sum(value * value for value in values)) - 1.0) > 0.01:
            return

        q_global_imu = Quaternion(
            w=float(source.pose.pose.orientation.w),
            x=float(source.pose.pose.orientation.x),
            y=float(source.pose.pose.orientation.y),
            z=float(source.pose.pose.orientation.z),
        ).normalized()
        q_global_base = (q_global_imu * self.q_imu_base).normalized()
        offset_global = rotate_vector(q_global_imu, self.p_base_in_imu)
        rotation_global_imu = quaternion_to_rotation_matrix(q_global_imu)
        rotation_base_imu = quaternion_to_rotation_matrix(self.q_base_imu)
        identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

        # VIO_ver1' pose covariance is [position_G, attitude_error_I]. Moving
        # the origin through the lever arm couples attitude uncertainty into
        # position, while the fixed I->B rotation maps attitude error to B.
        pose_coupling = _negative_product(
            rotation_global_imu, _skew(self.p_base_in_imu)
        )
        pose_jacobian = _block_jacobian(
            identity, pose_coupling, rotation_base_imu
        )
        pose_covariance = _covariance_transform(
            list(source.pose.covariance), pose_jacobian
        )

        output = Odometry()
        output.header.stamp = source.header.stamp
        # VIO_ver1 is gravity aligned but its yaw/origin are arbitrary. ROS odom
        # is the appropriate continuous local frame; the PX4 bridge marks it as
        # POSE_FRAME_FRD so EKF2 estimates the vision-to-navigation yaw offset.
        output.header.frame_id = "odom"
        output.child_frame_id = "base_link"
        output.pose = copy.deepcopy(source.pose)
        output.pose.pose.position.x = source.pose.pose.position.x + offset_global[0]
        output.pose.pose.position.y = source.pose.pose.position.y + offset_global[1]
        output.pose.pose.position.z = source.pose.pose.position.z + offset_global[2]
        output.pose.pose.orientation.w = q_global_base.w
        output.pose.pose.orientation.x = q_global_base.x
        output.pose.pose.orientation.y = q_global_base.y
        output.pose.pose.orientation.z = q_global_base.z
        output.pose.covariance = pose_covariance
        output.twist = copy.deepcopy(source.twist)
        velocity_imu = (
            float(source.twist.twist.linear.x),
            float(source.twist.twist.linear.y),
            float(source.twist.twist.linear.z),
        )
        omega_imu = (
            float(source.twist.twist.angular.x),
            float(source.twist.twist.angular.y),
            float(source.twist.twist.angular.z),
        )
        lever_velocity_imu = (
            omega_imu[1] * self.p_base_in_imu[2]
            - omega_imu[2] * self.p_base_in_imu[1],
            omega_imu[2] * self.p_base_in_imu[0]
            - omega_imu[0] * self.p_base_in_imu[2],
            omega_imu[0] * self.p_base_in_imu[1]
            - omega_imu[1] * self.p_base_in_imu[0],
        )
        velocity_base = rotate_vector(
            self.q_base_imu,
            tuple(
                velocity_imu[index] + lever_velocity_imu[index]
                for index in range(3)
            ),
        )
        omega_base = rotate_vector(self.q_base_imu, omega_imu)
        output.twist.twist.linear.x = velocity_base[0]
        output.twist.twist.linear.y = velocity_base[1]
        output.twist.twist.linear.z = velocity_base[2]
        output.twist.twist.angular.x = omega_base[0]
        output.twist.twist.angular.y = omega_base[1]
        output.twist.twist.angular.z = omega_base[2]

        # VIO_ver1 publishes [velocity_I, angular_velocity_I]. Propagate the
        # complete covariance while moving both the origin and axes to B.
        twist_coupling = _negative_product(
            rotation_base_imu, _skew(self.p_base_in_imu)
        )
        twist_jacobian = _block_jacobian(
            rotation_base_imu, twist_coupling, rotation_base_imu
        )
        output.twist.covariance = _covariance_transform(
            list(source.twist.covariance), twist_jacobian
        )
        self.publisher.publish(output)
        self.last_stamp_ns = stamp_ns


def main(args=None) -> None:
    """Run the VIO_ver1 odometry adapter."""
    rclpy.init(args=args)
    node = VioOdometryAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
