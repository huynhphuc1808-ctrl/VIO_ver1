import math
import time

from nav_msgs.msg import Odometry
import numpy as np
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from .frame_transforms import (
    Quaternion,
    quaternion_to_rotation_matrix,
    rotate_vector,
    transform_orientation_local_flu_to_frd,
    transform_vector_local_flu_to_frd,
)


class RosOdometryToPx4VisualOdometry(Node):
    """Convert local FLU odometry into PX4 arbitrary-heading FRD odometry."""

    def __init__(self) -> None:
        super().__init__("ros_odometry_to_px4_visual_odometry")

        self.declare_parameter("input_topic", "/vio/odometry")
        self.declare_parameter("output_topic", "/fmu/in/vehicle_visual_odometry")
        self.declare_parameter("stale_timeout_s", 0.5)
        self.declare_parameter("future_tolerance_s", 0.1)
        self.declare_parameter("max_position_jump_m", 1.0)
        self.declare_parameter("max_velocity_variance", 1.0)
        self.declare_parameter("allowed_parent_frames", ["map", "odom"])
        self.declare_parameter("expected_child_frame", "base_link")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.stale_timeout_us = int(
            float(self.get_parameter("stale_timeout_s").value) * 1_000_000
        )
        self.future_tolerance_us = int(
            float(self.get_parameter("future_tolerance_s").value) * 1_000_000
        )
        self.max_position_jump_m = float(
            self.get_parameter("max_position_jump_m").value
        )
        self.max_velocity_variance = float(
            self.get_parameter("max_velocity_variance").value
        )
        for value in (self.stale_timeout_us, self.max_position_jump_m,
                      self.max_velocity_variance):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("VIO validation limits must be finite and positive")
        if self.future_tolerance_us < 0:
            raise ValueError("future_tolerance_s must be non-negative")
        self.allowed_parent_frames = {
            str(frame)
            for frame in self.get_parameter("allowed_parent_frames").value
        }
        self.expected_child_frame = str(
            self.get_parameter("expected_child_frame").value
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.visual_odom_pub = self.create_publisher(
            VehicleOdometry,
            self.output_topic,
            px4_qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.input_topic,
            self._on_odometry,
            qos_profile_sensor_data,
        )

        self.last_position_flu: tuple[float, float, float] | None = None
        self.last_velocity_flu: tuple[float, float, float] | None = None
        self.last_sample_us: int | None = None
        self.reset_counter = 0
        self.rejected_samples = 0
        self.discontinuity_latched = False
        self.last_accept_monotonic: float | None = None
        self.last_rejection = ""
        self.status_pub = self.create_publisher(String, "/vio/status", 10)
        self.create_timer(
            0.1, self._publish_status, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

        self.get_logger().info(
            f"Publishing PX4 visual odometry: {self.input_topic} -> "
            f"{self.output_topic}"
        )

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds // 1000)

    def _stamp_us(self, msg: Odometry) -> int:
        stamp = msg.header.stamp
        return int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1000

    def _publish_status(self) -> None:
        status = "UNINITIALIZED"
        if self.discontinuity_latched:
            status = "LOST: unannounced discontinuity; inspect VIO before restart"
        elif self.last_accept_monotonic is not None:
            age = time.monotonic() - self.last_accept_monotonic
            sample_age = self._now_us() - self.last_sample_us
            if age > self.stale_timeout_us / 1e6 or not (
                0 <= sample_age <= self.stale_timeout_us
            ):
                status = "LOST: stale VIO"
            elif self.last_rejection:
                status = f"DEGRADED: {self.last_rejection}"
            else:
                status = "VALID_ODOMETRY"
        elif self.last_rejection:
            status = f"DEGRADED: {self.last_rejection}"
        self.status_pub.publish(String(data=status))

    def _reject(self, reason: str) -> None:
        self.rejected_samples += 1
        self.last_rejection = reason
        if self.rejected_samples == 1 or self.rejected_samples % 20 == 0:
            self.get_logger().warning(
                f"Rejected external odometry sample: {reason}"
            )

    def _validate_odometry(
        self,
        odom: Odometry,
        sample_us: int,
        position_flu: tuple[float, float, float],
    ) -> bool:
        if self.discontinuity_latched:
            return False
        if odom.header.frame_id not in self.allowed_parent_frames:
            self._reject(f"unexpected frame_id={odom.header.frame_id!r}")
            return False
        if odom.child_frame_id != self.expected_child_frame:
            self._reject(
                f"unexpected child_frame_id={odom.child_frame_id!r}"
            )
            return False

        orientation = odom.pose.pose.orientation
        values = [
            *position_flu,
            orientation.w,
            orientation.x,
            orientation.y,
            orientation.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            *odom.pose.covariance,
            *odom.twist.covariance,
        ]
        if not all(math.isfinite(float(value)) for value in values):
            self._reject("non-finite pose, orientation, or covariance")
            return False
        quaternion_norm = math.sqrt(
            orientation.w * orientation.w
            + orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        if abs(quaternion_norm - 1.0) > 0.01:
            self._reject("non-unit quaternion")
            return False

        for values in (odom.pose.covariance, odom.twist.covariance):
            covariance = np.asarray(values).reshape(6, 6)
            if (not np.allclose(covariance, covariance.T, atol=1e-8)
                    or np.linalg.eigvalsh(covariance).min() < -1e-8
                    or np.any(covariance.diagonal()[:3] <= 0)):
                self._reject("invalid or unknown covariance")
                return False
        if any(odom.pose.covariance[index] <= 0 for index in (21, 28, 35)):
            self._reject("unknown orientation covariance")
            return False
        velocity_cov = np.asarray(odom.twist.covariance).reshape(6, 6)[:3, :3]
        if np.linalg.eigvalsh(velocity_cov).max() > self.max_velocity_variance:
            self._reject("velocity uncertainty exceeds VIO diagnostic limit")
            return False

        if sample_us <= 0:
            self._reject("zero sensor timestamp")
            return False

        now_us = self._now_us()
        if now_us <= 0:
            self._reject("ROS clock not initialized")
            return False
        if now_us > 0 and sample_us < now_us - self.stale_timeout_us:
            self._reject("stale timestamp")
            return False
        if now_us > 0 and sample_us > now_us + self.future_tolerance_us:
            self._reject("timestamp is in the future")
            return False
        if self.last_sample_us is not None and sample_us <= self.last_sample_us:
            self._reject("timestamp is not monotonic")
            return False
        if self.last_position_flu is not None:
            # Account for real movement during a transport gap. This is only a
            # measurement discontinuity diagnostic; it never creates odometry.
            predicted = (0.0, 0.0, 0.0)
            if self.last_velocity_flu is not None and self.last_sample_us is not None:
                dt = (sample_us - self.last_sample_us) / 1e6
                predicted = tuple(v * dt for v in self.last_velocity_flu)
            jump = math.sqrt(
                sum(
                    (position_flu[index] - self.last_position_flu[index]
                     - predicted[index]) ** 2
                    for index in range(3)
                )
            )
            if jump > self.max_position_jump_m:
                # nav_msgs/Odometry has no estimator reset metadata. Never
                # turn an outlier into a reset command to EKF2. Quarantine this
                # stream; PX4 handles loss of aiding using its normal failsafe.
                self.discontinuity_latched = True
                self._reject(f"position jump {jump:.3f} m")
                return False
        return True

    def _variance3_from_pose_covariance(
        self,
        covariance: list[float],
    ) -> list[float]:
        # FLU -> FRD changes signs only, so diagonal variances keep their order.
        x_var = float(covariance[0])
        y_var = float(covariance[7])
        z_var = float(covariance[14])
        return [x_var, y_var, z_var]

    def _orientation_variance_from_pose_covariance(
        self,
        covariance: list[float],
    ) -> list[float]:
        roll_var = float(covariance[21])
        pitch_var = float(covariance[28])
        yaw_var = float(covariance[35])
        return [roll_var, pitch_var, yaw_var]

    def _velocity_frd(
        self,
        odom: Odometry,
        q_global_base: Quaternion,
    ) -> tuple[list[float], list[float]]:
        # nav_msgs twist is expressed in child_frame_id. Preserve the full
        # linear covariance while rotating into the arbitrary-heading world.
        linear = odom.twist.twist.linear
        velocity_flu = rotate_vector(
            q_global_base, (float(linear.x), float(linear.y), float(linear.z))
        )
        velocity_frd = transform_vector_local_flu_to_frd(*velocity_flu)
        rotation = np.asarray(quaternion_to_rotation_matrix(q_global_base))
        covariance = np.asarray(odom.twist.covariance).reshape(6, 6)[:3, :3]
        rotated = rotation @ covariance @ rotation.T
        return list(velocity_frd), [float(value) for value in rotated.diagonal()]

    def _on_odometry(self, odom: Odometry) -> None:
        sample_us = self._stamp_us(odom)

        position_flu = (
            float(odom.pose.pose.position.x),
            float(odom.pose.pose.position.y),
            float(odom.pose.pose.position.z),
        )
        if not self._validate_odometry(odom, sample_us, position_flu):
            return
        position_frd = transform_vector_local_flu_to_frd(*position_flu)

        q_ros = Quaternion(
            w=float(odom.pose.pose.orientation.w),
            x=float(odom.pose.pose.orientation.x),
            y=float(odom.pose.pose.orientation.y),
            z=float(odom.pose.pose.orientation.z),
        ).normalized()
        q_px4 = transform_orientation_local_flu_to_frd(q_ros)

        velocity_frd, velocity_variance = self._velocity_frd(
            odom, q_ros
        )
        self.last_position_flu = position_flu
        self.last_sample_us = sample_us
        linear = odom.twist.twist.linear
        self.last_velocity_flu = rotate_vector(q_ros, (linear.x, linear.y, linear.z))

        msg = VehicleOdometry()
        now_us = self._now_us()
        msg.timestamp = now_us
        msg.timestamp_sample = sample_us
        msg.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        msg.position = [float(v) for v in position_frd]
        msg.q = [float(q_px4.w), float(q_px4.x), float(q_px4.y), float(q_px4.z)]
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_FRD
        msg.velocity = velocity_frd
        angular = odom.twist.twist.angular
        angular_base_flu = (
            float(angular.x),
            float(angular.y),
            float(angular.z),
        )
        if all(math.isfinite(value) for value in angular_base_flu):
            msg.angular_velocity = [
                float(value)
                for value in transform_vector_local_flu_to_frd(
                    *angular_base_flu
                )
            ]
        else:
            msg.angular_velocity = [float("nan")] * 3

        pose_cov = list(odom.pose.covariance)
        msg.position_variance = self._variance3_from_pose_covariance(
            pose_cov,
        )
        msg.orientation_variance = self._orientation_variance_from_pose_covariance(
            pose_cov
        )
        msg.velocity_variance = velocity_variance
        msg.reset_counter = self.reset_counter
        # Binary adapter validity, not a fabricated VIO_ver1 feature score.
        msg.quality = 1

        self.visual_odom_pub.publish(msg)
        self.last_accept_monotonic = time.monotonic()
        self.last_rejection = ""


def main(args=None) -> None:
    """Run the ROS odometry to PX4 visual odometry bridge."""
    rclpy.init(args=args)
    node = RosOdometryToPx4VisualOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
