"""Read-only PX4 telemetry bridge for ROS visualization."""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from .frame_transforms import (
    Quaternion,
    transform_orientation_px4_to_ros,
    transform_vector_ned_to_enu,
)


def parse_model_pose(value: str) -> tuple[float, float, float]:
    """Parse x,y or x,y,z PX4 spawn pose strings."""
    parts = [part.strip() for part in str(value).split(",") if part.strip()]
    values = [float(part) for part in parts]
    if len(values) == 2:
        return values[0], values[1], 0.0
    if len(values) >= 3:
        return values[0], values[1], values[2]
    raise ValueError(f"model_pose must contain at least x,y: {value!r}")


class ExternalBridge(Node):
    """Convert PX4 NED/FRD odometry to ROS ENU/FLU without commanding PX4."""

    def __init__(self) -> None:
        super().__init__("external_bridge")
        self.declare_parameter("model_pose", "0,0,0")
        self.declare_parameter("viz_enabled", False)
        self.spawn_offset = parse_model_pose(
            str(self.get_parameter("model_pose").value)
        )
        self.viz_enabled = bool(self.get_parameter("viz_enabled").value)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self._on_vehicle_odometry,
            px4_qos,
        )
        self.odom_pub = self.create_publisher(Odometry, "/odometry", 10)
        self.path_pub = self.create_publisher(Path, "/odometry_path", 10)
        self.path = Path()
        self.path.header.frame_id = "map"
        self.get_logger().info(
            "Read-only PX4 telemetry bridge ready; PX4 Navigator owns mission"
        )

    def _on_vehicle_odometry(self, msg: VehicleOdometry) -> None:
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            self.get_logger().warning(
                f"Ignoring unsupported PX4 pose_frame={msg.pose_frame}"
            )
            return
        values = [*msg.position, *msg.q]
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().warning("Ignoring non-finite PX4 odometry")
            return

        x_enu, y_enu, z_enu = transform_vector_ned_to_enu(*msg.position)
        q_ros = transform_orientation_px4_to_ros(
            Quaternion(
                w=float(msg.q[0]),
                x=float(msg.q[1]),
                y=float(msg.q[2]),
                z=float(msg.q[3]),
            )
        )
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x_enu + self.spawn_offset[0]
        odom.pose.pose.position.y = y_enu + self.spawn_offset[1]
        odom.pose.pose.position.z = z_enu + self.spawn_offset[2]
        odom.pose.pose.orientation.w = q_ros.w
        odom.pose.pose.orientation.x = q_ros.x
        odom.pose.pose.orientation.y = q_ros.y
        odom.pose.pose.orientation.z = q_ros.z
        if (
            msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED
            and all(math.isfinite(float(value)) for value in msg.velocity)
        ):
            velocity = transform_vector_ned_to_enu(*msg.velocity)
            odom.twist.twist.linear.x = velocity[0]
            odom.twist.twist.linear.y = velocity[1]
            odom.twist.twist.linear.z = velocity[2]
        self.odom_pub.publish(odom)

        if not self.viz_enabled:
            return
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path.header.stamp = odom.header.stamp
        self.path.poses.append(pose)
        self.path.poses = self.path.poses[-2000:]
        self.path_pub.publish(self.path)


def main(args=None) -> None:
    """Run the read-only PX4 telemetry bridge."""
    rclpy.init(args=args)
    node = ExternalBridge()
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
