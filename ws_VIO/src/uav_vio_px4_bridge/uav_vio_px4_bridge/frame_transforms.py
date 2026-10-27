"""Central ENU/FLU and NED/FRD frame conversions."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Quaternion:
    """Minimal immutable quaternion used by the bridge transforms."""

    w: float
    x: float
    y: float
    z: float

    def normalized(self) -> "Quaternion":
        """Return a unit quaternion, or identity for a zero norm."""
        n = math.sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
        if n == 0.0:
            return Quaternion(1.0, 0.0, 0.0, 0.0)
        return Quaternion(self.w / n, self.x / n, self.y / n, self.z / n)

    def __mul__(self, other: "Quaternion") -> "Quaternion":
        """Return the Hamilton product with another quaternion."""
        # Hamilton product.
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z
        return Quaternion(
            w=w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            x=w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            y=w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            z=w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Build a quaternion using ZYX yaw-pitch-roll composition."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # ZYX order.
    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )


def quaternion_get_yaw(q: Quaternion) -> float:
    """Extract yaw from a quaternion."""
    q = q.normalized()
    q0, q1, q2, q3 = q.w, q.x, q.y, q.z
    return math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))


def quaternion_conjugate(q: Quaternion) -> Quaternion:
    """Return the quaternion conjugate."""
    return Quaternion(w=q.w, x=-q.x, y=-q.y, z=-q.z)


def rotate_vector(
    q: Quaternion, vector: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Rotate a 3-vector by a Hamilton quaternion."""
    unit = q.normalized()
    pure = Quaternion(0.0, *vector)
    rotated = unit * pure * quaternion_conjugate(unit)
    return rotated.x, rotated.y, rotated.z


def quaternion_to_rotation_matrix(
    q: Quaternion,
) -> tuple[tuple[float, float, float], ...]:
    """Return the active 3-D rotation matrix represented by a quaternion."""
    unit = q.normalized()
    w, x, y, z = unit.w, unit.x, unit.y, unit.z
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


# Static quaternion (see px4_ros_com/include/px4_ros_com/frame_transforms.h)
NED_ENU_Q = quaternion_from_euler(math.pi, 0.0, math.pi / 2.0)
AIRCRAFT_BASELINK_Q = quaternion_from_euler(math.pi, 0.0, 0.0)


def transform_orientation_enu_to_ned(q_enu: Quaternion) -> Quaternion:
    """Transform an ENU orientation into NED."""
    return (NED_ENU_Q * q_enu).normalized()


def transform_orientation_ned_to_enu(q_ned: Quaternion) -> Quaternion:
    """Transform a NED orientation into ENU."""
    return (quaternion_conjugate(NED_ENU_Q) * q_ned).normalized()


def transform_orientation_baselink_to_aircraft(q_baselink: Quaternion) -> Quaternion:
    """Transform a ROS base_link orientation into PX4 aircraft FRD."""
    return (q_baselink * AIRCRAFT_BASELINK_Q).normalized()


def transform_orientation_aircraft_to_baselink(q_aircraft: Quaternion) -> Quaternion:
    """Transform a PX4 aircraft FRD orientation into ROS base_link."""
    return (q_aircraft * AIRCRAFT_BASELINK_Q).normalized()


def transform_orientation_ros_to_px4(q_ros: Quaternion) -> Quaternion:
    """Convert a ROS base_link/ENU orientation to PX4 aircraft/NED."""
    return transform_orientation_baselink_to_aircraft(
        transform_orientation_enu_to_ned(q_ros)
    )


def transform_orientation_px4_to_ros(q_px4: Quaternion) -> Quaternion:
    """Convert a PX4 aircraft/NED orientation to ROS base_link/ENU."""
    return transform_orientation_aircraft_to_baselink(
        transform_orientation_ned_to_enu(q_px4)
    )


def transform_orientation_local_flu_to_frd(q_flu: Quaternion) -> Quaternion:
    """Convert an arbitrary-yaw local FLU pose into local FRD coordinates."""
    return (
        AIRCRAFT_BASELINK_Q * q_flu * AIRCRAFT_BASELINK_Q
    ).normalized()


def transform_vector_local_flu_to_frd(
    x_flu: float,
    y_flu: float,
    z_flu: float,
) -> tuple[float, float, float]:
    """Convert a local x-forward/y-left/z-up vector to local FRD."""
    return float(x_flu), float(-y_flu), float(-z_flu)


def transform_vector_enu_to_ned(
    x_enu: float,
    y_enu: float,
    z_enu: float,
) -> tuple[float, float, float]:
    """Convert an ENU vector to NED by swapping X/Y and negating Z."""
    return float(y_enu), float(x_enu), float(-z_enu)


def transform_vector_ned_to_enu(
    x_ned: float,
    y_ned: float,
    z_ned: float,
) -> tuple[float, float, float]:
    """Convert a NED vector to ENU by swapping X/Y and negating Z."""
    return float(y_ned), float(x_ned), float(-z_ned)
