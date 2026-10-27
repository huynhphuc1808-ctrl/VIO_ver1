from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch both directions of the external-vision/PX4 bridge."""
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    model_pose = LaunchConfiguration("model_pose")
    viz_enabled = LaunchConfiguration("viz_enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic",
                default_value="/vio/odometry",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/fmu/in/vehicle_visual_odometry",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("model_pose", default_value="0,0,0"),
            DeclareLaunchArgument("viz_enabled", default_value="false"),
            Node(
                package="uav_vio_px4_bridge",
                executable="odom_to_px4_visual_odometry",
                name="odom_to_px4_visual_odometry",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "output_topic": output_topic,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="uav_vio_px4_bridge",
                executable="external_bridge",
                name="external_bridge",
                output="screen",
                parameters=[
                    {
                        "model_pose": model_pose,
                        "viz_enabled": ParameterValue(viz_enabled, value_type=bool),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),
        ]
    )
