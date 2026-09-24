"""Start the RPLIDAR A3 driver and a matching RViz view."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("hl_ku_core")
    serial_port = LaunchConfiguration("serial_port")
    lidar_yaw_rad = LaunchConfiguration("lidar_yaw_rad")
    perception_enabled = LaunchConfiguration("perception")
    obstacle_trigger_m = LaunchConfiguration("obstacle_trigger_m")
    rviz_enabled = LaunchConfiguration("rviz")
    rviz_config = PathJoinSubstitution([share, "rviz", "rplidar_a3.rviz"])
    system_config = PathJoinSubstitution([share, "config", "system.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/tmp/nucleo-lidar"),
            # A 3 m target placed on the physical vehicle heading measured the
            # A3 scan origin at +132.68 degrees from vehicle +X.
            DeclareLaunchArgument("lidar_yaw_rad", default_value="-2.3157"),
            DeclareLaunchArgument("perception", default_value="true"),
            DeclareLaunchArgument("obstacle_trigger_m", default_value="8.0"),
            DeclareLaunchArgument("rviz", default_value="true"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="vehicle_heading_display_tf",
                output="screen",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--yaw", "1.5708",
                    "--pitch", "0", "--roll", "0",
                    "--frame-id", "lidar_aligned",
                    "--child-frame-id", "vehicle_heading",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="rplidar_mount_tf",
                output="screen",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--yaw", lidar_yaw_rad,
                    "--pitch", "0", "--roll", "0",
                    "--frame-id", "vehicle_heading",
                    "--child-frame-id", "laser",
                ],
            ),
            Node(
                package="rplidar_ros",
                executable="rplidar_node",
                name="rplidar_a3",
                output="screen",
                parameters=[
                    {
                        "channel_type": "serial",
                        "serial_port": ParameterValue(serial_port, value_type=str),
                        "serial_baudrate": 256000,
                        "frame_id": "laser",
                        "inverted": False,
                        "angle_compensate": True,
                        "scan_mode": "Sensitivity",
                        "topic_name": "scan",
                    }
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="lidar_perception",
                name="lidar_perception",
                output="screen",
                condition=IfCondition(perception_enabled),
                parameters=[
                    system_config,
                    {
                        "obstacle_trigger_m": ParameterValue(
                            obstacle_trigger_m, value_type=float
                        )
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rplidar_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(rviz_enabled),
            ),
        ]
    )
