"""Start the safety and MCU bridge for supervised keyboard calibration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare("hl_ku_core"), "config", "system.yaml"]
    )
    drive_enabled = ParameterValue(
        LaunchConfiguration("drive_enabled"), value_type=bool
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description="Explicitly enables low-duty keyboard calibration",
            ),
            Node(
                package="hl_ku_core",
                executable="safety_supervisor",
                output="screen",
                parameters=[
                    config,
                    {
                        "drive_enabled": drive_enabled,
                        "manual_test_mode": True,
                        "require_perception": False,
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="mcu_udp_bridge",
                output="screen",
                parameters=[config],
            ),
        ]
    )
