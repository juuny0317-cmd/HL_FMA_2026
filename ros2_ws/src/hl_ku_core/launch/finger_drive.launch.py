"""Wheels-up full-range finger-drive support with deadman and watchdog."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("hl_ku_core")
    config = LaunchConfiguration("config_file")
    route_test_config = LaunchConfiguration("route_test_config_file")
    finger_config = LaunchConfiguration("finger_drive_config_file")
    enabled = LaunchConfiguration("drive_enabled")
    maximum_drive_duty = LaunchConfiguration("maximum_drive_duty")
    maximum_drive_command = LaunchConfiguration("maximum_drive_command")
    parameter_files = [config, route_test_config, finger_config]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution([share, "config", "system.yaml"]),
            ),
            DeclareLaunchArgument(
                "route_test_config_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "route_test.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "finger_drive_config_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "finger_drive.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description="Explicit wheels-up finger-drive enable",
            ),
            DeclareLaunchArgument(
                "maximum_drive_duty",
                default_value="1.0",
                description="Manual normalized duty ceiling",
            ),
            DeclareLaunchArgument(
                "maximum_drive_command",
                default_value="100",
                description="NUCLEO manual command ceiling",
            ),
            Node(
                package="hl_ku_core",
                executable="safety_supervisor",
                output="screen",
                parameters=[
                    *parameter_files,
                    {
                        "drive_enabled": ParameterValue(enabled, value_type=bool),
                        "manual_test_mode": True,
                        "manual_maximum_drive_duty": ParameterValue(
                            maximum_drive_duty, value_type=float
                        ),
                        "require_perception": False,
                        "require_preflight": False,
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="nucleo_serial_bridge",
                output="screen",
                parameters=[
                    *parameter_files,
                    {
                        "maximum_drive_command": ParameterValue(
                            maximum_drive_command, value_type=int
                        )
                    },
                ],
            ),
        ]
    )
