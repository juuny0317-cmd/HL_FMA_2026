"""Supervised low-output bench test for the installed NUCLEO UART firmware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("hl_ku_core")
    config = LaunchConfiguration("config_file")
    test_config = LaunchConfiguration("route_test_config_file")
    enabled = LaunchConfiguration("drive_enabled")
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
                "drive_enabled",
                default_value="false",
                description="Explicit bench-test enable; first run must stay false",
            ),
            Node(
                package="hl_ku_core",
                executable="safety_supervisor",
                output="screen",
                parameters=[
                    config,
                    test_config,
                    {
                        "drive_enabled": ParameterValue(enabled, value_type=bool),
                        "manual_test_mode": True,
                        "require_perception": False,
                        "require_preflight": False,
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="nucleo_serial_bridge",
                output="screen",
                parameters=[config, test_config],
            ),
            Node(
                package="hl_ku_core",
                executable="lidar_perception",
                output="screen",
                parameters=[config, test_config],
            ),
            Node(
                package="hl_ku_core",
                executable="perception_aggregator",
                output="screen",
                parameters=[config, test_config],
            ),
        ]
    )
