"""Record an ENU route without starting any actuator bridge."""

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
    ntrip_config = LaunchConfiguration("ntrip_config_file")
    output_file = LaunchConfiguration("output_file")
    common = {
        "output": "screen",
        "parameters": [config, test_config, ntrip_config],
    }
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
                "ntrip_config_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "ntrip_private.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "output_file",
                description="Absolute output path for the measured route CSV",
            ),
            Node(package="hl_ku_core", executable="um982_serial", **common),
            Node(package="hl_ku_core", executable="ntrip_client", **common),
            Node(package="hl_ku_core", executable="gnss_localizer", **common),
            Node(
                package="hl_ku_core",
                executable="route_recorder",
                output="screen",
                parameters=[
                    config,
                    test_config,
                    ntrip_config,
                    {
                        "output_file": ParameterValue(
                            output_file, value_type=str
                        )
                    },
                ],
            ),
        ]
    )
