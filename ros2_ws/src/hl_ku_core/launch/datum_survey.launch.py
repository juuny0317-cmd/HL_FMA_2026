"""Start only GNSS/NTRIP and the fixed-datum survey helper."""

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
    duration = LaunchConfiguration("duration_sec")
    samples = LaunchConfiguration("minimum_samples")
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
            DeclareLaunchArgument("duration_sec", default_value="120.0"),
            DeclareLaunchArgument("minimum_samples", default_value="100"),
            Node(package="hl_ku_core", executable="um982_serial", **common),
            Node(package="hl_ku_core", executable="ntrip_client", **common),
            Node(
                package="hl_ku_core",
                executable="gnss_survey",
                output="screen",
                parameters=[
                    {
                        "duration_sec": ParameterValue(duration, value_type=float),
                        "minimum_samples": ParameterValue(samples, value_type=int),
                    }
                ],
            ),
        ]
    )
