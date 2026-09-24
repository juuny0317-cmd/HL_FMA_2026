"""Bring up the UM982 and private NTRIP connection without actuator nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("hl_ku_core")
    receiver_config = LaunchConfiguration("receiver_config_file")
    ntrip_config = LaunchConfiguration("ntrip_config_file")
    common = {
        "output": "screen",
        "parameters": [receiver_config, ntrip_config],
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "receiver_config_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "rtk_field_test.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_config_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "ntrip_private.yaml"]
                ),
            ),
            Node(package="hl_ku_core", executable="um982_serial", **common),
            Node(package="hl_ku_core", executable="ntrip_client", **common),
        ]
    )
