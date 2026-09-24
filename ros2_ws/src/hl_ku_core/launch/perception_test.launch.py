"""Run only the replaceable perception baseline; no actuator nodes are started."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("hl_ku_core")
    config = PathJoinSubstitution([package_share, "config", "system.yaml"])
    common = {"output": "screen", "parameters": [config]}
    return LaunchDescription(
        [
            Node(package="hl_ku_core", executable="camera_perception", **common),
            Node(package="hl_ku_core", executable="lidar_perception", **common),
            Node(package="hl_ku_core", executable="perception_aggregator", **common),
        ]
    )
