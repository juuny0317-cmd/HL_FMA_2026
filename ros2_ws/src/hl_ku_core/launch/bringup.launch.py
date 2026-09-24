"""Bring up the fail-closed HL_KU driving foundation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("hl_ku_core")
    default_config = PathJoinSubstitution([package_share, "config", "system.yaml"])
    default_calibration = PathJoinSubstitution(
        [package_share, "config", "calibration_record.yaml"]
    )
    default_route = PathJoinSubstitution([package_share, "routes", "course_template.csv"])
    config = LaunchConfiguration("config_file")
    calibration = LaunchConfiguration("calibration_file")
    route = LaunchConfiguration("route_file")
    drive_enabled = LaunchConfiguration("drive_enabled")
    route_calibrated = LaunchConfiguration("route_calibrated")
    use_camera_perception = LaunchConfiguration("use_baseline_camera_perception")
    use_lidar_perception = LaunchConfiguration("use_baseline_lidar_perception")
    use_local_detour = LaunchConfiguration("enable_local_detour_steering")
    drive_enabled_bool = ParameterValue(drive_enabled, value_type=bool)
    route_calibrated_bool = ParameterValue(route_calibrated, value_type=bool)
    common = {"output": "screen", "parameters": [config]}
    nodes = [
        Node(
            package="hl_ku_core",
            executable="preflight_check",
            output="screen",
            parameters=[
                {
                    "system_config_file": ParameterValue(config, value_type=str),
                    "calibration_file": ParameterValue(calibration, value_type=str),
                    "route_file": ParameterValue(route, value_type=str),
                }
            ],
        ),
        Node(package="hl_ku_core", executable="um982_serial", **common),
        Node(package="hl_ku_core", executable="ntrip_client", **common),
        Node(package="hl_ku_core", executable="gnss_localizer", **common),
        Node(package="hl_ku_core", executable="velocity_selector", **common),
        Node(
            package="hl_ku_core",
            executable="camera_perception",
            condition=IfCondition(use_camera_perception),
            **common,
        ),
        Node(
            package="hl_ku_core",
            executable="lidar_perception",
            condition=IfCondition(use_lidar_perception),
            **common,
        ),
        Node(package="hl_ku_core", executable="perception_aggregator", **common),
        Node(
            package="hl_ku_core",
            executable="path_tracker",
            output="screen",
            parameters=[
                config,
                {"route_file": route, "route_calibrated": route_calibrated_bool},
            ],
        ),
        Node(
            package="hl_ku_core",
            executable="local_detour",
            condition=IfCondition(use_local_detour),
            output="screen",
            parameters=[config, {"route_file": route, "preview_mode": False}],
        ),
        Node(
            package="hl_ku_core",
            executable="mission_manager",
            output="screen",
            parameters=[
                config,
                {
                    "enable_local_detour_steering": ParameterValue(
                        use_local_detour, value_type=bool
                    )
                },
            ],
        ),
        Node(package="hl_ku_core", executable="vehicle_controller", **common),
        Node(
            package="hl_ku_core",
            executable="safety_supervisor",
            output="screen",
            parameters=[
                config,
                {
                    "drive_enabled": drive_enabled_bool,
                    "route_calibrated": route_calibrated_bool,
                },
            ],
        ),
        Node(package="hl_ku_core", executable="mcu_udp_bridge", **common),
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Vehicle-specific ROS parameter YAML",
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=default_calibration,
                description="Measured commissioning and acceptance record",
            ),
            DeclareLaunchArgument(
                "route_file",
                default_value=default_route,
                description="Measured course CSV; template routes never pass preflight",
            ),
            DeclareLaunchArgument(
                "use_baseline_camera_perception",
                default_value="true",
                description="Disable when a validated external camera perception stack is used",
            ),
            DeclareLaunchArgument(
                "use_baseline_lidar_perception",
                default_value="true",
                description="Disable when a validated external LiDAR perception stack is used",
            ),
            DeclareLaunchArgument(
                "enable_local_detour_steering",
                default_value="false",
                description="Opt in to live detour steering; requires map-frame obstacles",
            ),
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description="Final explicit actuator enable; keep false for bench tests",
            ),
            DeclareLaunchArgument(
                "route_calibrated",
                default_value="false",
                description="Confirms that datum, route and vehicle geometry were measured",
            ),
            *nodes,
        ]
    )
