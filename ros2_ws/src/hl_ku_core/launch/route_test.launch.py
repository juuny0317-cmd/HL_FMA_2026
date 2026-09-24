"""Low-speed global-route test using the vehicle's current NUCLEO UART setup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("hl_ku_core")
    default_config = PathJoinSubstitution([package_share, "config", "system.yaml"])
    default_test_config = PathJoinSubstitution(
        [package_share, "config", "route_test.yaml"]
    )
    default_calibration = PathJoinSubstitution(
        [package_share, "config", "calibration_record.yaml"]
    )
    default_route = PathJoinSubstitution([package_share, "routes", "course_template.csv"])

    config = LaunchConfiguration("config_file")
    test_config = LaunchConfiguration("route_test_config_file")
    calibration = LaunchConfiguration("calibration_file")
    route = LaunchConfiguration("route_file")
    drive_enabled = LaunchConfiguration("drive_enabled")
    route_calibrated = LaunchConfiguration("route_calibrated")
    use_lidar = LaunchConfiguration("use_lidar_perception")
    use_local_detour = LaunchConfiguration("enable_local_detour_steering")
    common = {"output": "screen", "parameters": [config, test_config]}

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument(
                "route_test_config_file", default_value=default_test_config
            ),
            DeclareLaunchArgument("calibration_file", default_value=default_calibration),
            DeclareLaunchArgument(
                "route_file",
                default_value=default_route,
                description="Measured ENU route; the included template is always rejected",
            ),
            DeclareLaunchArgument("use_lidar_perception", default_value="true"),
            DeclareLaunchArgument(
                "enable_local_detour_steering",
                default_value="false",
                description="Opt in to live detour steering; requires map-frame obstacles",
            ),
            DeclareLaunchArgument(
                "route_calibrated",
                default_value="false",
                description="Explicit confirmation that the launched route was checked",
            ),
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description="Final actuator enable; leave false through all bench tests",
            ),
            Node(
                package="hl_ku_core",
                executable="preflight_check",
                output="screen",
                parameters=[
                    {
                        "profile": "global_route_test",
                        "system_config_file": ParameterValue(config, value_type=str),
                        "override_config_file": ParameterValue(
                            test_config, value_type=str
                        ),
                        "calibration_file": ParameterValue(
                            calibration, value_type=str
                        ),
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
                executable="lidar_perception",
                condition=IfCondition(use_lidar),
                **common,
            ),
            Node(
                package="hl_ku_core",
                executable="scan_obstacle_mapper",
                condition=IfCondition(use_lidar),
                **common,
            ),
            Node(package="hl_ku_core", executable="perception_aggregator", **common),
            Node(
                package="hl_ku_core",
                executable="path_tracker",
                output="screen",
                parameters=[
                    config,
                    test_config,
                    {
                        "route_file": route,
                        "route_calibrated": ParameterValue(
                            route_calibrated, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="local_detour",
                condition=IfCondition(use_local_detour),
                output="screen",
                parameters=[
                    config,
                    {
                        "route_file": route,
                        "preview_mode": False,
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="mission_manager",
                output="screen",
                parameters=[
                    config,
                    test_config,
                    {
                        "enable_local_detour_steering": ParameterValue(
                            use_local_detour, value_type=bool
                        ),
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
                    test_config,
                    {
                        "drive_enabled": ParameterValue(
                            drive_enabled, value_type=bool
                        ),
                        "route_calibrated": ParameterValue(
                            route_calibrated, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="nucleo_serial_bridge",
                **common,
            ),
        ]
    )
