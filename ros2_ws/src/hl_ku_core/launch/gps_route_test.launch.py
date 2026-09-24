"""GPS/RTK-only measured-route driving with no camera or LiDAR nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("hl_ku_core")
    default_config = PathJoinSubstitution([share, "config", "system.yaml"])
    default_gps_config = PathJoinSubstitution(
        [share, "config", "gps_only_route.yaml"]
    )
    default_ntrip_config = PathJoinSubstitution(
        [share, "config", "ntrip_private.yaml"]
    )
    default_calibration = PathJoinSubstitution(
        [share, "config", "calibration_record.yaml"]
    )
    default_route = PathJoinSubstitution([share, "routes", "course_06_vehicle.csv"])
    default_rviz_config = PathJoinSubstitution(
        [share, "rviz", "gps_route_alignment.rviz"]
    )

    config = LaunchConfiguration("config_file")
    gps_config = LaunchConfiguration("gps_config_file")
    ntrip_config = LaunchConfiguration("ntrip_config_file")
    calibration = LaunchConfiguration("calibration_file")
    route = LaunchConfiguration("route_file")
    drive_enabled = LaunchConfiguration("drive_enabled")
    route_calibrated = LaunchConfiguration("route_calibrated")
    actuator_bridge_enabled = LaunchConfiguration("actuator_bridge_enabled")
    require_preflight = LaunchConfiguration("require_preflight")
    relaxed_route_test_mode = LaunchConfiguration("relaxed_route_test_mode")
    require_velocity_feedback = LaunchConfiguration("require_velocity_feedback")
    mission_speed_mps = LaunchConfiguration("mission_speed_mps")
    maximum_drive_duty = LaunchConfiguration("maximum_drive_duty")
    minimum_forward_duty = LaunchConfiguration("minimum_forward_duty")
    fixed_drive_pwm = LaunchConfiguration("fixed_drive_pwm")
    maximum_drive_command = LaunchConfiguration("maximum_drive_command")
    allow_reverse = LaunchConfiguration("allow_reverse")
    use_lidar_pipeline = LaunchConfiguration("use_lidar_pipeline")
    start_lidar_driver = LaunchConfiguration("start_lidar_driver")
    use_local_detour_planner = LaunchConfiguration("use_local_detour_planner")
    lidar_geometry_verified = LaunchConfiguration("lidar_geometry_verified")
    enable_local_detour_steering = LaunchConfiguration(
        "enable_local_detour_steering"
    )
    enable_parking_selection = LaunchConfiguration("enable_parking_selection")
    use_camera_perception = LaunchConfiguration("use_camera_perception")
    use_yolo_missions = LaunchConfiguration("use_yolo_missions")
    integrated_yolo_model = LaunchConfiguration("integrated_yolo_model")
    endpoint_yolo_model = LaunchConfiguration("endpoint_yolo_model")
    hill_hold_enabled = LaunchConfiguration("hill_hold_enabled")
    hill_hold_duty = LaunchConfiguration("hill_hold_duty")
    hill_approach_pwm = LaunchConfiguration("hill_approach_pwm")
    hill_post_stop_pwm = LaunchConfiguration("hill_post_stop_pwm")
    parking_source_route_file = LaunchConfiguration("parking_source_route_file")
    parking_source_variant_id = LaunchConfiguration("parking_source_variant_id")
    parking_source_start_s_m = LaunchConfiguration("parking_source_start_s_m")
    parking_source_length_m = LaunchConfiguration("parking_source_length_m")
    direction_change_hold_sec = LaunchConfiguration(
        "direction_change_hold_sec"
    )
    rviz_enabled = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    common = {
        "output": "screen",
        "parameters": [config, gps_config, ntrip_config],
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("gps_config_file", default_value=default_gps_config),
            DeclareLaunchArgument(
                "ntrip_config_file", default_value=default_ntrip_config
            ),
            DeclareLaunchArgument("calibration_file", default_value=default_calibration),
            DeclareLaunchArgument(
                "route_file",
                default_value=default_route,
                description="Measured ENU route; the template cannot enable driving",
            ),
            DeclareLaunchArgument(
                "route_calibrated",
                default_value="false",
                description="Explicit confirmation of datum and route alignment",
            ),
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description="Final actuator enable for a closed outdoor course",
            ),
            DeclareLaunchArgument(
                "actuator_bridge_enabled",
                default_value="true",
                description="Open the NUCLEO serial port; false gives a GNSS/path-only preview",
            ),
            DeclareLaunchArgument(
                "require_preflight",
                default_value="true",
                description=(
                    "Keep true normally; false permits an explicitly supervised "
                    "field trial while retaining live safety gates"
                ),
            ),
            DeclareLaunchArgument(
                "relaxed_route_test_mode",
                default_value="false",
                description="Remove GNSS quality thresholds for early route commissioning",
            ),
            DeclareLaunchArgument(
                "require_velocity_feedback",
                default_value="true",
                description="Require fresh measured velocity before drive output",
            ),
            DeclareLaunchArgument(
                "mission_speed_mps",
                default_value="0.30",
                description=(
                    "Route cruise-speed override and mission speed limit; "
                    "zero-speed FINISH points remain stopped"
                ),
            ),
            DeclareLaunchArgument(
                "maximum_drive_duty",
                default_value="0.30",
                description=(
                    "Maximum normalized drive output; with scale 100, "
                    "0.30 maps to NUCLEO command 30"
                ),
            ),
            DeclareLaunchArgument(
                "minimum_forward_duty",
                default_value="0.30",
                description="Minimum output while forward motion is requested",
            ),
            DeclareLaunchArgument(
                "fixed_drive_pwm",
                default_value="-1",
                description="Direct signed drive PWM magnitude 0..100; -1 keeps speed control",
            ),
            DeclareLaunchArgument(
                "maximum_drive_command",
                default_value="30",
                description="Final integer drive-command clamp in the NUCLEO serial bridge",
            ),
            DeclareLaunchArgument(
                "allow_reverse",
                default_value="false",
                description="Permit signed reverse output for authored parking routes",
            ),
            DeclareLaunchArgument(
                "use_lidar_pipeline",
                default_value="false",
                description=(
                    "Run LiDAR detection, scan-to-map projection and perception summary"
                ),
            ),
            DeclareLaunchArgument(
                "start_lidar_driver",
                default_value="false",
                description=(
                    "Start the RPLIDAR A3 driver on the NUCLEO mux lidar port"
                ),
            ),
            DeclareLaunchArgument(
                "use_local_detour_planner",
                default_value="false",
                description="Publish a map-frame local detour candidate for monitoring",
            ),
            DeclareLaunchArgument(
                "lidar_geometry_verified",
                default_value="false",
                description="Use the measured LiDAR mount for scan-to-map and live detour planning",
            ),
            DeclareLaunchArgument(
                "enable_local_detour_steering",
                default_value="false",
                description=(
                    "Feed verified live detour candidates into mission steering"
                ),
            ),
            DeclareLaunchArgument(
                "enable_parking_selection",
                default_value="false",
                description="Select T1/T2 and P1/P2 from live map-frame LiDAR clusters",
            ),
            DeclareLaunchArgument(
                "use_camera_perception",
                default_value="false",
                description="Run the camera cue baseline and feed its outputs to missions",
            ),
            DeclareLaunchArgument(
                "use_yolo_missions",
                default_value="false",
                description=(
                    "Run section-gated traffic/pedestrian YOLO and the "
                    "endpoint_2class_v3 F1/F2 selector"
                ),
            ),
            DeclareLaunchArgument(
                "integrated_yolo_model",
                default_value=PathJoinSubstitution(
                    [share, "weights", "integrated_7class", "best.pt"]
                ),
            ),
            DeclareLaunchArgument(
                "endpoint_yolo_model",
                default_value=PathJoinSubstitution(
                    [share, "weights", "endpoint_2class_v3", "best.pt"]
                ),
            ),
            DeclareLaunchArgument(
                "hill_hold_enabled",
                default_value="false",
                description="Apply the configured positive duty only during HILL_HOLD",
            ),
            DeclareLaunchArgument(
                "hill_hold_duty",
                default_value="0.0",
                description="Normalized forward duty used during the timed hill hold",
            ),
            DeclareLaunchArgument(
                "hill_approach_pwm",
                default_value="-1",
                description=(
                    "Direct PWM from HILL zone entry to HILL_STOP; "
                    "-1 keeps fixed_drive_pwm"
                ),
            ),
            DeclareLaunchArgument(
                "hill_post_stop_pwm",
                default_value="-1",
                description=(
                    "Direct PWM after HILL_HOLD until the HILL zone ends; "
                    "-1 keeps fixed_drive_pwm"
                ),
            ),
            DeclareLaunchArgument("parking_source_route_file", default_value=""),
            DeclareLaunchArgument("parking_source_variant_id", default_value=""),
            DeclareLaunchArgument("parking_source_start_s_m", default_value="0.0"),
            DeclareLaunchArgument("parking_source_length_m", default_value="0.0"),
            DeclareLaunchArgument(
                "direction_change_hold_sec",
                default_value="0.75",
                description="Full-stop hold before an authored forward/reverse change",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Open the GNSS route-alignment RViz view",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz configuration for route alignment",
            ),
            Node(
                package="hl_ku_core",
                executable="preflight_check",
                output="screen",
                condition=IfCondition(require_preflight),
                parameters=[
                    {
                        "profile": "gps_only_route_test",
                        "system_config_file": ParameterValue(config, value_type=str),
                        "override_config_file": ParameterValue(
                            gps_config, value_type=str
                        ),
                        "ntrip_config_file": ParameterValue(
                            ntrip_config, value_type=str
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
                package="rplidar_ros",
                executable="rplidar_node",
                name="rplidar_a3",
                output="screen",
                condition=IfCondition(start_lidar_driver),
                parameters=[
                    {
                        "channel_type": "serial",
                        "serial_port": "/tmp/nucleo-lidar",
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
                condition=IfCondition(use_lidar_pipeline),
                **common,
            ),
            Node(
                package="hl_ku_core",
                executable="scan_obstacle_mapper",
                condition=IfCondition(use_lidar_pipeline),
                output="screen",
                parameters=[
                    config,
                    gps_config,
                    ntrip_config,
                    {
                        "geometry_verified": ParameterValue(
                            lidar_geometry_verified, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="perception_aggregator",
                # Keep monitor-only LiDAR data out of mission steering.  The
                # aggregator is connected to the mission only with the final
                # explicit detour-steering opt-in.
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            enable_local_detour_steering,
                            "' == 'true' or '",
                            use_camera_perception,
                            "' == 'true' or '",
                            use_yolo_missions,
                            "' == 'true'",
                        ]
                    )
                ),
                **common,
            ),
            Node(
                package="hl_ku_core",
                executable="camera_perception",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            use_camera_perception,
                            "' == 'true' or '",
                            use_yolo_missions,
                            "' == 'true'",
                        ]
                    )
                ),
                parameters=[
                    config,
                    gps_config,
                    ntrip_config,
                    {
                        "publish_traffic_light": ParameterValue(
                            PythonExpression(
                                ["'", use_yolo_missions, "' != 'true'"]
                            ),
                            value_type=bool,
                        )
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="yolo_perception",
                output="screen",
                condition=IfCondition(use_yolo_missions),
                parameters=[
                    config,
                    {
                        "model_path": ParameterValue(
                            integrated_yolo_model, value_type=str
                        )
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="endpoint_perception",
                output="screen",
                condition=IfCondition(use_yolo_missions),
                parameters=[
                    config,
                    {
                        "model_path": ParameterValue(
                            endpoint_yolo_model, value_type=str
                        )
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="path_tracker",
                output="screen",
                parameters=[
                    config,
                    gps_config,
                    ntrip_config,
                    {
                        "route_file": route,
                        "route_calibrated": ParameterValue(
                            route_calibrated, value_type=bool
                        ),
                        "target_speed_override_mps": ParameterValue(
                            mission_speed_mps, value_type=float
                        ),
                        "direction_change_hold_sec": ParameterValue(
                            direction_change_hold_sec, value_type=float
                        ),
                        "enable_parking_selection": ParameterValue(
                            enable_parking_selection, value_type=bool
                        ),
                        "enable_endpoint_selection": ParameterValue(
                            use_yolo_missions, value_type=bool
                        ),
                        "parking_source_route_file": ParameterValue(
                            parking_source_route_file, value_type=str
                        ),
                        "parking_source_variant_id": ParameterValue(
                            parking_source_variant_id, value_type=str
                        ),
                        "parking_source_start_s_m": ParameterValue(
                            parking_source_start_s_m, value_type=float
                        ),
                        "parking_source_length_m": ParameterValue(
                            parking_source_length_m, value_type=float
                        ),
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="mission_manager",
                output="screen",
                parameters=[
                    config,
                    gps_config,
                    ntrip_config,
                    {
                        "default_speed_mps": ParameterValue(
                            mission_speed_mps, value_type=float
                        ),
                        "enable_local_detour_steering": ParameterValue(
                            enable_local_detour_steering, value_type=bool
                        ),
                        "enable_parking_selection": ParameterValue(
                            enable_parking_selection, value_type=bool
                        ),
                        "enable_yolo_missions": ParameterValue(
                            use_yolo_missions, value_type=bool
                        ),
                        "parking_active_route_file": ParameterValue(
                            route, value_type=str
                        ),
                        "parking_source_route_file": ParameterValue(
                            parking_source_route_file, value_type=str
                        ),
                        "parking_source_variant_id": ParameterValue(
                            parking_source_variant_id, value_type=str
                        ),
                        "parking_source_start_s_m": ParameterValue(
                            parking_source_start_s_m, value_type=float
                        ),
                        "parking_source_length_m": ParameterValue(
                            parking_source_length_m, value_type=float
                        ),
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="local_detour",
                condition=IfCondition(use_local_detour_planner),
                output="screen",
                parameters=[
                    config,
                    gps_config,
                    {
                        "route_file": route,
                        "preview_mode": False,
                        "live_geometry_verified": ParameterValue(
                            lidar_geometry_verified, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="vehicle_controller",
                output="screen",
                parameters=[
                    config,
                    gps_config,
                    ntrip_config,
                    {
                        "maximum_duty": ParameterValue(
                            maximum_drive_duty, value_type=float
                        ),
                        "minimum_forward_duty": ParameterValue(
                            minimum_forward_duty, value_type=float
                        ),
                        "fixed_drive_pwm": ParameterValue(
                            fixed_drive_pwm, value_type=int
                        ),
                        "require_velocity_feedback": ParameterValue(
                            require_velocity_feedback, value_type=bool
                        ),
                        "hill_hold_enabled": ParameterValue(
                            hill_hold_enabled, value_type=bool
                        ),
                        "hill_hold_duty": ParameterValue(
                            hill_hold_duty, value_type=float
                        ),
                        "hill_approach_pwm": ParameterValue(
                            hill_approach_pwm, value_type=int
                        ),
                        "hill_post_stop_pwm": ParameterValue(
                            hill_post_stop_pwm, value_type=int
                        ),
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="safety_supervisor",
                output="screen",
                parameters=[
                    config,
                    gps_config,
                    ntrip_config,
                    {
                        "drive_enabled": ParameterValue(
                            drive_enabled, value_type=bool
                        ),
                        "route_calibrated": ParameterValue(
                            route_calibrated, value_type=bool
                        ),
                        "require_preflight": ParameterValue(
                            require_preflight, value_type=bool
                        ),
                        "relaxed_route_test_mode": ParameterValue(
                            relaxed_route_test_mode, value_type=bool
                        ),
                        "autonomous_maximum_drive_duty": ParameterValue(
                            maximum_drive_duty, value_type=float
                        ),
                    },
                ],
            ),
            Node(
                package="hl_ku_core",
                executable="nucleo_serial_bridge",
                condition=IfCondition(actuator_bridge_enabled),
                output="screen",
                parameters=[
                    config,
                    gps_config,
                    ntrip_config,
                    {
                        "maximum_drive_command": ParameterValue(
                            maximum_drive_command, value_type=int
                        ),
                        "allow_reverse": ParameterValue(
                            allow_reverse, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="gps_route_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(rviz_enabled),
            ),
            LogInfo(
                condition=UnlessCondition(require_preflight),
                msg=(
                    "SUPERVISED TRIAL: static preflight is bypassed; live RTK, "
                    "heading, pose, feedback, command and mission gates remain active"
                ),
            ),
        ]
    )
