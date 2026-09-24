"""Live Mando route trial with camera and the Foxglove dashboard bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    core_share = FindPackageShare("hl_ku_core")
    foxglove_share = FindPackageShare("hl_ku_foxglove")
    route_launch = PathJoinSubstitution(
        [core_share, "launch", "gps_route_test.launch.py"]
    )

    forwarded = (
        "config_file",
        "gps_config_file",
        "ntrip_config_file",
        "calibration_file",
        "route_file",
        "drive_enabled",
        "route_calibrated",
        "actuator_bridge_enabled",
        "require_preflight",
        "relaxed_route_test_mode",
        "require_velocity_feedback",
        "mission_speed_mps",
        "maximum_drive_duty",
        "minimum_forward_duty",
        "fixed_drive_pwm",
        "maximum_drive_command",
        "allow_reverse",
        "direction_change_hold_sec",
        "use_lidar_pipeline",
        "start_lidar_driver",
        "use_local_detour_planner",
        "lidar_geometry_verified",
        "enable_local_detour_steering",
        "enable_parking_selection",
        "use_camera_perception",
        "use_yolo_missions",
        "integrated_yolo_model",
        "endpoint_yolo_model",
        "hill_hold_enabled",
        "hill_hold_duty",
        "hill_approach_pwm",
        "hill_post_stop_pwm",
        "rviz",
        "rviz_config",
    )
    defaults = {
        "config_file": PathJoinSubstitution([core_share, "config", "system.yaml"]),
        "gps_config_file": PathJoinSubstitution(
            [core_share, "config", "gps_only_route.yaml"]
        ),
        "ntrip_config_file": PathJoinSubstitution(
            [core_share, "config", "ntrip_private.yaml"]
        ),
        "calibration_file": PathJoinSubstitution(
            [core_share, "config", "calibration_record.yaml"]
        ),
        "route_file": PathJoinSubstitution(
            [core_share, "routes", "course_06_vehicle.csv"]
        ),
        "drive_enabled": "false",
        "route_calibrated": "false",
        "actuator_bridge_enabled": "true",
        "require_preflight": "true",
        "relaxed_route_test_mode": "false",
        "require_velocity_feedback": "true",
        "mission_speed_mps": "0.30",
        "maximum_drive_duty": "0.30",
        "minimum_forward_duty": "0.30",
        "fixed_drive_pwm": "-1",
        "maximum_drive_command": "30",
        "allow_reverse": "false",
        "direction_change_hold_sec": "0.75",
        "use_lidar_pipeline": "true",
        "start_lidar_driver": "true",
        "use_local_detour_planner": "true",
        "lidar_geometry_verified": "false",
        "enable_local_detour_steering": "false",
        "enable_parking_selection": "false",
        "use_camera_perception": "false",
        "use_yolo_missions": "false",
        "integrated_yolo_model": PathJoinSubstitution(
            [core_share, "weights", "integrated_7class", "best.pt"]
        ),
        "endpoint_yolo_model": PathJoinSubstitution(
            [core_share, "weights", "endpoint_2class_v3", "best.pt"]
        ),
        "hill_hold_enabled": "false",
        "hill_hold_duty": "0.0",
        "hill_approach_pwm": "-1",
        "hill_post_stop_pwm": "-1",
        "rviz": "false",
        "rviz_config": PathJoinSubstitution(
            [core_share, "rviz", "local_detour_preview.rviz"]
        ),
    }

    camera_enabled = LaunchConfiguration("camera_enabled")
    camera_device = LaunchConfiguration("camera_device")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_framerate = LaunchConfiguration("camera_framerate")
    bridge_port = LaunchConfiguration("bridge_port")
    route_file = LaunchConfiguration("route_file")
    variant_name = LaunchConfiguration("active_variant_name")
    source_route_file = LaunchConfiguration("source_route_file")
    source_variant_id = LaunchConfiguration("source_variant_id")
    source_start_s_m = LaunchConfiguration("source_start_s_m")
    source_length_m = LaunchConfiguration("source_length_m")
    source_segment_name = LaunchConfiguration("source_segment_name")
    source_fsm_label = LaunchConfiguration("source_fsm_label")

    actions = [
        *[
            DeclareLaunchArgument(name, default_value=default)
            for name, default in defaults.items()
        ],
        DeclareLaunchArgument("active_variant_name", default_value="Mando live route"),
        DeclareLaunchArgument(
            "source_route_file",
            default_value=PathJoinSubstitution(
                [core_share, "routes", "course_07_vehicle.csv"]
            ),
        ),
        DeclareLaunchArgument("source_start_s_m", default_value="0.0"),
        DeclareLaunchArgument("source_variant_id", default_value=""),
        DeclareLaunchArgument("source_length_m", default_value="0.0"),
        DeclareLaunchArgument("source_segment_name", default_value=""),
        DeclareLaunchArgument("source_fsm_label", default_value=""),
        DeclareLaunchArgument("camera_enabled", default_value="true"),
        DeclareLaunchArgument(
            "camera_device",
            default_value="/dev/video4",
        ),
        DeclareLaunchArgument("camera_width", default_value="640"),
        DeclareLaunchArgument("camera_height", default_value="480"),
        DeclareLaunchArgument("camera_framerate", default_value="30.0"),
        DeclareLaunchArgument("bridge_port", default_value="8765"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(route_launch),
            launch_arguments={
                **{name: LaunchConfiguration(name) for name in forwarded},
                "parking_source_route_file": source_route_file,
                "parking_source_variant_id": source_variant_id,
                "parking_source_start_s_m": source_start_s_m,
                "parking_source_length_m": source_length_m,
            }.items(),
        ),
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="mando_camera",
            output="screen",
            condition=IfCondition(camera_enabled),
            parameters=[
                {
                    "video_device": ParameterValue(camera_device, value_type=str),
                    "framerate": ParameterValue(camera_framerate, value_type=float),
                    "io_method": "mmap",
                    "pixel_format": "mjpeg2rgb",
                    "image_width": ParameterValue(camera_width, value_type=int),
                    "image_height": ParameterValue(camera_height, value_type=int),
                    "camera_name": "mando_camera",
                    "frame_id": "camera_link",
                }
            ],
            remappings=[
                ("image_raw", "/camera/image_raw"),
                ("camera_info", "/camera/camera_info"),
            ],
        ),
        Node(
            package="hl_ku_foxglove",
            executable="replay_visualizer",
            name="hl_ku_foxglove_visualizer",
            output="screen",
            parameters=[
                {
                    "active_route_file": ParameterValue(route_file, value_type=str),
                    "active_variant_name": ParameterValue(
                        variant_name, value_type=str
                    ),
                    "source_route_file": ParameterValue(
                        source_route_file, value_type=str
                    ),
                    "source_route_variant_id": ParameterValue(
                        source_variant_id, value_type=str
                    ),
                    "source_start_s_m": ParameterValue(
                        source_start_s_m, value_type=float
                    ),
                    "source_length_m": ParameterValue(
                        source_length_m, value_type=float
                    ),
                    "source_segment_name": ParameterValue(
                        source_segment_name, value_type=str
                    ),
                    "source_fsm_label": ParameterValue(
                        source_fsm_label, value_type=str
                    ),
                    "source_mode": "LIVE_DRIVE: camera + GNSS + route",
                    "use_sim_time": False,
                }
            ],
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[
                {
                    "address": "127.0.0.1",
                    "port": ParameterValue(bridge_port, value_type=int),
                    "use_sim_time": False,
                }
            ],
        ),
    ]
    return LaunchDescription(actions)
