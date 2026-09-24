"""RTK check with the C270 camera and a live Foxglove connection."""

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
    receiver_config = LaunchConfiguration("receiver_config_file")
    ntrip_config = LaunchConfiguration("ntrip_config_file")
    gps_config = LaunchConfiguration("gps_config_file")
    camera_device = LaunchConfiguration("camera_device")
    bridge_port = LaunchConfiguration("bridge_port")
    survey_enabled = LaunchConfiguration("survey_enabled")
    survey_duration = LaunchConfiguration("survey_duration_sec")
    survey_samples = LaunchConfiguration("survey_minimum_samples")
    survey_hdop = LaunchConfiguration("survey_maximum_hdop")
    survey_output = LaunchConfiguration("survey_output_file")
    rtk_launch = PathJoinSubstitution(
        [core_share, "launch", "rtk_check.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "receiver_config_file",
                default_value=PathJoinSubstitution(
                    [core_share, "config", "rtk_field_test.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_config_file",
                default_value=PathJoinSubstitution(
                    [core_share, "config", "ntrip_private.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "gps_config_file",
                default_value=PathJoinSubstitution(
                    [core_share, "config", "gps_only_route.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "camera_device",
                default_value="/dev/video4",
            ),
            DeclareLaunchArgument("bridge_port", default_value="8765"),
            DeclareLaunchArgument("survey_enabled", default_value="false"),
            DeclareLaunchArgument("survey_duration_sec", default_value="120.0"),
            DeclareLaunchArgument("survey_minimum_samples", default_value="100"),
            DeclareLaunchArgument("survey_maximum_hdop", default_value="2.5"),
            DeclareLaunchArgument("survey_output_file", default_value="/tmp/konkuk_datum.yaml"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtk_launch),
                launch_arguments={
                    "receiver_config_file": receiver_config,
                    "ntrip_config_file": ntrip_config,
                }.items(),
            ),
            Node(
                package="hl_ku_core",
                executable="gnss_localizer",
                name="gnss_localizer",
                output="screen",
                parameters=[gps_config],
            ),
            Node(
                package="hl_ku_core",
                executable="gnss_survey",
                name="konkuk_datum_survey",
                output="screen",
                condition=IfCondition(survey_enabled),
                parameters=[
                    {
                        "duration_sec": ParameterValue(
                            survey_duration, value_type=float
                        ),
                        "minimum_samples": ParameterValue(
                            survey_samples, value_type=int
                        ),
                        "maximum_hdop": ParameterValue(
                            survey_hdop, value_type=float
                        ),
                        "output_file": ParameterValue(
                            survey_output, value_type=str
                        ),
                        "site_name": "konkuk_university",
                    }
                ],
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="mando_camera",
                output="screen",
                parameters=[
                    {
                        "video_device": ParameterValue(camera_device, value_type=str),
                        "framerate": 30.0,
                        "io_method": "mmap",
                        "pixel_format": "mjpeg2rgb",
                        "image_width": 640,
                        "image_height": 480,
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
                executable="live_status_visualizer",
                name="hl_ku_foxglove_live_status",
                output="screen",
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
                    }
                ],
            ),
        ]
    )
