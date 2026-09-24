"""Read-only Foxglove bridge and course 07 replay visualization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    foxglove_share = FindPackageShare("hl_ku_foxglove")
    core_share = FindPackageShare("hl_ku_core")
    default_config = PathJoinSubstitution(
        [foxglove_share, "config", "course_07_replay.yaml"]
    )
    default_active_route = PathJoinSubstitution(
        [core_share, "routes", "mando", "competition", "course_07_full.csv"]
    )
    default_recorded_route = ""
    variant_names = [
        "course_07_p1_t1_f1_parallel_csv.csv",
        "course_07_p1_t1_f2_parallel_csv.csv",
        "course_07_p2_t1_f1_parallel_csv.csv",
        "course_07_p2_t1_f2_parallel_csv.csv",
        "course_07_p1_t2_f1_parallel_csv.csv",
        "course_07_p1_t2_f2_parallel_csv.csv",
        "course_07_p2_t2_f1_parallel_csv.csv",
        "course_07_p2_t2_f2_parallel_csv.csv",
    ]
    variant_parameters = {
        f"variant_{index:02d}_file": ParameterValue(
            PathJoinSubstitution(
                [core_share, "routes", "course_07_parking_overlay", filename]
            ),
            value_type=str,
        )
        for index, filename in enumerate(variant_names, start=1)
    }

    config = LaunchConfiguration("config_file")
    active_route = LaunchConfiguration("active_route_file")
    recorded_route = LaunchConfiguration("recorded_route_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    open_foxglove = LaunchConfiguration("open_foxglove")
    bridge_port = LaunchConfiguration("bridge_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument(
                "active_route_file", default_value=default_active_route
            ),
            DeclareLaunchArgument(
                "recorded_route_file", default_value=default_recorded_route
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("open_foxglove", default_value="true"),
            DeclareLaunchArgument("bridge_port", default_value="8765"),
            Node(
                package="hl_ku_core",
                executable="gnss_localizer",
                name="gnss_localizer",
                output="screen",
                parameters=[
                    config,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
            Node(
                package="hl_ku_foxglove",
                executable="replay_visualizer",
                name="hl_ku_foxglove_visualizer",
                output="screen",
                parameters=[
                    config,
                    {
                        "recorded_route_file": ParameterValue(
                            recorded_route, value_type=str
                        ),
                        "active_route_file": ParameterValue(
                            active_route, value_type=str
                        ),
                        **variant_parameters,
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    },
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
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    }
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "foxglove-studio",
                    ["foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:", bridge_port, "/"],
                ],
                condition=IfCondition(open_foxglove),
                output="screen",
            ),
        ]
    )
