"""RViz-only local detour preview; no motor or steering nodes are started."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    route = LaunchConfiguration("route_file")
    obstacle_s = LaunchConfiguration("obstacle_s_m")
    progress_s = LaunchConfiguration("progress_s_m")
    lateral = LaunchConfiguration("obstacle_lateral_m")
    open_rviz = LaunchConfiguration("open_rviz")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "route_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("hl_ku_core"), "routes", "course_07_vehicle.csv"]
                ),
            ),
            DeclareLaunchArgument("obstacle_s_m", default_value="210.0"),
            DeclareLaunchArgument("progress_s_m", default_value="200.0"),
            DeclareLaunchArgument("obstacle_lateral_m", default_value="0.0"),
            DeclareLaunchArgument("open_rviz", default_value="true"),
            Node(
                package="hl_ku_core",
                executable="local_detour",
                name="local_detour_preview",
                output="screen",
                parameters=[
                    {
                        "route_file": ParameterValue(route, value_type=str),
                        "preview_mode": True,
                        "preview_obstacle_s_m": ParameterValue(obstacle_s, value_type=float),
                        "preview_progress_s_m": ParameterValue(progress_s, value_type=float),
                        "preview_obstacle_lateral_m": ParameterValue(lateral, value_type=float),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="detour_preview_rviz",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [FindPackageShare("hl_ku_core"), "rviz", "local_detour_preview.rviz"]
                    ),
                ],
                condition=IfCondition(open_rviz),
            ),
        ]
    )
