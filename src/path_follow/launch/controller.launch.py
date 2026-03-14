import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("path_follow")
    params_file = os.path.join(pkg_share, "config", "controller_params.yaml")

    use_prm = LaunchConfiguration("use_prm")

    prm_builder = Node(
        package="path_follow",
        executable="prm_builder",
        name="prm_builder",
        output="screen",
        parameters=[params_file],
    )

    astar_search = Node(
        package="path_follow",
        executable="astar_search",
        name="astar_planner",
        output="screen",
        parameters=[params_file, {"use_prm": use_prm}],
    )

    waypoint_controller = Node(
        package="path_follow",
        executable="waypoint_controller",
        name="waypoint_controller",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_prm",
            default_value="true",
            description="Use PRM graph for A* (true) or raw grid A* (false)",
        ),
        prm_builder,
        astar_search,
        waypoint_controller,
    ])
