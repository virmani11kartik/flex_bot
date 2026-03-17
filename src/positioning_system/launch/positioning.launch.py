import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share   = get_package_share_directory("positioning_system")
    params_file = os.path.join(pkg_share, "config", "positioning_params.yaml")
    wp_file     = os.path.join(pkg_share, "config", "markers.json")

    dock_controller = Node(
        package="positioning_system",
        executable="pgv_dock_controller",
        name="pgv_dock_controller",
        output="screen",
        parameters=[
            params_file,
            {"markers_file": wp_file},
        ],
    )

    return LaunchDescription([
        dock_controller,
    ])
