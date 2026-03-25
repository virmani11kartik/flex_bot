# positioning.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory("positioning_system")

    # default to package share, but easily overridable
    markers_arg = DeclareLaunchArgument(
        "markers_file",
        default_value=os.path.join(pkg_share, "config", "markers.json"),
        description="Path to markers.json"
    )

    params_file = os.path.join(pkg_share, "config", "positioning_params.yaml")

    return LaunchDescription([
        markers_arg,
        Node(
            package="positioning_system",
            executable="pgv_dock_controller",
            name="pgv_dock_controller",
            output="screen",
            parameters=[
                params_file,
                {"markers_file": LaunchConfiguration("markers_file")}
            ],
        )
    ])