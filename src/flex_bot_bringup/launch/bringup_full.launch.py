import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("flex_bot_bringup")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = os.path.join(pkg_share, "rviz", "bringup.rviz")

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "bringup_sensors.launch.py"))
    )

    static_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "static_tfs.launch.py"))
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "bringup_state_estimation.launch.py"))
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "slam_sync.launch.py"))
    )

    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", rviz_config],
    #     condition=IfCondition(use_rviz),
    # )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        sensors,
        static_tfs,
        ekf,
        slam,
        # rviz,
    ])
