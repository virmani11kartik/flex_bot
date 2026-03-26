import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share      = get_package_share_directory("flex_bot_bringup")
    wheel_odom_pkg = get_package_share_directory("flex_bot_odometry")

    use_rviz       = LaunchConfiguration("use_rviz")
    rviz_config    = os.path.join(pkg_share, "rviz", "bringup.rviz")
    amcl_params_file = os.path.join(pkg_share, "config", "amcl.yaml")
    map_yaml_path  = os.path.join(pkg_share, "maps", "my_map.yaml")

    wheel_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheel_odom_pkg, "launch", "wheel_odom.launch.py")))

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "bringup_sensors.launch.py")))

    static_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "static_tfs.launch.py")))

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "bringup_state_estimation.launch.py")))

    # ── obstacle detection (delayed — waits for tf tree to settle) ──────────
    obstacle_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "obstacle_detection.launch.py")))

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "yaml_filename": map_yaml_path,
        }])

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_params_file])

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz))

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to launch RViz"),

        # ── launch order ──────────────────────────────────────────────────
        sensors,           # lidar first — tf tree needs scan frames
        static_tfs,        # static transforms
        wheel_odom,        # odom → base_link
        ekf,               # filtered odometry
        map_server,        # map frame
        amcl,              # map → odom localization
        lifecycle_manager, # activates map_server + amcl
        rviz,
        obstacle_detection,
    ])