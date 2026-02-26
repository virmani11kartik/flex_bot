from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("flex_bot_teleop")

    udp_yaml   = os.path.join(pkg_share, "config", "flex_bot_udp.yaml")
    teleop_yaml = os.path.join(pkg_share, "config", "teleop.yaml")

    return LaunchDescription([

        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "dev": "/dev/input/event19",
                "deadzone": 0.05,
                "autorepeat_rate": 50.0,
            }],
        ),

        Node(
            package="flex_bot_teleop",
            executable="flex_bot_udp_bridge",
            name="flex_bot_udp_bridge",
            output="screen",
            parameters=[udp_yaml],
        ),

        Node(
            package="flex_bot_teleop",
            executable="teleop_node",
            name="teleop_node",
            output="screen",
            parameters=[teleop_yaml],
        ),
    ])
