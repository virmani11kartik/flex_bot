from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("flex_bot_teleop")

    udp_yaml    = os.path.join(pkg_share, "config", "flex_bot_udp.yaml")
    teleop_yaml = os.path.join(pkg_share, "config", "teleop.yaml")
    serial_yaml = os.path.join(pkg_share, "config", "serial_bridge.yaml")

    return LaunchDescription([

        # ── Joystick driver ───────────────────────────────────────────────
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

        # ── Teleop: joystick → all cmd topics ─────────────────────────────
        Node(
            package="flex_bot_teleop",
            executable="teleop_node",
            name="teleop_node",
            output="screen",
            parameters=[teleop_yaml],
        ),

        # ── UDP bridge: drive + turret cmds → iMX7 ────────────────────────
        Node(
            package="flex_bot_teleop",
            executable="flex_bot_udp_bridge",
            name="flex_bot_udp_bridge",
            output="screen",
            parameters=[udp_yaml],
        ),

        # ── Serial bridge: actuator + stepper cmds → ESP32 ────────────────
        Node(
            package="flex_bot_teleop",
            executable="serial_bridge_node",
            name="serial_bridge_node",
            output="screen",
            parameters=[serial_yaml],
        ),

    ])
