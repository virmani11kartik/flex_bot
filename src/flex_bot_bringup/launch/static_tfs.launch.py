from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # tf_imu = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_tf_xsens_imu",
    #     output="screen",
    #     arguments=["0", "0", "0", "0", "0", "0", "base_link", "xsens_imu"],
    # )

    # tf_imu = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_tf_sick_imu",
    #     output="screen",
    #     arguments=["0", "0", "0", "0", "0", "0", "base_link", "sick_imu"],
    # )

    tf_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_laser_1",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser_1"],
    )

    return LaunchDescription([tf_lidar])
