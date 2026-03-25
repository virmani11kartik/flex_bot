from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    extractor = Node(
        package='obstacle_detector',
        executable='obstacle_extractor_node',
        name='obstacle_extractor',
        parameters=[{
            'use_sim_time':              False,
            'active':                    True,
            'use_scan':                  True,
            'use_pcl':                   False,
            'use_split_and_merge':       True,
            'circles_from_visibles':     True,
            'discard_converted_segments': True,
            'transform_coordinates':     True,
            'min_group_points':          5,
            'max_group_distance':        0.1,
            'distance_proportion':       0.00628,
            'max_split_distance':        0.2,
            'max_merge_separation':      0.2,
            'max_merge_spread':          0.2,
            'max_circle_radius':         0.6,
            'radius_enlargement':        0.3,
            'frame_id':                  'map',
        }],
        remappings=[
            ('/scan', '/scan_fullframe'),
        ]
    )

    tracker = Node(
        package='obstacle_detector',
        executable='obstacle_tracker_node',
        name='obstacle_tracker',
        parameters=[{
            'use_sim_time':            False,
            'active':                  True,
            'loop_rate':               100.0,
            'tracking_duration':       2.0,
            'min_correspondence_cost': 0.3,
            'std_correspondence_dev':  0.15,
            'process_variance':        0.01,
            'process_rate_variance':   0.1,
            'measurement_variance':    1.0,
            'frame_id':                'map',
        }],
        remappings=[
            ('/odom', '/odometry/filtered'),
        ]
    )

    return LaunchDescription([
        TimerAction(period=8.0, actions=[extractor, tracker])
    ])