from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sphereformer_ros',
            executable='sphereformer_ros.py',
            name='sphereformer',
            output='screen'
        ),
        Node(
            package='points_downsampler',
            executable='ring_filter',
            name='ring_filter',
            output='screen',
            parameters=[
                {'points_topic': '/lidar_tc/veloydne_points'},
                {'output_log': False},
                {'measurement_range': 100}
            ]
        ),
    ])
