from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clrernet_ros',
            executable='clrernet_lane_detection.py',
            name='clrernet_ros_node',
            output='screen'
        ),
        Node(
            package='clrernet_ros',
            executable='clrernet_lane_transform.py',
            name='clrernet_transform',
            output='screen'
        ),
    ])
