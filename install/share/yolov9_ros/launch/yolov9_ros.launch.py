from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolov9_ros',
            executable='yolov9_object_detection.py',
            name='yolov9_object_detection',
            output='screen'
        ),
        Node(
            package='yolov9_ros',
            executable='objects_transform.py',
            name='objects_transform',
            output='screen'
        ),
    ])
