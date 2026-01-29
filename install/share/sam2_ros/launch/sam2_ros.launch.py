from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sam2_ros',
            executable='samv2_image_segmenation.py',
            name='Samv2ImageSegmentation',
            output='screen'
        ),
        Node(
            package='sam2_ros',
            executable='samv2_mask_transform.py',
            name='Samv2MaskTransform',
            output='screen'
        ),
    ])
