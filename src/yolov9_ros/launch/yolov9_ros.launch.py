"""Legacy ROS 1-era detector launch. Not part of the supported Docker path.

objects_transform was removed on 2026-09-09: it had been dead since 2026-04-29 (KeyError on
a topics.yaml key that never existed), subscribed radar_msgs/RadarTrackArray while the
vehicle publishes delphi_esr_driver/msg/EsrTrackArray, and published /fused_bbox -- the same
topic name the live fusion_node uses -- with an incompatible message type. Radar is now
handled by src/radar_ros. The original is at
`git show pre-radar-known-good:src/yolov9_ros/objects_transform.py`.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolov9_ros',
            executable='yolov9_object_detection',
            name='yolov9_object_detection',
            output='screen'
        ),
    ])
