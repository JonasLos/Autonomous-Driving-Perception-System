from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_ring_filter",
            default_value="false",
            description="Start points_downsampler ring_filter node (requires points_downsampler to be built)",
        ),
        DeclareLaunchArgument(
            "input_topic",
            default_value="/lidar_tc/velodyne_points",
            description="Input PointCloud2 topic for SphereFormer when ring_filter is disabled.",
        ),
        Node(
            package='sphereformer_ros',
            executable='sphereformer_ros',
            name='sphereformer',
            output='screen',
            parameters=[
                {
                    'input_topic': LaunchConfiguration('input_topic'),
                }
            ]
        ),
        Node(
            package='points_downsampler',
            executable='ring_filter',
            name='ring_filter',
            output='screen',
            condition=IfCondition(LaunchConfiguration("use_ring_filter")),
            parameters=[
                {'points_topic': LaunchConfiguration('input_topic')},
                {'output_log': False},
                {'measurement_range': 100}
            ]
        ),
    ])
