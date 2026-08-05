from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    points_topic = LaunchConfiguration('points_topic')
    output_topic = LaunchConfiguration('output_topic')
    output_log = LaunchConfiguration('output_log')
    measurement_range = LaunchConfiguration('measurement_range')

    return LaunchDescription([
        DeclareLaunchArgument(
            'points_topic',
            default_value='/lidar_tc/velodyne_points',
            description='Input point cloud topic',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/lidar_tc/velodyne_points/downsampled',
            description='Output point cloud topic',
        ),
        DeclareLaunchArgument(
            'output_log',
            default_value='false',
            description='Enable CSV logging',
        ),
        DeclareLaunchArgument(
            'measurement_range',
            default_value='200.0',
            description='Maximum measurement range',
        ),
        Node(
            package='points_downsampler',
            executable='ring_filter',
            name='ring_filter',
            output='screen',
            parameters=[{
                'points_topic': points_topic,
                'output_topic': output_topic,
                'output_log': output_log,
                'measurement_range': measurement_range,
            }],
        ),
    ])