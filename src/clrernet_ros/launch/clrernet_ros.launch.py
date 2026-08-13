from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Live vehicle is the primary target, so sim time is off by default. Set
    # use_sim_time:=true when replaying a bag with --clock. Fusion staleness gating
    # compares two message stamps to each other and does not depend on this, but TF
    # lookups and other node-clock arithmetic do.
    # Substitutions resolve to strings, so the target type is declared explicitly —
    # otherwise the node rejects the override against its double/bool declaration.
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool
    )

    # Each lane detection is fused against the buffered LiDAR projection captured nearest
    # to its own header stamp, so camera and inference latency delay *when* a lane appears
    # without displacing *where* its points land. max_pairing_skew therefore only has to
    # cover the LiDAR period and jitter, not the pipeline latency; raise
    # projection_buffer_duration instead if detection latency ever exceeds it (the node
    # logs which case it hit).
    max_pairing_skew = ParameterValue(
        LaunchConfiguration('max_pairing_skew'), value_type=float
    )
    projection_buffer_duration = ParameterValue(
        LaunchConfiguration('projection_buffer_duration'), value_type=float
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock instead of wall time (set true for bag replay).',
        ),
        DeclareLaunchArgument(
            'max_pairing_skew',
            default_value='0.08',
            description=(
                'Largest capture-time gap, in seconds, allowed between a lane detection '
                'and the buffered LiDAR projection it is fused with.'
            ),
        ),
        DeclareLaunchArgument(
            'projection_buffer_duration',
            default_value='2.0',
            description=(
                'How much projection history, in seconds, to retain for matching. Must '
                'exceed the camera-to-detection latency.'
            ),
        ),
        Node(
            package='clrernet_ros',
            executable='clrernet_lane_detection',
            name='clrernet_ros_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='clrernet_ros',
            executable='clrernet_lane_transform',
            name='clrernet_transform',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'max_pairing_skew': max_pairing_skew,
                'projection_buffer_duration': projection_buffer_duration,
            }],
        ),
    ])
