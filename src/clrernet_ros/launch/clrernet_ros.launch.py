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

    # Detections run at ~5 Hz against ~10 Hz LiDAR, so a projection can be up to one
    # detection interval behind. 0.25s = that interval plus margin; lower it toward
    # 0.12s to keep only tightly-paired frames at the cost of output rate.
    max_detection_age = ParameterValue(
        LaunchConfiguration('max_detection_age'), value_type=float
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock instead of wall time (set true for bag replay).',
        ),
        DeclareLaunchArgument(
            'max_detection_age',
            default_value='0.25',
            description=(
                'Max capture-time skew, in seconds, between a LiDAR projection and the '
                'lane detection fused with it. Projections outside this bound are dropped.'
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
                'max_detection_age': max_detection_age,
            }],
        ),
    ])
