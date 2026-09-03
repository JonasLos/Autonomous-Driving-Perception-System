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

    # The grid is read-only at runtime, so its extent has to be set here or not at all.
    lane_grid_max_x = ParameterValue(
        LaunchConfiguration('lane_grid_max_x'), value_type=float
    )
    # The output-shape switch: see the node docstring for what changes and why it is off.
    centerline_from_grid = ParameterValue(
        LaunchConfiguration('centerline_from_grid'), value_type=bool
    )
    # Exposed here because it is the one parameter that becomes wrong the moment the vehicle's
    # calibration is fixed, and whoever fixes it will be looking at a launch file.
    lane_ego_yaw_deg = ParameterValue(
        LaunchConfiguration('lane_ego_yaw_deg'), value_type=float
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
        DeclareLaunchArgument(
            'lane_grid_max_x',
            default_value='100.0',
            description=(
                'Farthest range, in metres, the lane grid reaches. Nodes past a lane\'s own '
                'measured span are masked out, so an over-long grid only costs unused slots '
                'while an over-short one truncates the published horizon. Read-only at '
                'runtime, so it must be set here.'
            ),
        ),
        DeclareLaunchArgument(
            'centerline_from_grid',
            default_value='false',
            description=(
                'Publish left/right/centerline sampled on the longitudinal grid (matched '
                'range, uniform spacing) instead of the selected boundaries\' own matched '
                'returns. False keeps the point count and spacing the planning stack '
                'receives today, at the cost of a centerline whose paired points sit a '
                'measured 3.60 m apart in range at the median.'
            ),
        ),
        DeclareLaunchArgument(
            'lane_ego_yaw_deg',
            default_value='-5.35',
            description=(
                "Azimuth of the ego path in the LiDAR frame, in degrees. lidar_tc is yawed "
                "about 5.35 deg from the vehicle axis, so y=0 is not the ego path -- it "
                "diverges by 9.4 cm per metre of range, which is what made the lane selector "
                "jump to the neighbouring lane. Measured from the detected lanes themselves "
                "(-5.354) and corroborated by the bumper radar (-5.443), camera_fl's optical "
                "axis (-5.061) and the lane vanishing point (-4.95); /tf_static publishes "
                "lidar_tc -> base_link as identity, which is the only transform on the vehicle "
                "that disagrees. Set to 0.0 once that transform carries a real calibration and "
                "/lidar_2d_projection arrives already in the vehicle frame."
            ),
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
                'lane_grid_max_x': lane_grid_max_x,
                'centerline_from_grid': centerline_from_grid,
                'lane_ego_yaw_deg': lane_ego_yaw_deg,
            }],
        ),
    ])
