"""Launch the radar fusion node.

Every parameter is declared here so the whole surface is visible in one place and settable
with `ros2 launch ... <arg>:=<value>`, matching how yolo.launch.py exposes the fusion
parameters. The two gates default to false: this launch file on its own changes nothing
about what the obstacle path publishes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        # --- the two gates, both off ---------------------------------------------
        ("enable_radar_fusion", "false", "Allow radar to MODIFY a fused object. False = passthrough + shadow logging."),
        ("publish_radar_only", "false", "Allow radar to CREATE an object. Requires enable_radar_fusion."),
        # --- pairing --------------------------------------------------------------
        ("max_pairing_skew", "0.05", "Max |capture-time| difference between a fused frame and a radar sweep, seconds."),
        ("radar_buffer_duration", "2.0", "How long radar sweeps are retained for matching, seconds."),
        ("wait_for_newer", "0.05", "Bounded wait for a radar sweep at or after the fused frame. 0.0 = one-sided."),
        ("radar_stamp_offset", "0.0", "Applied to radar stamps before matching, seconds."),
        # --- association ----------------------------------------------------------
        ("assoc_max_range_err", "3.0", "Radial gate between a fused object and a radar track, metres."),
        ("assoc_max_azimuth_err_deg", "3.0", "Angular gate, degrees. Wider than the radial gate on purpose."),
        # --- track gating ---------------------------------------------------------
        ("min_range", "1.0", "Discard radar tracks nearer than this, metres."),
        ("max_range", "175.0", "Discard radar tracks beyond this, metres."),
        ("min_amplitude", "-1e9", "Discard radar tracks weaker than this. Default off: the sensor floor is -10 and ~30% of tracks sit on it."),
        ("min_update_count", "0", "INERT: this driver never populates update_count. Any positive value drops ALL tracks."),
        # --- misc -----------------------------------------------------------------
        ("range_dispute_threshold", "5.0", "|fused range - radar range| above which range_disputed is set, metres."),
        ("output_timeout", "0.5", "Publish an empty array if output stops being refreshed, seconds."),
        ("radar_frame_override", "delphi_esr_radar", "Frame assumed when the driver sends an empty frame_id."),
        ("use_sim_time", "false", "Run on /clock. Never set this on the vehicle."),
    ]

    float_params = {
        "max_pairing_skew", "radar_buffer_duration", "wait_for_newer",
        "radar_stamp_offset", "assoc_max_range_err", "assoc_max_azimuth_err_deg",
        "min_range", "max_range", "min_amplitude", "range_dispute_threshold",
        "output_timeout",
    }
    bool_params = {"enable_radar_fusion", "publish_radar_only", "use_sim_time"}
    int_params = {"min_update_count"}

    def typed(name):
        # Substitutions resolve to strings, so the target type is declared explicitly --
        # otherwise the node rejects the override against its own typed declaration.
        cfg = LaunchConfiguration(name)
        if name in float_params:
            return ParameterValue(cfg, value_type=float)
        if name in bool_params:
            return ParameterValue(cfg, value_type=bool)
        if name in int_params:
            return ParameterValue(cfg, value_type=int)
        return ParameterValue(cfg, value_type=str)

    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args]
        + [
            Node(
                package="radar_ros",
                executable="radar_fusion_node",
                name="radar_fusion_node",
                output="screen",
                parameters=[{n: typed(n) for n, _, _ in args}],
            )
        ]
    )
