"""Brings up the object-fusion stack. Every gate is declared here and defaults OFF, except the
camera-LiDAR depth-jump gate, class vote and near-field ground levelling, which are on because
they were measured (see measurement_gate.py, class_vote.py, ground_segmentation.py).

Substitutions resolve to strings, so each argument is passed through ParameterValue with an
explicit value_type or the nodes reject them -- the same trap radar_fusion.launch.py documents.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_FLOAT = {"ego_yaw_correction_deg", "ego_z_offset", "measurement_lag", "sigma_long",
          "sensor_height", "camera_height_m", "assoc_max_dist", "merge_max_dist",
          "sticky_sanity_dist",
          "ground_max_range",
          "ground_rejection_min_range", "ground_margin",
          "depth_gate_min_m", "depth_gate_range_frac", "class_vote_window_s",
          "levelling_near_range", "max_odom_gap_s",
          "sigma_lat", "output_timeout", "max_pairing_skew", "wait_for_newer",
          "projection_buffer_duration", "min_range", "max_range", "min_amplitude"}
_BOOL = {"enable_radar_only_birth", "enable_extent_estimation", "use_sim_time",
         "enable_centroid_correction", "enable_camera_only_fallback",
         "publish_debug_clouds", "segmentation_empty_fallback", "enable_depth_gate",
         "enable_class_vote", "ground_levelling", "radar_camera_gate",
         "enable_lidar_clusters"}
_INT = {"min_update_count", "ground_min_points"}

_ARGS = {
    "use_sim_time": "false",
    "publish_mode": "filtered",
    "enable_radar_only_birth": "false",
    "enable_extent_estimation": "false",
    "ego_yaw_correction_deg": "-5.35",
    "ego_z_offset": "0.0",
    "measurement_lag": "0.12",
    "sigma_long": "2.0",
    "sigma_lat": "1.0",
    "output_timeout": "0.5",
    # /odom and /odom_grid are bit-identical in every TWIST component (they differ only in
    # pose yaw, by a constant 1.287 deg of UTM grid convergence) and this stack uses the twist
    # only. /odom is the default because topics.yaml's replay_input category lists it, so it
    # is what play_rosbag.sh actually replays -- defaulting to /odom_grid meant subscribing to
    # a topic no replay published. Override on the vehicle if odom_grid is what runs there.
    "odom_topic": "/novatel/oem7/odom",
    "max_pairing_skew": "0.06",
    "wait_for_newer": "0.06",
    "projection_buffer_duration": "2.0",
    "min_range": "1.0",
    "max_range": "175.0",
    "min_amplitude": "-1e9",
    "min_update_count": "0",
    "radar_frame_override": "delphi_esr_radar",
    # ---- ground-flagged projection (always published; USED only when the detector's
    #      projection_topic points at it) ----
    "ground_backend": "auto",
    "ground_max_range": "120.0",
    "publish_debug_clouds": "false",
    "sensor_height": "2.37",
    # Level the near field (< levelling_near_range) by odometry roll/pitch before Patchwork++;
    # beyond it the labels are the unlevelled ones. Measured: see LevelledGroundSegmenter.
    "ground_levelling": "true",
    "levelling_near_range": "25.0",
    "max_odom_gap_s": "0.2",
    "ground_rejection_min_range": "10.0",
    "ground_margin": "0.4",
    # Which projection camera_lidar_detector consumes. /perception/lidar_2d_projection_ground
    # enables ground segmentation (measured better 15-80 m); /lidar_2d_projection is the rollback.
    "projection_topic": "/lidar_2d_projection",
    "enable_centroid_correction": "false",
    "enable_camera_only_fallback": "false",
    # A box whose returns are ALL ground: publish nothing (false) or use every point (true, the
    # original rule, which put missed cones on road rings).
    "segmentation_empty_fallback": "false",
    # Withhold a one-frame depth outlier per ByteTrack id; never two in a row.
    "enable_depth_gate": "true",
    "depth_gate_min_m": "1.5",
    "depth_gate_range_frac": "0.05",
    # Size/label boxes from the tracker id's 2 s score-weighted majority class (a 2-frame
    # truck -> train relabel made a 200 m box).
    "enable_class_vote": "true",
    "class_vote_window_s": "2.0",
    "camera_height_m": "1.5275",
    "assoc_max_dist": "6.0",
    "merge_max_dist": "2.5",
    "radar_camera_gate": "true",
    # The 360-degree LiDAR cluster path: ground_projection publishes the full-sweep non-ground
    # cloud, lidar_cluster_detector clusters it, and the aggregator lets those clusters SUSTAIN
    # existing tracks (never birth). One flag turns the whole chain on.
    "enable_lidar_clusters": "false",
    "sticky_sanity_dist": "20.0",
}


def _p(name):
    cfg = LaunchConfiguration(name)
    if name in _FLOAT:
        return {name: ParameterValue(cfg, value_type=float)}
    if name in _BOOL:
        return {name: ParameterValue(cfg, value_type=bool)}
    if name in _INT:
        return {name: ParameterValue(cfg, value_type=int)}
    return {name: ParameterValue(cfg, value_type=str)}


def _params(*names):
    out = {}
    for n in names:
        out.update(_p(n))
    return [out]


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in _ARGS.items()]
    return LaunchDescription(args + [
        Node(package="object_fusion", executable="ego_frame_publisher",
             name="ego_frame_publisher", output="screen",
             parameters=_params("use_sim_time", "ego_yaw_correction_deg", "ego_z_offset")),
        Node(package="object_fusion", executable="ground_projection",
             name="ground_projection", output="screen",
             parameters=_params("use_sim_time", "ground_backend", "ground_max_range",
                                "sensor_height", "publish_debug_clouds", "ground_levelling",
                                "levelling_near_range", "max_odom_gap_s", "odom_topic")
             + [{"publish_nonground": ParameterValue(
                 LaunchConfiguration("enable_lidar_clusters"), value_type=bool)}]),
        Node(package="object_fusion", executable="lidar_cluster_detector",
             name="lidar_cluster_detector", output="screen",
             condition=IfCondition(LaunchConfiguration("enable_lidar_clusters")),
             parameters=_params("use_sim_time")),
        Node(package="object_fusion", executable="camera_lidar_detector",
             name="camera_lidar_detector", output="screen",
             parameters=_params("use_sim_time", "max_pairing_skew", "wait_for_newer",
                                "projection_buffer_duration", "enable_extent_estimation",
                                "enable_centroid_correction", "enable_camera_only_fallback",
                                "camera_height_m", "projection_topic",
                                "ground_rejection_min_range", "ground_margin",
                                "segmentation_empty_fallback", "enable_depth_gate",
                                "depth_gate_min_m", "depth_gate_range_frac",
                                "enable_class_vote", "class_vote_window_s")),
        Node(package="object_fusion", executable="radar_detector",
             name="radar_detector", output="screen",
             parameters=_params("use_sim_time", "min_range", "max_range", "min_amplitude",
                                "min_update_count", "radar_frame_override")),
        Node(package="object_fusion", executable="object_aggregator",
             name="object_aggregator", output="screen",
             parameters=_params("use_sim_time", "publish_mode", "enable_radar_only_birth",
                                "ego_yaw_correction_deg", "measurement_lag", "sigma_long",
                                "sigma_lat", "output_timeout", "odom_topic",
                                "assoc_max_dist", "merge_max_dist", "radar_camera_gate",
                                "enable_lidar_clusters", "sticky_sanity_dist")),
    ])
