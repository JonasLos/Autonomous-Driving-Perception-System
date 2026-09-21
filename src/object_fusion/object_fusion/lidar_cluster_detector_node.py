"""360-degree LiDAR clusters as measurements: the side and rear the camera never sees.

    /perception/nonground (full-sweep non-ground, from ground_projection)
      -> object_fusion.lidar_clusters
      -> /perception/measurements/lidar   (Detection3DArray, sensor = SENSOR_LIDAR)

These measurements may only SUSTAIN a track the camera started; the aggregator never births from
them. A cluster carries no semantics -- a bush, a kerb and a car look alike -- and the radar work
(HANDOFF item 8) measured what unrefereed birth costs.

WHY IT EARNS ITS PLACE, measured before this node existed (scripts/lidar_cluster_ab.py). Following
static objects past the moment their track died, with a mirrored-position control for chance:

    cluster present, by sector    2.5-15 m   15-40 m   40-60 m      chance
      ahead  |bearing| < 30 deg     62.7%     58.8%     52.7%    0.0 / 4.2 / 30.9%
      side   30-150 deg             40.7%     22.0%     69.0%    0.2 / 11.0 / 0.0%
      behind > 150 deg              52.7%     64.7%     69.5%    0.0 / 1.0 / 0.7%

The REAR is the best-covered sector of all. The SIDE is the roof LiDAR's blind zone -- a 0.45 m
cone within ~4 m sits under the lowest beam -- and an object the car has just passed crosses it on
its way to the rear. Tolerating that half second (which the 0.5 s coast budget already does) takes
departing tracks held from 25% to 45%, for a median 4.3 s instead of 0.7 s.

MOVING objects are NOT established: that harness propagated a dead track at constant velocity
without ever updating it, which drifts within a second, and they scored at chance. This node
updates the track every sweep, so it should do better -- but that is a claim to measure once it
runs, not one to believe.
"""

import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from fusion_msgs.msg import Detection3D, Detection3DArray

from object_fusion.lidar_clusters import ClusterParams, cluster_nonground


class LidarClusterDetectorNode(Node):
    def __init__(self):
        super().__init__("lidar_cluster_detector")
        cfg = yaml.safe_load(
            open(get_package_share_directory("object_fusion") + "/config/topics.yaml",
                 encoding="utf-8"))
        m = cfg["topics"]["measurements"]
        self._in = str(self.declare_parameter("nonground_topic", m["nonground"]).value)
        self._out = str(self.declare_parameter("lidar_topic", m["lidar"]).value)
        p = self.declare_parameter
        self._params = ClusterParams(
            min_range=float(p("cluster_min_range", ClusterParams.min_range).value),
            max_range=float(p("cluster_max_range", ClusterParams.max_range).value),
            voxel=float(p("cluster_voxel", ClusterParams.voxel).value),
            gap_base=float(p("cluster_gap_base", ClusterParams.gap_base).value),
            gap_per_m=float(p("cluster_gap_per_m", ClusterParams.gap_per_m).value),
            min_voxels=int(p("cluster_min_voxels", ClusterParams.min_voxels).value),
            max_length=float(p("cluster_max_length", ClusterParams.max_length).value),
            min_height=float(p("cluster_min_height", ClusterParams.min_height).value),
        )
        self._pub = self.create_publisher(Detection3DArray, self._out, 5)
        self.create_subscription(PointCloud2, self._in, self._cb, 5)
        self._n = self._clusters = 0
        self._ms = []
        # Steady clock: under use_sim_time a looping replay rewinds ROS time and a ROS-time timer
        # then fires on every /clock tick to catch up.
        self.create_timer(5.0, self._log, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"lidar_cluster_detector: {self._in} -> {self._out} | "
            f"{self._params.min_range:.1f}-{self._params.max_range:.0f} m, voxel "
            f"{self._params.voxel:.2f} m, gap {self._params.gap_base:.2f}+"
            f"{self._params.gap_per_m:.3f}/m | clusters SUSTAIN tracks, never birth them")

    def _cb(self, msg):
        xyz = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
        if xyz.size == 0:
            return
        # Wall clock: under use_sim_time the node clock does not advance inside a callback.
        t0 = time.perf_counter()
        clusters = cluster_nonground(np.asarray(xyz, dtype=np.float64), self._params)
        self._ms.append((time.perf_counter() - t0) * 1000.0)
        self._n += 1
        self._clusters += len(clusters)

        out = Detection3DArray()
        out.header = msg.header
        for c in clusters:
            d = Detection3D()
            d.header = msg.header
            d.sensor = Detection3D.SENSOR_LIDAR
            d.pose.position.x, d.pose.position.y = float(c.x), float(c.y)
            d.pose.orientation.w = 1.0
            d.size.x, d.size.y, d.size.z = float(c.length), float(c.width), float(c.height)
            d.size_measured = True
            d.yaw = float(c.yaw)
            d.yaw_source = Detection3D.YAW_FROM_SHAPE_FIT
            d.yaw_measured = True
            d.class_id = -1
            d.class_name = ""          # a cluster has no class, and must not pretend to
            d.score = 0.0
            d.point_count = int(c.n_points)
            out.detections.append(d)
        self._pub.publish(out)

    def _log(self):
        if not self._n:
            self.get_logger().warning(f"no sweeps on {self._in}: is ground_projection running "
                                      "with publish_nonground?")
            return
        ms = np.asarray(self._ms)
        self.get_logger().info(
            f"sweeps={self._n} clusters/sweep {self._clusters / self._n:.0f} "
            f"cluster {ms.mean():.1f} ms (p90 {np.percentile(ms, 90):.1f})")
        self._ms = self._ms[-200:]


def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
