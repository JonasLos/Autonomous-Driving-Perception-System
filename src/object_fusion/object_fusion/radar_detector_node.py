"""Delphi ESR tracks -> the common Detection3D measurement type. Deliberately dumb.

All association intelligence lives in the aggregator. This node gates and republishes, and
its only opinions are the gate defaults -- which are not opinions but measurements:

* min_update_count MUST stay 0. The field is a never-decoded stub on this driver, 0 on every
  track in every sweep, so ANY positive threshold discards 100% of tracks.
* min_amplitude MUST stay below -10. That is the sensor's floor, not a weak return: -10 is
  the single most common value (~30% of tracks), and a naive threshold of 0.0 discards ~72%
  of everything the radar reports.

The native polar measurement is carried through untouched. A derived Cartesian pose is filled
in for convenience and for RViz, but the aggregator uses the polar form.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
import yaml
from ament_index_python.packages import get_package_share_directory

from delphi_esr_driver.msg import EsrTrackArray
from fusion_msgs.msg import Detection3D, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from radar_ros.radar_geometry import gate_tracks, polar_to_cartesian

NAN = float("nan")


class RadarDetectorNode(Node):
    def __init__(self):
        super().__init__("radar_detector")
        cfg = yaml.safe_load(
            open(get_package_share_directory("object_fusion") + "/config/topics.yaml",
                 encoding="utf-8"))
        common = yaml.safe_load(
            open(get_package_share_directory("perception_common") + "/topics.yaml",
                 encoding="utf-8"))

        self._in = common["topics"]["raw"]["radar_tracks"]
        self._out = cfg["topics"]["measurements"]["radar"]
        self._frame = self.declare_parameter("radar_frame_override", "delphi_esr_radar").value

        self._min_range = float(self.declare_parameter("min_range", 1.0).value)
        self._max_range = float(self.declare_parameter("max_range", 175.0).value)
        self._min_amplitude = float(self.declare_parameter("min_amplitude", -1e9).value)
        self._min_update_count = int(self.declare_parameter("min_update_count", 0).value)

        self._pub = self.create_publisher(Detection3DArray, self._out, 10)
        # The radar returns are the ~0.1 m range reference every camera+LiDAR position was scored
        # against, so they are worth seeing next to the boxes in RViz.
        self._markers = self.create_publisher(MarkerArray, self._out + "_markers", 10)
        self.create_subscription(EsrTrackArray, self._in, self._cb, 10)
        self._warned = False
        self.get_logger().info(f"radar_detector: {self._in} -> {self._out} "
                               f"frame={self._frame}")

    def _cb(self, msg: EsrTrackArray):
        n = len(msg.tracks)
        if n == 0:
            return

        def arr(attr, dtype=float):
            return np.fromiter((getattr(t, attr) for t in msg.tracks), dtype, n)

        keep = gate_tracks(arr("range"), arr("angle"), arr("amplitude"),
                           arr("track_status", int), arr("update_count", int),
                           min_range=self._min_range, max_range=self._max_range,
                           min_amplitude=self._min_amplitude,
                           min_update_count=self._min_update_count)

        out = Detection3DArray()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = self._frame
            if not self._warned:
                self._warned = True
                self.get_logger().info(
                    f"radar messages carry an empty frame_id (the driver sets one only on its "
                    f"markers); assuming '{self._frame}'")

        rng, az = arr("range")[keep], arr("angle")[keep]
        rr, amp = arr("range_rate")[keep], arr("amplitude")[keep]
        tid = arr("track_id", int)[keep]
        xs, ys = polar_to_cartesian(rng, az)

        for i in range(int(keep.sum())):
            d = Detection3D()
            d.header = out.header
            d.sensor = Detection3D.SENSOR_RADAR
            d.pose.position.x = float(xs[i])
            d.pose.position.y = float(ys[i])
            # The ESR has no elevation channel; z is UNKNOWN, not zero. Left at 0 with an
            # effectively infinite variance below rather than guessed.
            d.pose.orientation.w = 1.0
            cov = [0.0] * 9
            a = math.radians(float(az[i]))
            u = np.array([math.cos(a), math.sin(a)])
            nvec = np.array([-u[1], u[0]])
            B = np.column_stack((u, nvec))
            C = B @ np.diag([0.1 ** 2, (float(rng[i]) * math.radians(1.0)) ** 2]) @ B.T
            cov[0], cov[1], cov[3], cov[4] = C[0, 0], C[0, 1], C[1, 0], C[1, 1]
            cov[8] = 1e6                      # z unobserved
            d.position_covariance = cov
            d.size_measured = False
            d.yaw_measured = False
            d.yaw_source = Detection3D.YAW_FROM_RAY_DEFAULT
            d.class_id = -1
            d.class_name = "radar_track"
            d.score = 0.0
            d.tracker_id = ""
            d.radar_range = float(rng[i])
            d.radar_range_rate = float(rr[i])
            d.radar_azimuth_deg = float(az[i])
            d.radar_amplitude = float(amp[i])
            d.radar_track_id = int(tid[i])
            d.point_count = 0
            out.detections.append(d)

        self._pub.publish(out)

        arr = MarkerArray()
        clear = Marker()
        clear.header = out.header
        clear.ns = "radar"
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, d in enumerate(out.detections):
            m = Marker()
            m.header = out.header
            m.ns = "radar"
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = d.pose
            m.pose.position.z = -1.2          # no elevation channel; drawn near bumper height
            m.scale.x = m.scale.y = m.scale.z = 0.8
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.45, 0.10, 0.9
            m.lifetime.nanosec = 150_000_000
            arr.markers.append(m)
        self._markers.publish(arr)


def main():
    rclpy.init()
    node = RadarDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
