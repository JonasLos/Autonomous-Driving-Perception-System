#!/usr/bin/env python3
# type: ignore

import os

from ament_index_python.packages import get_package_share_directory
import message_filters
import numpy as np
import ros2_numpy as ros_numpy
import rclpy
from rclpy.node import Node
from sensor_msgs_py import point_cloud2 as pc2
import yaml
from scipy.spatial import KDTree
from sensor_msgs.msg import PointCloud2, PointField

from clrernet_msgs.msg import LanePoints
from perception_common.utils import timer

TOPICS_PATH = os.path.join(
    get_package_share_directory("perception_common"), "topics.yaml"
)
with open(TOPICS_PATH, "r") as f:
    config = yaml.safe_load(f)

CLRERNET_ALL_LANES_TOPIC = config["topics"]["clrernet"]["all_lanes"]
CLRERNET_LEFT_LANE_TOPIC = config["topics"]["clrernet"]["left_lane"]
CLRERNET_RIGHT_LANE_TOPIC = config["topics"]["clrernet"]["right_lane"]
CLRERNET_CENTERLINE_TOPIC = config["topics"]["clrernet"]["centerline"]
LIDAR_2D_PROJ_TOPIC = config["topics"]["transform"]["lidar_2d_projection"]

PIXEL_LIM = 10
LIDAR_ORIGIN = np.array([0.0, 0.0, 0.0])


class Clrernet_Lane_Transform(Node):
    def __init__(self):
        super().__init__("clrernet_lane_transform")

        self.fields = [
            PointField(
                name="x", offset=0, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="y", offset=4, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="z", offset=8, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        self.prev_centerline = None
        self.prev_left_lane = None
        self.prev_right_lane = None
        self.ema_alpha = 0.5  # EMA smoothing factor
        self.latest_lanes_msg = None

        # Use latest-message fusion instead of timestamp sync because sources can
        # come from different time domains (bag time vs wall clock).
        self.create_subscription(PointCloud2, LIDAR_2D_PROJ_TOPIC, self.proj_callback, 10)
        self.create_subscription(LanePoints, CLRERNET_ALL_LANES_TOPIC, self.lanes_msg_callback, 10)

        # Publishers
        self.left_pcl_pub = self.create_publisher(PointCloud2, CLRERNET_LEFT_LANE_TOPIC, 5)
        self.right_pcl_pub = self.create_publisher(PointCloud2, CLRERNET_RIGHT_LANE_TOPIC, 5)
        self.centerline_pub = self.create_publisher(PointCloud2, CLRERNET_CENTERLINE_TOPIC, 1)

    def lanes_msg_callback(self, msg_all_lanes):
        self.latest_lanes_msg = msg_all_lanes

    def proj_callback(self, msg_proj):
        if self.latest_lanes_msg is None:
            return
        self.lanes_callback(msg_proj, self.latest_lanes_msg)

    @timer
    def lanes_callback(self, msg_proj, msg_all_lanes):
        print("Received synchronized messages")
        pc_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(
            msg_proj, remove_nans=True
        )
        points_list = list(
            pc2.read_points(
                msg_proj, field_names=("x", "y", "z", "u", "v"), skip_nans=True
            )
        )
        if len(points_list) == 0:
            self.get_logger().warning("No points in projected PointCloud2")
            return

        points_np = np.array(points_list)
        if points_np.dtype.fields is not None:
            u = points_np["u"]
            v = points_np["v"]
        else:
            points_np = np.atleast_2d(points_np)
            if points_np.shape[1] < 5:
                self.get_logger().warning("Projected PointCloud2 missing u/v fields")
                return
            u = points_np[:, 3]
            v = points_np[:, 4]

        lane_dict = {}
        for pt in msg_all_lanes.points:
            lane_dict.setdefault(pt.lane_id, []).append([pt.x, pt.y])

        lanes = [np.array(pts) for _, pts in sorted(lane_dict.items())]
        if len(lanes) == 0:
            self.get_logger().warning("No lane points detected!")
            return

        left_pts, right_pts, center_pts = self.get_closest_lane_pair_3d(
            lanes, u, v, pc_arr, LIDAR_ORIGIN
        )
        if len(left_pts) == 0 or len(right_pts) == 0:
            self.get_logger().warning("Unable to find left or right lane boundaries.")
        if center_pts.shape[0] == 0:
            self.get_logger().warning("Centerline could not be computed.")

        self.publish_3d_lane(
            left_pts, u, v, pc_arr, self.left_pcl_pub, msg_proj.header, side="left"
        )
        self.publish_3d_lane(
            right_pts, u, v, pc_arr, self.right_pcl_pub, msg_proj.header, side="right"
        )
        self.publish_centerline(center_pts, msg_proj.header)

    def get_closest_lane_pair_3d(self, lanes, u, v, pc_arr, lidar_origin):
        print("Finding closest lane pair in 3D")
        tree = KDTree(np.column_stack((u, v)))

        def transform_lane_to_3d(lane_uv, tree):
            matched_pts = []
            dist, idx = tree.query(lane_uv)  # Vectorized nearest neighbors
            mask = dist < PIXEL_LIM
            matched_pts = pc_arr[idx[mask]]
            if matched_pts.shape[0] == 0:
                self.get_logger().warning("No 3D match found for a lane")
            return matched_pts if matched_pts.shape[0] > 0 else np.empty((0, 3))

        def compute_centerline(pts1, pts2):
            min_len = min(len(pts1), len(pts2))
            if min_len == 0:
                return np.empty((0, 3))
            return (pts1[:min_len] + pts2[:min_len]) / 2.0

        lanes_3d = []
        for lane in lanes:
            lane_3d = transform_lane_to_3d(lane, tree)
            if lane_3d.shape[0] > 0:
                mean_y = np.mean(lane_3d[:, 1])
                lanes_3d.append((lane, lane_3d, mean_y))

        if len(lanes_3d) < 2:
            self.get_logger().warning("Less than 2 valid 3D lanes detected. Cannot compute centerline.")
            return ([], [], np.empty((0, 3)))

        lanes_3d.sort(key=lambda tup: tup[2])
        best_pair = ([], [])
        best_centerline = np.empty((0, 3))
        min_dist = float("inf")

        for i in range(len(lanes_3d) - 1):
            lane1_uv, lane1_3d, _ = lanes_3d[i]
            lane2_uv, lane2_3d, _ = lanes_3d[i + 1]

            centerline_pts = compute_centerline(lane1_3d, lane2_3d)
            if centerline_pts.shape[0] == 0:
                continue

            center_mean = centerline_pts[:10].mean(axis=0)
            dist = np.linalg.norm(center_mean[1] - lidar_origin[1])

            if dist < min_dist:
                min_dist = dist
                if np.mean(lane1_3d[:, 1]) > np.mean(lane2_3d[:, 1]):
                    best_pair = (lane1_uv, lane2_uv)
                else:
                    best_pair = (lane2_uv, lane1_uv)
                best_centerline = centerline_pts

        return best_pair[0], best_pair[1], best_centerline

    def publish_3d_lane(self, lane_uv, u, v, pc_arr, publisher, header, side="left"):
        if len(lane_uv) == 0:
            return

        tree = KDTree(np.column_stack((u, v)))
        matched_pts = []

        for pt in lane_uv:
            dist, idx = tree.query(pt)
            if dist < PIXEL_LIM:
                matched_pts.append(pc_arr[idx])

        matched = np.array(matched_pts) if matched_pts else np.empty((0, 3))
        if matched.shape[0] > 0:
            intensity = np.ones((matched.shape[0], 1), dtype=np.float32)
            cloud_data = np.hstack((matched, intensity))
            cloud_msg = pc2.create_cloud(header, self.fields, cloud_data)
            publisher.publish(cloud_msg)

    def publish_centerline(self, center_pts, header):
        if center_pts.shape[0] == 0:
            return

        # === Apply Temporal Smoothing ===
        if (
            self.prev_centerline is not None
            and self.prev_centerline.shape == center_pts.shape
        ):
            center_pts = (
                self.ema_alpha * center_pts
                + (1 - self.ema_alpha) * self.prev_centerline
            )
        self.prev_centerline = center_pts.copy()

        intensity = np.ones((center_pts.shape[0], 1), dtype=np.float32)
        cloud_data = np.hstack((center_pts, intensity))
        cloud_msg = pc2.create_cloud(header, self.fields, cloud_data)
        self.centerline_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Clrernet_Lane_Transform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
