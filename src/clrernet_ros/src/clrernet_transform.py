#!/usr/bin/env python3

import os

import message_filters
import numpy as np
import ros_numpy
import rospy
import sensor_msgs.point_cloud2 as pc2
import yaml
from scipy.spatial import KDTree
from sensor_msgs.msg import PointCloud2

from clrernet_ros.msg import LanePoints

# from src.utils import timer

pixel_lim = 10  # pixels

TOPICS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "topics.yaml"
)

# Load YAML config
with open(TOPICS_PATH, "r") as f:
    config = yaml.safe_load(f)

# === TOPICS ===
LIDAR_2D_PROJ_TOPIC = config["topics"]["transform"]["lidar_2d_projection"]
LEFT_LANE_TOPIC = config["topics"]["lane_detection"]["left_lane"]
RIGHT_LANE_TOPIC = config["topics"]["lane_detection"]["right_lane"]
LEFT_LANE_BOUNDARY_TOPIC = config["topics"]["lane_detection"]["left_lane_boundary"]
RIGHT_LANE_BOUNDARY_TOPIC = config["topics"]["lane_detection"]["right_lane_boundary"]


class LaneTo3D:
    def __init__(self):
        # Subscribers
        self.sub_proj = message_filters.Subscriber(LIDAR_2D_PROJ_TOPIC, PointCloud2)
        # self.sub_lanes = message_filters.Subscriber(LEFT_LANE_TOPIC, LanePoints)
        self.left_pointSub = message_filters.Subscriber(LEFT_LANE_TOPIC, LanePoints)
        self.right_pointSub = message_filters.Subscriber(RIGHT_LANE_TOPIC, LanePoints)

        # Publishers
        self.left_pcl_pub = rospy.Publisher(
            LEFT_LANE_BOUNDARY_TOPIC, PointCloud2, queue_size=1
        )
        self.right_pcl_pub = rospy.Publisher(
            RIGHT_LANE_BOUNDARY_TOPIC, PointCloud2, queue_size=1
        )

        self.lane_3d_pub = rospy.Publisher(
            LEFT_LANE_BOUNDARY_TOPIC, PointCloud2, queue_size=1
        )
        # Synchronize topics
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_proj, self.left_pointSub, self.right_pointSub],
            queue_size=10,
            slop=0.5,
            allow_headerless=True,
        )
        ts.registerCallback(self.lanes_callback)

        self.fields = [
            pc2.PointField(
                name="x", offset=0, datatype=pc2.PointField.FLOAT32, count=1
            ),
            pc2.PointField(
                name="y", offset=4, datatype=pc2.PointField.FLOAT32, count=1
            ),
            pc2.PointField(
                name="z", offset=8, datatype=pc2.PointField.FLOAT32, count=1
            ),
            pc2.PointField(
                name="intensity", offset=12, datatype=pc2.PointField.FLOAT32, count=1
            ),
        ]

    # @timer
    def lanes_callback(self, msgProj, msgLeftPoint: LanePoints, msgRightPoint):
        # self.msgProj = msgProj
        # self.msgLane = msgLane

        # Convert PointCloud2 projected points to numpy structured array
        pc_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(
            msgProj, remove_nans=True
        )
        points_list = list(
            pc2.read_points(
                msgProj, field_names=("x", "y", "z", "u", "v"), skip_nans=True
            )
        )
        if len(points_list) == 0:
            rospy.logwarn("No points in projected PointCloud2")
            return

        points_np = np.array(points_list)
        u = points_np[:, 3]
        v = points_np[:, 4]

        self.get_lane_points(
            msgLeftPoint, u, v, pc_arr, self.left_pcl_pub, msgProj.header
        )
        self.get_lane_points(
            msgRightPoint, u, v, pc_arr, self.right_pcl_pub, msgProj.header
        )

    def get_lane_points(self, msgLane, u, v, pc_arr, publisher, header):
        if not msgLane or not msgLane.points:
            rospy.logwarn("No lane points received")
            return
        lane_uv = np.array([[p.x, p.y] for p in msgLane.points])
        lanes_3d = self.find_matching_points_kdtree(lane_uv, u, v, pc_arr)
        # print(lane_uv.shape, lanes_3d.shape, "uv and lanes shape")
        if lanes_3d.size > 0:
            self.create_cloud(lanes_3d, publisher, header)

        rospy.loginfo(f"Published {lanes_3d.shape[0]} 3D lane points.")

    def find_matching_points_kdtree(self, boundary_points, u, v, pc_arr):
        tree = KDTree(np.column_stack((u, v)))
        idx = []
        for contour_point in boundary_points:
            matches = tree.query_ball_point(contour_point, pixel_lim)
            idx.extend(matches)
        return pc_arr[np.array(idx)] if idx else np.empty((0, 4))

    def create_cloud(self, points_3d, publisher, header):
        # Our fields expect 4 values per point (x, y, z, intensity)
        # So add a dummy intensity value of 1.0 to every point.
        intensity = np.ones((points_3d.shape[0], 1), dtype=np.float32)
        cloud_data = np.hstack((points_3d, intensity))

        # Create and publish the PointCloud2 message
        cloud_msg = pc2.create_cloud(header, self.fields, cloud_data)
        publisher.publish(cloud_msg)
        rospy.loginfo("Published point cloud with %d points.", cloud_data.shape[0])


if __name__ == "__main__":
    rospy.init_node("clrernet_lane_to_3d", anonymous=True)
    LaneTo3D()
    rospy.spin()
