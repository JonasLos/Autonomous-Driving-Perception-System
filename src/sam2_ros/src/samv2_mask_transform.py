#!/usr/bin/env python3
# type: ignore

import os

import message_filters
import numpy as np
import ros_numpy
import rospy
import sensor_msgs.point_cloud2 as pc2
import torch
import yaml
from scipy.spatial import KDTree
from sensor_msgs.msg import PointCloud2

from sam2_ros.msg import DetectedRoadArea
from src.configs import T1
from src.utils import crop_pointcloud, inverse_rigid_transform, timer

# Define limits
lim_x, lim_y, lim_z, pixel_lim = [2, 50], [-10, 10], [-3.5, 1], 5
lim_x, lim_y, lim_z, pixel_lim = [2, 50], [-10, 10], [-3.5, 1], 5

TOPICS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "topics.yaml"
)

# Load YAML config
with open(TOPICS_PATH, "r") as f:
    config = yaml.safe_load(f)

# === TOPICS ===
LIDAR_2D_PROJ_TOPIC = config["topics"]["transform"]["lidar_2d_projection"]
SAM_LEFT_BOUNDARY = config["topics"]["sam"]["left_boundary"]
SAM_RIGHT_BOUNDARY = config["topics"]["sam"]["right_boundary"]
SAM_LEFT_CONTOUR_TOPIC = config["topics"]["sam"]["left_contour"]
SAM_RIGHT_CONTOUR_TOPIC = config["topics"]["sam"]["right_contour"]


class Samv2MaskTransform:
    def __init__(self):

        # Publishers
        self.left_boundary_pub = rospy.Publisher(
            SAM_LEFT_BOUNDARY, PointCloud2, queue_size=1
        )
        self.right_boundary_pub = rospy.Publisher(
            SAM_RIGHT_BOUNDARY, PointCloud2, queue_size=1
        )

        # Define the point fields
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

        # Subscribers
        self.sub_proj = message_filters.Subscriber(LIDAR_2D_PROJ_TOPIC, PointCloud2)
        self.sub_left_contour = message_filters.Subscriber(
            SAM_LEFT_CONTOUR_TOPIC, DetectedRoadArea
        )
        self.sub_right_contour = message_filters.Subscriber(
            SAM_RIGHT_CONTOUR_TOPIC, DetectedRoadArea
        )

        # Synchronize topics
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_proj, self.sub_left_contour, self.sub_right_contour],
            queue_size=20,
            slop=0.6,
            allow_headerless=True,
        )
        ts.registerCallback(self.callback)

        # Variables to store incoming data
        self.msgLeftBoundary = None
        self.msgRightBoundary = None
        self.msgProj = None

        rospy.loginfo("Node initialized and timer set.")

    def callback(self, msgProj, msgLeftBoundary, msgRightBoundary):
        self.msgProj = msgProj
        self.msgLeftBoundary = msgLeftBoundary
        self.msgRightBoundary = msgRightBoundary

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

        left_boundary_3d = self.find_matching_points_kdtree(
            np.array(self.msgLeftBoundary.RoadArea.data).reshape(-1, 2), u, v, pc_arr
        )
        right_boundary_3d = self.find_matching_points_kdtree(
            np.array(self.msgRightBoundary.RoadArea.data).reshape(-1, 2), u, v, pc_arr
        )

        if right_boundary_3d.size > 0:
            self.create_cloud(
                right_boundary_3d, self.right_boundary_pub, self.msgProj.header
            )
        if left_boundary_3d.size > 0:
            self.create_cloud(
                left_boundary_3d, self.left_boundary_pub, self.msgProj.header
            )

    @timer
    def process_loop(self, event):
        left_boundary_3d = self.find_matching_points_kdtree(
            np.array(self.msgLeftBoundary.RoadArea.data).reshape(-1, 2), u, v, pc_arr
        )
        right_boundary_3d = self.find_matching_points_kdtree(
            np.array(self.msgRightBoundary.RoadArea.data).reshape(-1, 2), u, v, pc_arr
        )

        if left_boundary_3d.size > 0:
            self.create_cloud(
                left_boundary_3d, self.left_boundary_pub, self.msgProj.header
            )
        if right_boundary_3d.size > 0:
            self.create_cloud(
                right_boundary_3d, self.right_boundary_pub, self.msgProj.header
            )

    def process_pointcloud(self, msgLidar):
        pc = ros_numpy.numpify(msgLidar)
        points = np.vstack((pc["x"], pc["y"], pc["z"], np.ones(pc["x"].shape[0]))).T
        pc_arr = crop_pointcloud(points, lim_x, lim_y, lim_z)

        # Apply transformation and projection
        m1 = torch.matmul(
            torch.tensor(inverse_rigid_transform(T1)), torch.tensor(pc_arr.T)
        )
        uv1 = torch.matmul(torch.tensor(PROJ), m1)
        u, v = (uv1[:2, :] / uv1[2, :]).numpy()
        return pc_arr, u, v

    def create_cloud(self, points_3d, publisher, header):
        # Our fields expect 4 values per point (x, y, z, intensity)
        # So add a dummy intensity value of 1.0 to every point.
        intensity = np.ones((points_3d.shape[0], 1), dtype=np.float32)
        cloud_data = np.hstack((points_3d, intensity))

        # Create and publish the PointCloud2 message
        cloud_msg = pc2.create_cloud(header, self.fields, cloud_data)
        publisher.publish(cloud_msg)
        rospy.loginfo("Published point cloud with %d points.", cloud_data.shape[0])

    def find_matching_points_kdtree(self, boundary_points, u, v, pc_arr):
        tree = KDTree(np.column_stack((u, v)))
        idx = []
        for contour_point in boundary_points:
            matches = tree.query_ball_point(contour_point, pixel_lim)
            idx.extend(matches)
        return pc_arr[np.array(idx)] if idx else np.empty((0, 4))


if __name__ == "__main__":
    rospy.init_node("Samv2 Mask Transform", anonymous=True)
    Samv2MaskTransform()
    rospy.spin()
