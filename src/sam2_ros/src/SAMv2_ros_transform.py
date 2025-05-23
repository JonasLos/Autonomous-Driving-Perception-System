#!/usr/bin/env python3

from functools import wraps

import message_filters
import numpy as np

np.float = np.float64
import ros_numpy
import rospy
import sensor_msgs.point_cloud2 as pc2
import torch
from scipy.spatial import KDTree
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray

from sam2_ros.msg import DetectedRoadArea
from src.configs import (
    LIDAR_TOPIC,
    PROJ,
    LIDAR_2D_PROJ_TOPIC,
    SAM_LEFT_BOUNDARY,
    SAM_LEFT_CONTOUR_TOPIC,
    SAM_RIGHT_BOUNDARY,
    SAM_RIGHT_CONTOUR_TOPIC,
    T1,
)

# Define limits
lim_x, lim_y, lim_z, pixel_lim = [2, 50], [-10, 10], [-3.5, 1], 5
lim_x, lim_y, lim_z, pixel_lim = [2, 50], [-10, 10], [-3.5, 1], 5


def inverse_rigid_transformation(arr: np.ndarray) -> np.ndarray:
    Rt = arr[:3, :3].T
    tt = -np.dot(Rt, arr[:3, 3])
    return np.vstack((np.column_stack((Rt, tt)), [0, 0, 0, 1]))


T_vel_cam = inverse_rigid_transformation(T1)


def timer(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = rospy.Time.now().to_sec()
        result = func(*args, **kwargs)
        end_time = rospy.Time.now().to_sec()
        print(f"{func.__name__} executed in {end_time - start_time:.4f} seconds")
        return result

    return wrapper


class RoadSegmentation3D:
    def __init__(self):
        rospy.init_node("segmentationTO3d")

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
        # self.sub_lidar = message_filters.Subscriber(LIDAR_TOPIC, PointCloud2)
        self.sub_proj = message_filters.Subscriber(LIDAR_2D_PROJ_TOPIC, PointCloud2)
        self.sub_left_contour = message_filters.Subscriber(
            SAM_LEFT_CONTOUR_TOPIC, DetectedRoadArea
        )
        self.sub_right_contour = message_filters.Subscriber(
            SAM_RIGHT_CONTOUR_TOPIC, DetectedRoadArea
        )
        
        # Synchronize topics
        ts = message_filters.ApproximateTimeSynchronizer(
            # [self.sub_lidar, self.sub_left_contour, self.sub_right_contour],
            [self.sub_proj, self.sub_left_contour, self.sub_right_contour],
            queue_size=10,
            slop=0.3,
            allow_headerless=True,
        )
        ts.registerCallback(self.callback)

        # Variables to store incoming data
        # self.msgLidar = None
        self.msgLeftBoundary = None
        self.msgRightBoundary = None
        self.msgProj = None

        # Timer to trigger processing loop
        # rospy.Timer(rospy.Duration(0.1), self.process_loop)
        rospy.loginfo("Node initialized and timer set.")

    # def callback(self, msgLidar,  msgLeftBoundary, msgRightBoundary):
    def callback(self, msgProj, msgLeftBoundary, msgRightBoundary):
        # self.msgLidar = msgLidar
        self.msgProj = msgProj
        self.msgLeftBoundary = msgLeftBoundary
        self.msgRightBoundary = msgRightBoundary

        # Parse projected u,v from Float32MultiArray
        # proj_data = np.array(self.msgProj.data).reshape(-1, 5)
        # pc_arr = np.column_stack((proj_data[:, 0], proj_data[:, 1], proj_data[:, 2], np.ones(proj_data.shape[0])))
        # u, v = proj_data[:, 0], proj_data[:, 1]


        # Convert PointCloud2 projected points to numpy structured array
        pc_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msgProj, remove_nans=True)

        # We expect msgProj points to have 5 floats per point: x,y,z,u,v
        # ros_numpy returns only x,y,z by default; so we need a custom approach to extract u,v

        # Alternative: extract all fields including u,v using raw pointcloud2 reading:
        points_list = list(pc2.read_points(msgProj, field_names=("x","y","z","u","v"), skip_nans=True))
        if len(points_list) == 0:
            rospy.logwarn("No points in projected PointCloud2")
            return

        points_np = np.array(points_list)
        xyz = points_np[:, 0:3]
        u = points_np[:, 3]
        v = points_np[:, 4]

        left_boundary_3d = self.find_matching_points_kdtree(
            np.array(self.msgLeftBoundary.RoadArea.data).reshape(-1, 2), u, v, pc_arr
        )
        right_boundary_3d = self.find_matching_points_kdtree(
            np.array(self.msgRightBoundary.RoadArea.data).reshape(-1, 2), u, v, pc_arr
        )

        if left_boundary_3d.size > 0:
            self.create_cloud(left_boundary_3d, self.left_boundary_pub, self.msgProj.header)
        if right_boundary_3d.size > 0:
            self.create_cloud(right_boundary_3d, self.right_boundary_pub, self.msgProj.header)


    @timer
    def process_loop(self, event):
        # if not self.msgLidar or not self.msgLeftBoundary or not self.msgRightBoundary:
        # if not self.msgLeftBoundary or not self.msgRightBoundary or not self.msgProj:
        #     # print(self.msgLeftBoundary, self.msgRightBoundary, self.msgLidar)
        #     print(self.msgLeftBoundary, self.msgRightBoundary, self.msgProj)
        #     print("Some topic is missing")
        #     return

        # if self.msgProj is None or self.msgLeftBoundary is None or self.msgRightBoundary is None:
        #     rospy.logwarn_throttle(5, "Waiting for synchronized messages...")
        #     return


        # pc_arr, u, v = self.process_pointcloud(self.msgLidar)

        # Parse projected u,v from Float32MultiArray
        # proj_data = np.array(self.msgProj.data).reshape(-1, 5)
        # pc_arr = np.column_stack((proj_data[:, 0], proj_data[:, 1], proj_data[:, 2], np.ones(proj_data.shape[0])))
        # u, v = proj_data[:, 0], proj_data[:, 1]

        left_boundary_3d = self.find_matching_points_kdtree(
            np.array(self.msgLeftBoundary.RoadArea.data).reshape(-1, 2), u, v, pc_arr
        )
        right_boundary_3d = self.find_matching_points_kdtree(
            np.array(self.msgRightBoundary.RoadArea.data).reshape(-1, 2), u, v, pc_arr
        )

        if left_boundary_3d.size > 0:
            self.create_cloud(left_boundary_3d, self.left_boundary_pub, self.msgProj.header)
        if right_boundary_3d.size > 0:
            self.create_cloud(right_boundary_3d, self.right_boundary_pub, self.msgProj.header)

    def process_pointcloud(self, msgLidar):
        pc = ros_numpy.numpify(msgLidar)
        points = np.vstack((pc["x"], pc["y"], pc["z"], np.ones(pc["x"].shape[0]))).T
        pc_arr = self.crop_pointcloud(points)

        # Apply transformation and projection
        m1 = torch.matmul(torch.tensor(T_vel_cam), torch.tensor(pc_arr.T))
        uv1 = torch.matmul(torch.tensor(PROJ), m1)
        u, v = (uv1[:2, :] / uv1[2, :]).numpy()
        return pc_arr, u, v

    def create_cloud(self, points_3d, publisher, header):
        # header = msgLidar.header
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

    def crop_pointcloud(self, pointcloud):
        mask = (
            (pointcloud[:, 0] >= lim_x[0])
            & (pointcloud[:, 0] <= lim_x[1])
            & (pointcloud[:, 1] >= lim_y[0])
            & (pointcloud[:, 1] <= lim_y[1])
            & (pointcloud[:, 2] >= lim_z[0])
            & (pointcloud[:, 2] <= lim_z[1])
        )
        return pointcloud[mask]


if __name__ == "__main__":
    RoadSegmentation3D()
    rospy.spin()
