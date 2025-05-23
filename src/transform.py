# #!/usr/bin/env python3

import numpy as np
import ros_numpy
import rospy
import sensor_msgs.point_cloud2 as pc2
import torch
import yaml
from sensor_msgs.msg import PointCloud2, PointField

from src.configs import PROJ, T1

# Path to the YAML file
TOPICS_PATH = "/home/dev/Documents/Autonomous-Driving-Perception-System/src/topics.yaml"

# Load YAML config
with open(TOPICS_PATH, "r") as f:
    topic_config = yaml.safe_load(f)

# === TOPICS ===
LIDAR_TOPIC = topic_config["topics"]["raw"]["lidar"]
LIDAR_2D_PROJ_TOPIC = topic_config["topics"]["transform"]["lidar_2d_projection"]

# Define limits
lim_x, lim_y, lim_z = [10, 80], [-10, 10], [-3.5, 1]


def inverse_rigid_transformation(arr: np.ndarray) -> np.ndarray:
    rt = arr[:3, :3].T
    tt = -np.dot(rt, arr[:3, 3])
    return np.vstack((np.column_stack((rt, tt)), [0, 0, 0, 1]))


T_vel_cam = inverse_rigid_transformation(T1)


class LidarTo2DProjection:
    def __init__(self):
        rospy.init_node("lidar_to_2d_projection")

        # Publisher
        self.projection_pub = rospy.Publisher(
            LIDAR_2D_PROJ_TOPIC, PointCloud2, queue_size=1
        )

        # Subscriber
        self.sub_lidar = rospy.Subscriber(LIDAR_TOPIC, PointCloud2, self.lidar_callback)

        rospy.loginfo("Node initialized and ready to publish 2D projections.")

    def lidar_callback(self, msgLidar):
        pc_arr, u, v = self.process_pointcloud(msgLidar)
        self.publish_projection(msgLidar.header, pc_arr, u, v)

    def process_pointcloud(self, msgLidar):
        pc = ros_numpy.numpify(msgLidar)
        points = np.vstack((pc["x"], pc["y"], pc["z"], np.ones(pc["x"].shape[0]))).T
        # pc_arr = self.crop_pointcloud(points)

        # Downsample to make it sparse
        pc_arr = self.voxel_downsample(points, voxel_size=2.0)

        # Apply transformation and projection
        m1 = torch.matmul(torch.tensor(T_vel_cam), torch.tensor(pc_arr.T))
        uv1 = torch.matmul(torch.tensor(PROJ), m1)
        u, v = (uv1[:2, :] / uv1[2, :]).numpy()
        return pc_arr, u, v

    def voxel_downsample(self, points, voxel_size=0.5):
        """Downsamples a pointcloud using voxel grid filtering."""
        # Only use x, y, z for voxelization
        coords = np.floor(points[:, :3] / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(coords, axis=0, return_index=True)
        return points[unique_indices]

    def publish_projection(self, header, pc_arr, u, v):
        # Define fields for PointCloud2 including u,v as extra fields
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("u", 12, PointField.FLOAT32, 1),
            PointField("v", 16, PointField.FLOAT32, 1),
        ]

        # Stack the data together (x,y,z,u,v)
        points = np.column_stack((pc_arr[:, :3], u, v))

        # Create PointCloud2 message
        pc2_msg = pc2.create_cloud(header, fields, points)

        self.projection_pub.publish(pc2_msg)
        rospy.loginfo(
            f"Published PointCloud2 projection with {points.shape[0]} points."
        )

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
    node = LidarTo2DProjection()
    rospy.spin()
