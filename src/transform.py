# #!/usr/bin/env python3

import os

import cython
import numpy as np
import ros2_numpy as ros_numpy
import rclpy
from rclpy.node import Node
import sensor_msgs.point_cloud2 as pc2
import torch
import yaml
from sensor_msgs.msg import PointCloud2, PointField

from src.configs import PROJ, T1
from src.utils import crop_pointcloud, inverse_rigid_transform, timer

# Path to the YAML file
TOPICS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "topics.yaml")

# Load YAML config
with open(TOPICS_PATH, "r") as f:
    topic_config = yaml.safe_load(f)

# === TOPICS ===
LIDAR_TOPIC = topic_config["topics"]["raw"]["lidar_tc"]
LIDAR_2D_PROJ_TOPIC = topic_config["topics"]["transform"]["lidar_2d_projection"]
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

# Define limits
lim_x, lim_y, lim_z = [0, 100], [-10, 10], [-3.5, 1]


class LidarTo2DProjection(Node):
    def __init__(self):
        super().__init__("lidar_to_2d_projection")

        # Publisher
        self.projection_pub = self.create_publisher(PointCloud2, LIDAR_2D_PROJ_TOPIC, 1)

        # Subscriber
        self.create_subscription(PointCloud2, LIDAR_TOPIC, self.lidar_callback, 1)

        self.get_logger().info("Node initialized and ready to publish 2D projections.")

    def lidar_callback(self, msgLidar):
        pc_arr, u, v = self.process_pointcloud(msgLidar)
        self.publish_projection(msgLidar.header, pc_arr, u, v)

    @timer
    @cython.inline
    def process_pointcloud(self, msgLidar):
        pc = ros_numpy.numpify(msgLidar)
        pc_arr = np.vstack((pc["x"], pc["y"], pc["z"], np.ones(pc["x"].shape[0]))).T
        pc_arr = crop_pointcloud(pc_arr, lim_x, lim_y, lim_z)

        # Downsample to make it sparse
        pc_arr = self.voxel_downsample(pc_arr, voxel_size=0.1)

        # Apply transformation and projection
        m1 = torch.matmul(
            torch.tensor(inverse_rigid_transform(T1)), torch.tensor(pc_arr.T)
        )
        uv1 = torch.matmul(torch.tensor(PROJ), m1)
        u, v = (uv1[:2, :] / uv1[2, :]).numpy()
        print(u.max(), u.min(), v.max(), v.min())

        # Correct element-wise filtering
        mask = (u > 0) & (u < 2048) & (v > 0) & (v < 1544)
        pc_arr = pc_arr[mask]
        u, v = u[mask], v[mask]

        return pc_arr, u, v

    @timer
    @cython.inline
    def voxel_downsample(self, points, voxel_size):
        """Downsamples a pointcloud using voxel grid filtering."""
        # Only use x, y, z for voxelization
        coords = np.floor(points[:, :3] / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(coords, axis=0, return_index=True)
        return points[unique_indices]

    @timer
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
        self.get_logger().info(f"Published PointCloud2 projection with {points.shape[0]} points.")


if __name__ == "__main__":
    rclpy.init()
    node = LidarTo2DProjection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
