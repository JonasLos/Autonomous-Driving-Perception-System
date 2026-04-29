# #!/usr/bin/env python3

import os

import cython
import numpy as np
import ros2_numpy as ros_numpy
import rclpy
from rclpy.node import Node
try:
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError:  # Fallback for environments exposing the old module path
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
        # Flatten organized/unorganized clouds into a consistent Nx4 array.
        x = np.asarray(pc["x"]).reshape(-1)
        y = np.asarray(pc["y"]).reshape(-1)
        z = np.asarray(pc["z"]).reshape(-1)
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[valid], y[valid], z[valid]
        pc_arr = np.column_stack((x, y, z, np.ones(x.shape[0], dtype=np.float32)))
        pc_arr = crop_pointcloud(pc_arr, lim_x, lim_y, lim_z)

        if pc_arr.size == 0:
            return pc_arr, np.array([], dtype=np.float32), np.array([], dtype=np.float32)

        # Downsample to make it sparse
        pc_arr = self.voxel_downsample(pc_arr, voxel_size=0.1)

        # Apply transformation and projection
        t_inv = torch.as_tensor(inverse_rigid_transform(T1), dtype=torch.float32)
        pc_t = torch.as_tensor(pc_arr.T, dtype=torch.float32)
        proj = torch.as_tensor(PROJ, dtype=torch.float32)
        m1 = torch.matmul(t_inv, pc_t)
        uv1 = torch.matmul(proj, m1)
        u, v = (uv1[:2, :] / uv1[2, :]).numpy()

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
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="u", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="v", offset=16, datatype=PointField.FLOAT32, count=1),
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
