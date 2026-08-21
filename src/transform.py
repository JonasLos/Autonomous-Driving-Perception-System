# #!/usr/bin/env python3

"""Projects each LiDAR sweep into camera pixels and republishes it as x,y,z,u,v.

This is the hub every camera-driven node pairs against, so the one rule here is that a sweep
is never dropped: a missing or mismatched camera_info degrades the geometry used for the
pixel bounds, it does not stop the projection. Everything downstream would go dark with it.

camera_info is matched to each sweep by capture time rather than "whatever arrived last", so
a stalled or divergent camera_info stream is reported instead of being silently applied.
wait_for_newer defaults to 0.0 here, unlike the downstream fusion nodes: deferring a sweep to
wait for camera_info would add latency to /lidar_2d_projection itself, which lands in every
consumer's budget.
"""

import os

import numpy as np
import ros2_numpy as ros_numpy
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
try:
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError:  # Fallback for environments exposing the old module path
    import sensor_msgs.point_cloud2 as pc2
import torch
import yaml
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField

from perception_common.configs import PROJ, T1
from perception_common.stamp_sync import StampMatchedBuffer, apply_bounded_parameters
from perception_common.utils import (
    crop_pointcloud,
    inverse_rigid_transform,
    timer,
)

try:
    from ament_index_python.packages import get_package_share_directory

    TOPICS_PATH = os.path.join(
        get_package_share_directory("perception_common"), "topics.yaml"
    )
except Exception:  # noqa: BLE001 - running from a bare checkout, no colcon install
    TOPICS_PATH = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "perception_common",
        "topics.yaml",
    )

# Load YAML config
with open(TOPICS_PATH, "r") as f:
    topic_config = yaml.safe_load(f)

# === TOPICS ===
LIDAR_TOPIC = topic_config["topics"]["raw"]["lidar_tc"]
LIDAR_2D_PROJ_TOPIC = topic_config["topics"]["transform"]["lidar_2d_projection"]
CAMERA_INFO_TOPIC = topic_config["topics"]["raw"]["camera_info"]

# Define limits
lim_x, lim_y, lim_z = [0, 100], [-20, 20], [-3.5, 1]


class LidarTo2DProjection(Node):
    def __init__(self):
        super().__init__("lidar_to_2d_projection")

        # Fallback geometry from the calibration, used until camera_info is matched and
        # whenever it goes missing. The camera is fixed, so these are the right numbers --
        # matching camera_info exists to detect a divergent or dead stream, not to track a
        # resolution that actually changes.
        self.image_width = int(round(float(PROJ[0, 2]) * 2.0))
        self.image_height = int(round(float(PROJ[1, 2]) * 2.0))

        # A camera_info this far from the sweep is not describing the same moment. Generous
        # compared to the fusion nodes' bound because camera_info is low-rate and its content
        # is near-static; the point is to catch a stream that has stopped or jumped.
        camera_info_max_skew = float(
            self.declare_parameter("camera_info_max_skew", 0.2)
            .get_parameter_value()
            .double_value
        )
        # 0.0 on purpose. Deferring a sweep to wait for camera_info would add that latency to
        # /lidar_2d_projection, which every downstream node then inherits -- and would widen
        # the very camera-vs-LiDAR phase gap the fusion nodes defer to close. Raise it only
        # after measuring `ros2 topic delay /lidar_2d_projection`.
        camera_info_wait_for_newer = float(
            self.declare_parameter("camera_info_wait_for_newer", 0.0)
            .get_parameter_value()
            .double_value
        )

        self._camera_info = StampMatchedBuffer(
            "camera_info",
            buffer_duration=2.0,
            max_skew=max(0.0, camera_info_max_skew),
            wait_for_newer=max(0.0, camera_info_wait_for_newer),
            wrap=lambda m: m,
        )
        self._last_camera_info_log = None

        # Publisher
        self.projection_pub = self.create_publisher(PointCloud2, LIDAR_2D_PROJ_TOPIC, 1)

        # Subscriber
        self.create_subscription(PointCloud2, LIDAR_TOPIC, self.lidar_callback, 1)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 1)

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"Node initialized and ready to publish 2D projections. "
            f"camera_info_max_skew={self._camera_info.max_skew:.3f}s "
            f"camera_info_wait_for_newer={self._camera_info.wait_for_newer:.3f}s "
            f"fallback geometry={self.image_width}x{self.image_height} "
            f"use_sim_time={self.get_parameter('use_sim_time').value}"
        )

    def _now(self):
        """Seconds on the node clock -- sim time when use_sim_time is set, else wall time."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_set_parameters(self, params):
        targets = {
            "camera_info_max_skew": (self._camera_info, "max_skew"),
            "camera_info_wait_for_newer": (self._camera_info, "wait_for_newer"),
        }
        ok, reason, applied = apply_bounded_parameters(params, targets)
        if not ok:
            return SetParametersResult(successful=False, reason=reason)
        for name, value in applied:
            self.get_logger().info(f"{name} set to {value:.3f}s")
        return SetParametersResult(successful=True)

    def camera_info_callback(self, msg):
        if msg.width > 0 and msg.height > 0:
            self._camera_info.add(msg)

    def lidar_callback(self, msgLidar):
        self._update_geometry(msgLidar.header)
        pc_arr, u, v = self.process_pointcloud(msgLidar)
        self.publish_projection(msgLidar.header, pc_arr, u, v)

    def _update_geometry(self, header):
        """Adopt the camera_info captured nearest this sweep, or keep the last known.

        A sweep is never dropped over camera_info: this is the hub the whole camera side of
        the stack pairs against, and going silent here takes every downstream node with it.
        An unmatched camera_info only means the pixel bounds come from the previous frame or
        from the calibration, which for a fixed camera is the same answer.
        """
        pairing = self._camera_info.match(header, now=self._now())
        if pairing.value is not None:
            self.image_width = int(pairing.value.width)
            self.image_height = int(pairing.value.height)
            return
        self._log_camera_info_gap(pairing.skew, pairing.reason)

    def _log_camera_info_gap(self, skew, reason):
        """Rate-limited, so a dead camera_info stream is visible without flooding."""
        now = self._now()
        if self._last_camera_info_log is not None and now - self._last_camera_info_log < 5.0:
            return
        self._last_camera_info_log = now
        self.get_logger().warning(
            f"Using {self.image_width}x{self.image_height} without a matched camera_info: "
            f"{self._camera_info.describe_unmatched(skew, 'lidar sweep', reason=reason)}"
        )

    @timer
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

        # Filter projected points against the active camera geometry.
        mask = (u > 0) & (u < self.image_width) & (v > 0) & (v < self.image_height)
        pc_arr = pc_arr[mask]
        u, v = u[mask], v[mask]

        return pc_arr, u, v

    @timer
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
