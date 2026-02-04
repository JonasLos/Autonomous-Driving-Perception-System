#!/usr/bin/env python3
# type: ignore

import os
import sys

from ament_index_python.packages import get_package_share_directory
import numpy as np
import ros2_numpy as ros_numpy
import rclpy
from rclpy.node import Node
import sensor_msgs.point_cloud2 as pc2
import spconv.pytorch as spconv
import torch
import yaml
from semantic_kitti_ros import SemanticKITTI
from sensor_msgs.msg import PointCloud2, PointField
from sklearn.cluster import DBSCAN
from perception_common.utils import crop_pointcloud, timer

PKG_SHARE = get_package_share_directory("sphereformer_ros")
if PKG_SHARE not in sys.path:
    sys.path.insert(0, PKG_SHARE)

from SphereFormer.util import config
from SphereFormer_changes.unet_spherical_transformer import Semantic as Model

CONFIG_PATH = os.path.join(
    PKG_SHARE,
    "SphereFormer_changes",
    "semantic_kitti_unet32_spherical_transformer.yaml",
)
CHECKPOINT_PATH = os.path.join(PKG_SHARE, "SphereFormer", "model_semantic_kitti.pth")

# Path to the YAML file
TOPICS_PATH = os.path.join(get_package_share_directory("perception_common"), "topics.yaml")

# Load YAML config
with open(TOPICS_PATH, "r") as f:
    topic_config = yaml.safe_load(f)

# === TOPICS ===
SPHEREFORMER_SEGMENTATION_TOPIC = topic_config["topics"]["sphereformer"][
    "segmentation_mask"
]
SPHEREFORMER_LEFT_BOUNDARY = topic_config["topics"]["sphereformer"]["left_boundary"]
SPHEREFORMER_RIGHT_BOUNDARY = topic_config["topics"]["sphereformer"]["right_boundary"]
RING_FILTERED_POINTS_TOPIC = topic_config["topics"]["sphereformer"]["ring_filtered"]
LIDAR_2D_PROJ_TOPIC = topic_config["topics"]["transform"]["lidar_2d_projection"]

lim_x, lim_y, lim_z = [-30, 100], [-10, 10], [-5, 0]

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
torch.cuda.set_per_process_memory_fraction(0.3, device)


class SphereformerLidarSegmentation(Node):
    def __init__(
        self,
        config_path,
        checkpoint_path,
    ):
        super().__init__("sphereformer_lidar_segmentation")
        # Configuration and model initialization
        self.config_path = config_path
        self.checkpoint_path = checkpoint_path
        self.cfg = config.load_cfg_from_cfg_file(self.config_path)
        self.model = self._load_model()
        self.semkitti_dataset = SemanticKITTI(split="val")

        # ROS2 publishers and subscribers
        self.pub = self.create_publisher(PointCloud2, SPHEREFORMER_SEGMENTATION_TOPIC, 1)
        self.left_boundary_pub = self.create_publisher(PointCloud2, SPHEREFORMER_LEFT_BOUNDARY, 5)
        self.right_boundary_pub = self.create_publisher(PointCloud2, SPHEREFORMER_RIGHT_BOUNDARY, 5)

        self.create_subscription(
            PointCloud2,
            RING_FILTERED_POINTS_TOPIC,
            self.ros_callback,
            1,
        )
        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        self.cluster_pointcloud = False

    @timer
    def _load_model(self):
        """Load and initialize the model."""

        self.get_logger().info("Loading and initializing the model...")

        # Model configuration
        self.cfg.patch_size = np.array(
            [self.cfg.voxel_size[i] * self.cfg.patch_size for i in range(3)]
        ).astype(np.float32)
        window_size = self.cfg.patch_size * self.cfg.window_size
        window_size_sphere = np.array(self.cfg.window_size_sphere)

        model = Model(
            input_c=self.cfg.input_c,
            m=self.cfg.m,
            classes=self.cfg.classes,
            block_reps=self.cfg.block_reps,
            block_residual=self.cfg.block_residual,
            layers=self.cfg.layers,
            window_size=window_size,
            window_size_sphere=window_size_sphere,
            quant_size=window_size / self.cfg.quant_size_scale,
            quant_size_sphere=window_size_sphere / self.cfg.quant_size_scale,
            rel_query=self.cfg.rel_query,
            rel_key=self.cfg.rel_key,
            rel_value=self.cfg.rel_value,
            drop_path_rate=self.cfg.drop_path_rate,
            window_size_scale=self.cfg.window_size_scale,
            grad_checkpoint_layers=self.cfg.grad_checkpoint_layers,
            sphere_layers=self.cfg.sphere_layers,
            a=self.cfg.a,
        )

        # Load checkpoint
        self.get_logger().info("Loading model weights from checkpoint...")
        checkpoint = torch.load(self.checkpoint_path)
        state_dict = {
            k.replace("module.", ""): v for k, v in checkpoint["state_dict"].items()
        }
        model.load_state_dict(state_dict, strict=False)
        model = model.to(device)
        if device.type == "cuda":
            model = model.half()  # enable mixed precision
        model.eval()
        return model

    @timer
    def ros_callback(self, msg):
        """ROS callback to process incoming PointCloud2 messages."""
        self.get_logger().info("Received a message, starting inference...")
        with torch.no_grad(), torch.cuda.amp.autocast():
            seg_points, output_labels = self.inference_from_ros_message(msg, self.model)
        self.get_logger().info("Inference complete. Processing results...")

        # Check if seg_points is structured and has the required dtype fields
        if seg_points is None or seg_points.dtype.names is None:
            self.get_logger().error(
                "Invalid data structure in seg_points. Ensure the point cloud data is structured correctly."
            )
            return

        # Ensure seg_points is on CPU and converted to NumPy
        seg_points = (
            seg_points.cpu().numpy()
            if hasattr(seg_points, "cpu")
            else np.array(seg_points)
        )
        output_labels = (
            output_labels.cpu().numpy()
            if hasattr(output_labels, "cpu")
            else np.array(output_labels)
        )

        # Get points where label is 8
        mask = output_labels == 8
        road_points = seg_points[mask]
        road_points = road_points.view(np.float32).reshape(-1, 4)

        if self.cluster_pointcloud:
            if road_points.size > 0:
                clustering = DBSCAN(eps=0.5, min_samples=10).fit(road_points[:, :3])
                labels = clustering.labels_
                road_points = road_points[labels != -1]  # Remove outliers

        left_points = []
        right_points = []
        if road_points.size > 0:
            min_x = np.min(road_points[:, 0])
            max_x = np.max(road_points[:, 0])
            bins = np.linspace(min_x, max_x, num=22)

            widths = []
            for i in range(len(bins) - 1):
                bin_mask = (road_points[:, 0] >= bins[i]) & (
                    road_points[:, 0] < bins[i + 1]
                )
                bin_points = road_points[bin_mask]
                if bin_points.size == 0:
                    continue

                leftmost_idx = np.argmax(bin_points[:, 1])  # Index of max y
                rightmost_idx = np.argmin(bin_points[:, 1])  # Index of min y

                left_points.append(bin_points[leftmost_idx])
                right_points.append(bin_points[rightmost_idx])

                width = bin_points[rightmost_idx, 1] - bin_points[leftmost_idx, 1]
                widths.append(width)

        left_points, right_points = np.array(left_points), np.array(right_points)
        self.create_cloud(road_points, self.pub, msg)

        if left_points.size > 0 and right_points.size > 0:
            self.create_cloud(left_points, self.left_boundary_pub, msg)

        if right_points.size > 0 and left_points.size > 0:
            self.create_cloud(right_points, self.right_boundary_pub, msg)

        self.get_logger().info("Publishing the processed point cloud and bounding boxes.")

    def create_cloud(self, points: np.ndarray, pub, ref_msg: PointCloud2):
        header = ref_msg.header
        data = [tuple(p) for p in np.asarray(points, dtype=np.float32)]
        pc_msg = pc2.create_cloud(header, self.fields, data)
        pub.publish(pc_msg)

    @timer
    def inference_from_ros_message(self, ros_msg, model):
        pc = ros_numpy.numpify(ros_msg)
        pcd = np.zeros((pc.shape[0], 4))
        pcd[:, 0] = pc["x"]
        pcd[:, 1] = pc["y"]
        pcd[:, 2] = pc["z"]
        np_points = np.asarray(pcd)
        np_points = crop_pointcloud(np_points, lim_x, lim_y, lim_z)
        p_points = np_points[:, 0:3]
        intensities = np_points[:, 3]
        binary_points, binary_labels = self.convert_to_binary_format(
            p_points, intensities
        )
        processed_data = self.semkitti_dataset.process_live_data(
            binary_points, binary_labels
        )
        batch_data = [processed_data]
        (coord, xyz, feat, target, offset, inds_reverse) = self.collation_fn_voxelmean(
            batch_data
        )
        inds_reverse = inds_reverse.to("cuda:0", non_blocking=True)
        offset_ = offset.clone()
        offset_[1:] = offset_[1:] - offset_[:-1]
        batch = torch.cat(
            [torch.tensor([ii] * o) for ii, o in enumerate(offset_)], 0
        ).long()
        coord = torch.cat([batch.unsqueeze(-1), coord], -1)
        spatial_shape = np.clip((coord.max(0)[0][1:] + 1).numpy(), 128, None)
        coord, xyz, feat, target, offset = (
            coord.cuda(non_blocking=True),
            xyz.cuda(non_blocking=True),
            feat.cuda(non_blocking=True),
            target.cuda(non_blocking=True),
            offset.cuda(non_blocking=True),
        )
        batch = batch.cuda(non_blocking=True)
        sinput = spconv.SparseConvTensor(feat, coord.int(), spatial_shape, 1)
        assert batch.shape[0] == feat.shape[0]
        with torch.no_grad():
            output = model(sinput, xyz, batch)
            output = output[inds_reverse, :]
            output = output.max(1)[1]
        points = np.zeros(
            np_points.shape[0],
            dtype=[
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("intensity", np.float32),
            ],
        )
        points["x"] = np_points[:, 0]
        points["y"] = np_points[:, 1]
        points["z"] = np_points[:, 2]
        return points, output

    def collation_fn_voxelmean(self, batch):
        coords, xyz, feats, labels, inds_recons = list(zip(*batch))
        inds_recons = list(inds_recons)
        accmulate_points_num = 0
        offset = []
        for i in range(len(coords)):
            inds_recons[i] = accmulate_points_num + inds_recons[i]
            accmulate_points_num += coords[i].shape[0]
            offset.append(accmulate_points_num)
        coords = torch.cat(coords)
        xyz = torch.cat(xyz)
        feats = torch.cat(feats)
        labels = torch.cat(labels)
        offset = torch.IntTensor(offset)
        inds_recons = torch.cat(inds_recons)
        return coords, xyz, feats, labels, offset, inds_recons

    def convert_to_binary_format(self, np_points, intensities):
        """Convert numpy points to binary format for inference."""
        np_points_with_intensity = np.hstack((np_points, intensities.reshape(-1, 1)))
        binary_points = np_points_with_intensity.astype(np.float32).tobytes()
        labels = np.zeros(np_points_with_intensity.shape[0], dtype=np.uint32)
        return np.asarray(binary_points), labels.tobytes()


def main(args=None):
    rclpy.init(args=args)
    node = SphereformerLidarSegmentation(CONFIG_PATH, CHECKPOINT_PATH)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
