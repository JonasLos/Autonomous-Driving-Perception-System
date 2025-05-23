#!/usr/bin/env python3
# type: ignore

import os
from functools import wraps

import numpy as np
import ros_numpy
import rospy
import sensor_msgs.point_cloud2 as pc2
import spconv.pytorch as spconv
import torch
import yaml
from semantic_kitti_ros import SemanticKITTI
from sensor_msgs.msg import PointCloud2, PointField
from sklearn.cluster import DBSCAN
from SphereFormer.util import config
from SphereFormer_changes.unet_spherical_transformer import Semantic as Model
from visualization_msgs.msg import MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(
    SCRIPT_DIR,
    "SphereFormer_changes/semantic_kitti_unet32_spherical_transformer.yaml",
)
CHECKPOINT_PATH = os.path.join(SCRIPT_DIR, "SphereFormer/model_semantic_kitti.pth")

# Path to the YAML file
TOPICS_PATH = "/home/dev/Documents/Autonomous-Driving-Perception-System/src/topics.yaml"

# Load YAML config
with open(TOPICS_PATH, "r") as f:
    topic_config = yaml.safe_load(f)

# === TOPICS ===
LIDAR_TOPIC = topic_config["topics"]["raw"]["lidar"]
LIDAR_BBOX_TOPIC = topic_config["topics"]["sphereformer"]["lidar_bbox"]
SPHEREFORMER_SEGMENTATION_TOPIC = topic_config["topics"]["sphereformer"]["segmentation"]
SPHEREFORMER_LEFT_BOUNDARY = topic_config["topics"]["sphereformer"]["left_boundary"]
SPHEREFORMER_RIGHT_BOUNDARY = topic_config["topics"]["sphereformer"]["right_boundary"]
SPHEREFORMER_CENTER_LINE_POINTS = topic_config["topics"]["sphereformer"][
    "centerline_points"
]

lim_x, lim_y, lim_z = [-50, 100], [-20, 20], [-5, 10]

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
torch.cuda.set_per_process_memory_fraction(0.3, device=torch.device("cuda:0"))


def timer(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = rospy.Time.now().to_sec()
        result = func(*args, **kwargs)
        end_time = rospy.Time.now().to_sec()
        print(f"{func.__name__} executed in {end_time - start_time:.4f} seconds")
        return result

    return wrapper


class PointCloudInference:
    def __init__(
        self,
        config_path,
        checkpoint_path,
    ):
        # Configuration and model initialization
        self.config_path = config_path
        self.checkpoint_path = checkpoint_path
        self.cfg = config.load_cfg_from_cfg_file(self.config_path)
        self.model = self._load_model()
        self.semkitti_dataset = SemanticKITTI(split="val")
        self.current_marker_ids = set()
        self.previous_centerline = None  # Store previous centerline points

        # ROS publishers and subscribers
        self.pub = rospy.Publisher(
            SPHEREFORMER_SEGMENTATION_TOPIC, PointCloud2, queue_size=1
        )
        # Publishers
        self.left_boundary_pub = rospy.Publisher(
            SPHEREFORMER_LEFT_BOUNDARY, PointCloud2, queue_size=1
        )
        self.right_boundary_pub = rospy.Publisher(
            SPHEREFORMER_RIGHT_BOUNDARY, PointCloud2, queue_size=1
        )
        self.bounding_box_pub = rospy.Publisher(
            LIDAR_BBOX_TOPIC, MarkerArray, queue_size=1
        )
        self.centerline_pub = rospy.Publisher(
            SPHEREFORMER_CENTER_LINE_POINTS, PointCloud2, queue_size=10
        )

        rospy.Subscriber(
            LIDAR_TOPIC,
            PointCloud2,
            self.ros_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]

    @timer
    def _load_model(self):
        """Load and initialize the model."""

        rospy.loginfo("Loading and initializing the model...")

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
        rospy.loginfo("Loading model weights from checkpoint...")
        checkpoint = torch.load(self.checkpoint_path)
        state_dict = {
            k.replace("module.", ""): v for k, v in checkpoint["state_dict"].items()
        }
        model.load_state_dict(state_dict, strict=False)
        model = model.cuda()
        model.eval()
        return model

    @timer
    def ros_callback(self, msg):
        """ROS callback to process incoming PointCloud2 messages."""
        rospy.loginfo("Received a message, starting inference...")
        seg_points, output_labels = self.inference_from_ros_message(msg, self.model)
        rospy.loginfo("Inference complete. Processing results...")

        # Check if seg_points is structured and has the required dtype fields
        if seg_points is None or seg_points.dtype.names is None:
            rospy.logerr(
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

        # if road_points.size > 0:
        #     clustering = DBSCAN(eps=0.5, min_samples=10).fit(road_points[:, :3])
        #     labels = clustering.labels_
        #     road_points = road_points[labels != -1]  # Remove outliers

        left_points = []
        right_points = []
        print(road_points.size)
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

                left_points.append(bin_points[leftmost_idx])  # Full row
                right_points.append(bin_points[rightmost_idx])  # Full row

                width = bin_points[rightmost_idx, 1] - bin_points[leftmost_idx, 1]
                widths.append(width)

        left_points, right_points = np.array(left_points), np.array(right_points)

        # Compute evenly spaced centerline points
        if left_points.shape[0] > 0 and right_points.shape[0] > 0:
            left_points_front = left_points[left_points[:, 0] > 0]
            right_points_front = right_points[right_points[:, 0] > 0]
            num_points = min(len(left_points_front), len(right_points_front))
            centerline_points = (
                left_points_front[: num_points - 1, :]
                + right_points_front[: num_points - 1, :]
            ) / 2  # Compute center points

            # Apply smoothing
            centerline_points = self.smooth_centerline(centerline_points)

            # Select three evenly spaced points
            num_points = centerline_points.shape[0]
            if num_points >= 3:
                indices = np.linspace(0, num_points - 1, num=3, dtype=int)
                selected_center_points = centerline_points[indices, :3]  # Only X, Y, Z
            else:
                selected_center_points = centerline_points[
                    :, :3
                ]  # Use all if less than 3

            print("Selected Center Points:", selected_center_points)

            # Publish centerline points as PointCloud2
            self.publish_centerline_points(msg, selected_center_points)

        self.create_cloud(road_points, self.pub, msg)
        if left_points.size > 0:
            self.create_cloud(left_points, self.left_boundary_pub, msg)
        if right_points.size > 0:
            self.create_cloud(right_points, self.right_boundary_pub, msg)
        rospy.loginfo("Publishing the processed point cloud and bounding boxes.")

    def smooth_centerline(self, new_centerline, alpha=0.3, num_fixed_points=10):
        """
        Apply exponential smoothing to stabilize centerline points.
        - alpha: Smoothing factor (0.0 - no update, 1.0 - instant update)
        - num_fixed_points: Ensure a fixed number of centerline points
        """
        if new_centerline.shape[0] == 0:
            return (
                self.previous_centerline
                if self.previous_centerline is not None
                else new_centerline
            )

        # Interpolate to ensure a fixed number of points
        num_points = new_centerline.shape[0]
        if num_points > 1:
            interp_indices = np.linspace(
                0, num_points - 1, num=num_fixed_points, dtype=int
            )
            new_centerline = new_centerline[interp_indices]
        elif num_points == 1:
            new_centerline = np.tile(
                new_centerline, (num_fixed_points, 1)
            )  # Duplicate same point

        # Initialize previous centerline if None
        if (
            self.previous_centerline is None
            or self.previous_centerline.shape != new_centerline.shape
        ):
            self.previous_centerline = new_centerline

        # Apply exponential smoothing
        self.previous_centerline = (
            alpha * new_centerline + (1 - alpha) * self.previous_centerline
        )
        return self.previous_centerline

    def publish_centerline_points(self, msg, selected_center_points):
        """
        Publish centerline points as a PointCloud2 message.
        Keeps the header timestamp and allows multiple points to be stored.
        """
        header = msg.header  # Preserve timestamp and frame

        # Define the fields: x, y, z as FLOAT32
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Convert points to a list of tuples
        point_cloud_data = [tuple(point) for point in selected_center_points]

        # Create and publish PointCloud2 message
        centerline_msg = pc2.create_cloud(header, fields, point_cloud_data)
        self.centerline_pub.publish(centerline_msg)
        rospy.loginfo("Published centerline points as PointCloud2")

    def create_cloud(self, points_3d, publisher, msg):
        header = msg.header
        pointcloud = pc2.create_cloud(header, self.fields, points_3d)
        publisher.publish(pointcloud)
        rospy.loginfo("Published point cloud with %d points.", len(points_3d))

    @timer
    def inference_from_ros_message(self, ros_msg, model):
        pc = ros_numpy.numpify(ros_msg)
        pcd = np.zeros((pc.shape[0], 4))
        pcd[:, 0] = pc["x"]
        pcd[:, 1] = pc["y"]
        pcd[:, 2] = pc["z"]
        np_points = np.asarray(pcd)
        np_points = self.crop_pointcloud(np_points)
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

    def crop_pointcloud(self, pointcloud):
        """Crop point cloud within the specified limits."""
        mask = np.where(
            (pointcloud[:, 0] >= lim_x[0])
            & (pointcloud[:, 0] <= lim_x[1])
            & (pointcloud[:, 1] >= lim_y[0])
            & (pointcloud[:, 1] <= lim_y[1])
            & (pointcloud[:, 2] >= lim_z[0])
            & (pointcloud[:, 2] <= lim_z[1])
        )
        return pointcloud[mask]

    def convert_to_binary_format(self, np_points, intensities):
        """Convert numpy points to binary format for inference."""
        np_points_with_intensity = np.hstack((np_points, intensities.reshape(-1, 1)))
        binary_points = np_points_with_intensity.astype(np.float32).tobytes()
        labels = np.zeros(np_points_with_intensity.shape[0], dtype=np.uint32)
        return np.asarray(binary_points), labels.tobytes()


if __name__ == "__main__":
    rospy.init_node("pointcloud_inference", anonymous=True)
    inference_node = PointCloudInference(CONFIG_PATH, CHECKPOINT_PATH)
    rospy.spin()
