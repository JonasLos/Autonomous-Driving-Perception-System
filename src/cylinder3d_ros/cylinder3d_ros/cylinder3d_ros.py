#!/usr/bin/env python3
# type: ignore

import os
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs_py import point_cloud2 as pc2
import torch
import yaml
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import PointCloud2, PointField
from perception_common.utils import crop_pointcloud, timer

# Add Cylinder3D path
PKG_SHARE = get_package_share_directory("cylinder3d_ros")
CYLINDER3D_PATH = os.path.join(PKG_SHARE, "Cylinder3D")
if CYLINDER3D_PATH not in sys.path:
    sys.path.insert(0, CYLINDER3D_PATH)

from builder import data_builder, model_builder
from config.config import load_config_data
from dataloader.dataset_semantickitti import collate_fn_BEV
from utils.load_save_util import load_checkpoint
from dataloader.dataset_semantickitti import collate_fn_BEV, cart2polar


# Path to the YAML file
TOPICS_PATH = os.path.join(get_package_share_directory("perception_common"), "topics.yaml")

# Load YAML config
with open(TOPICS_PATH, "r") as f:
    topic_config = yaml.safe_load(f)

# === TOPICS ===
CYLINDER3D_SEGMENTATION_TOPIC = topic_config["topics"]["cylinder3d"]["segmentation_mask"]
RING_FILTERED_POINTS_TOPIC = topic_config["topics"]["cylinder3d"]["ring_filtered"]

CONFIG_PATH = os.path.join(CYLINDER3D_PATH, "config/semantickitti.yaml")
CHECKPOINT_PATH = os.path.join(CYLINDER3D_PATH, "model_semantic_kitti.pt") # TODO: download this

lim_x, lim_y, lim_z = [-30, 100], [-10, 10], [-5, 0]

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

class Cylinder3DLidarSegmentation(Node):
    def __init__(self, config_path, checkpoint_path):
        super().__init__("cylinder3d_lidar_segmentation")
        # Configuration and model initialization
        self.configs = load_config_data(config_path)
        self.model = self._load_model(checkpoint_path)
        
        # ROS2 publishers and subscribers
        self.pub = self.create_publisher(PointCloud2, CYLINDER3D_SEGMENTATION_TOPIC, 1)
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
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

    def _load_model(self, model_load_path):
        """Load and initialize the model."""
        self.get_logger().info("Loading and initializing the model...")
        model_config = self.configs['model_params']
        my_model = model_builder.build(model_config)
        if os.path.exists(model_load_path):
            my_model = load_checkpoint(model_load_path, my_model)
        my_model.to(device)
        my_model.eval()
        return my_model

    @timer
    def ros_callback(self, msg):
        """ROS callback to process incoming PointCloud2 messages."""
        self.get_logger().info("Received a message, starting inference...")
        with torch.no_grad():
            seg_points, output_labels = self.inference_from_ros_message(msg, self.model)
        self.get_logger().info("Inference complete. Publishing results...")
        self.create_cloud(seg_points, self.pub, msg)

    def create_cloud(self, points: np.ndarray, pub, ref_msg: PointCloud2):
        header = ref_msg.header
        # Add color based on labels
        # TODO: Implement color mapping based on labels
        pc_msg = pc2.create_cloud(header, self.fields, points)
        pub.publish(pc_msg)

    @timer
    def inference_from_ros_message(self, ros_msg, model):
        pc_generator = pc2.read_points(ros_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        pcd = np.array(list(pc_generator))
        
        # Replicate the data processing from Cylinder3D's dataloader
        dataset_config = self.configs['dataset_params']
        grid_size = np.asarray(self.configs['model_params']['output_shape'])
        
        # 1. Get points and features
        xyz = pcd[:, :3]
        sig = pcd[:, 3] # intensity

        # 2. Convert to polar coordinates
        xyz_pol = cart2polar(xyz)

        # 3. Voxelization
        max_bound = np.asarray(dataset_config['max_volume_space'])
        min_bound = np.asarray(dataset_config['min_volume_space'])
        crop_range = max_bound - min_bound
        intervals = crop_range / (grid_size - 1)
        
        if (intervals == 0).any():
            self.get_logger().error("Zero interval detected in voxelization!")
            return pcd, np.zeros(pcd.shape[0])
            
        grid_ind = (np.floor((np.clip(xyz_pol, min_bound, max_bound) - min_bound) / intervals)).astype(np.int32)

        # 4. Center data on each voxel
        voxel_centers = (grid_ind.astype(np.float32) + 0.5) * intervals + min_bound
        return_xyz = xyz_pol - voxel_centers
        return_xyz = np.concatenate((return_xyz, xyz_pol, xyz[:, :2]), axis=1)
        
        # 5. Create feature tensor
        return_fea = np.concatenate((return_xyz, sig[..., np.newaxis]), axis=1).astype(np.float32)

        # 6. Assemble data for collate_fn_BEV
        # We are processing a single scan, so we create a batch of 1
        # The original collate function expects a list of data tuples
        # We create the data tuple that __getitem__ would produce
        
        # `voxel_position` and `processed_label` are not directly used for inference in the demo
        # but are required by the collate function. We can create dummy versions.
        voxel_position = np.zeros(grid_size, dtype=np.float32)
        processed_label = np.ones(grid_size, dtype=np.uint8) * self.configs['dataset_params']['ignore_label']
        
        # `labels` are also not used for inference, create dummy labels
        labels = np.zeros(pcd.shape[0], dtype=np.uint8)

        data_tuple = (voxel_position, processed_label, grid_ind, labels, return_fea)
        
        # Now, apply the collate function
        data_collated = collate_fn_BEV([data_tuple])
        
        # 7. Unpack collated data and run inference
        (voxel_position_t, processed_label_t, grid_ind_t, point_label_t, xyz_t) = data_collated
        
        predict_labels = model(xyz_t, grid_ind_t, device)
        predict_labels = torch.argmax(predict_labels, dim=1)
        predict_labels = predict_labels.cpu().detach().numpy()

        # 8. Map predictions back to points
        # The model predicts one label per voxel. We need to map this back to the original points.
        # The simplest way is to assign the voxel's label to all points within that voxel.
        
        # Create a colored point cloud for visualization
        output_points = np.zeros(pcd.shape[0], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4'), ('label', 'u1')])
        output_points['x'] = pcd[:, 0]
        output_points['y'] = pcd[:, 1]
        output_points['z'] = pcd[:, 2]
        output_points['intensity'] = pcd[:, 3]
        
        # This mapping is not perfect but is a good starting point.
        # The output of the model is on the voxel grid, not the original points.
        # A more accurate mapping would require using the `inds_recons` from the original dataset,
        # which we don't have in this live pipeline.
        
        # For now, we will return the labels per voxel, and handle the point mapping later.
        # The output of the model is what we need.
        
        # For now, just return the input points and the predicted labels per point (approximated)
        point_labels = predict_labels[grid_ind[:, 0], grid_ind[:, 1], grid_ind[:, 2]]
        
        return pcd, point_labels

# Add the cart2polar function to the class or file
def cart2polar(input_xyz):
    rho = np.sqrt(input_xyz[:, 0] ** 2 + input_xyz[:, 1] ** 2)
    phi = np.arctan2(input_xyz[:, 1], input_xyz[:, 0])
    return np.stack((rho, phi, input_xyz[:, 2]), axis=1)



def main(args=None):
    rclpy.init(args=args)
    node = Cylinder3DLidarSegmentation(CONFIG_PATH, CHECKPOINT_PATH)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
