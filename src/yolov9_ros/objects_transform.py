#!/usr/bin/env python3
# type: ignore

import os
from collections import defaultdict

from ament_index_python.packages import get_package_share_directory
import message_filters
import rclpy
from rclpy.node import Node
import numpy as np
import ros2_numpy as ros_numpy
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import torch
import yaml
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from radar_msgs.msg import RadarTrackArray
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import DBSCAN

from perception_common.configs import PROJ, T1
from perception_common.utils import crop_pointcloud, inverse_rigid_transform
from yolov9_msgs.msg import BboxList

# Point cloud limits
lim_x, lim_y, lim_z = [2.5, 100], [-10, 10], [-3.5, 1.5]
pixel_lim = 20

# Radar Limit Cutoff
RADAR_LIMIT = 20  # meters
CLOSE_DISTANCE_THRESHOLD = 7  # meters

# Average Class Dimensions
BASE_PATH = os.path.dirname(os.path.abspath(__file__))
PKG_SHARE = get_package_share_directory("yolov9_ros")
CLASS_AVERAGE_PATH = os.path.join(PKG_SHARE, "class_averages.yaml")
with open(CLASS_AVERAGE_PATH, "r", encoding="utf-8") as file:
    average_dimensions = yaml.safe_load(file)

# === Load YAML configs ===
TOPICS_PATH = os.path.join(
    get_package_share_directory("perception_common"), "topics.yaml"
)
with open(TOPICS_PATH, "r") as f:
    topic_config = yaml.safe_load(f)

# === Assign topic variables ===
LIDAR_TOPIC = topic_config["topics"]["raw"]["lidar_tc"]
RADAR_TRACKS_TOPIC = topic_config["topics"]["raw"]["radar"]
YOLO_BBOX_TOPIC = topic_config["topics"]["yolo"]["bbox"]
FUSED_BBOX_TOPIC = topic_config["topics"]["yolo"]["fused_bbox"]


class ObjectsTransform(Node):
    """
    Class to handle the transformation of detected bounding box coordinates
    from 2D image space to 3D lidar space.
    """

    def __init__(self):
        super().__init__("objects_transform")
        self.get_logger().info("Initializing TransformFuse node...")
        # Initialize ROS2 publishers
        self.bbox_publish = self.create_publisher(BoundingBoxArray, FUSED_BBOX_TOPIC, 1)

        # Point field configuration for PointCloud2
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

        # Initialize ROS2 message_filters subscribers
        self.sub_lidar = message_filters.Subscriber(self, PointCloud2, LIDAR_TOPIC)
        self.sub_image = message_filters.Subscriber(self, BboxList, YOLO_BBOX_TOPIC)
        self.sub_radar = message_filters.Subscriber(self, RadarTrackArray, RADAR_TRACKS_TOPIC)

        # Initialize header
        self.header = std_msgs.msg.Header()
        self.header.frame_id = "lidar_tc"

        # Time synchronizer for lidar, image, and radar data
        ts = message_filters.ApproximateTimeSynchronizer([self.sub_lidar, self.sub_image, self.sub_radar], 15, 0.4)
        ts.registerCallback(self.callback)

    def callback(self, msgLidar, msgPoint, msgRadar):
        """
        Callback function for synchronized lidar, image, and radar data.

        Args:
            msgLidar (sensor_msgs.msg.PointCloud2): Lidar point cloud message.
            msgPoint (yolov9ros.msg.BboxCentersClass): Bounding box centers from image detection.
            msgRadar (derived_object_msgs.msg.ObjectWithCovarianceArray): Radar objects message.
        """
        self.get_logger().info("Received synchronized messages.")

        # Convert lidar data to numpy array
        pc = ros_numpy.numpify(msgLidar)
        points = np.vstack((pc["x"], pc["y"], pc["z"], np.ones(pc["x"].shape[0]))).T

        # Crop point cloud and transform to camera frame
        pc_arr = crop_pointcloud(points, lim_x, lim_y, lim_z)
        pc_arr_pick = np.transpose(pc_arr)
        m1 = torch.matmul(
            torch.tensor(inverse_rigid_transform(T1)), torch.tensor(pc_arr_pick)
        )
        uv1 = torch.matmul(torch.tensor(PROJ), m1)
        uv1[:2, :] /= uv1[2, :]

        center_3d = []
        label = []
        u, v = uv1[0, :].numpy(), uv1[1, :].numpy()

        # Match bounding box centers with point cloud data
        for bbox_info in msgPoint.Bboxes:
            x_min, y_min = bbox_info.x_min, bbox_info.y_min
            x_max, y_max = bbox_info.x_max, bbox_info.y_max
            class_id, _ = bbox_info.class_id, bbox_info.confidence

            # Calculate center of the bounding box
            center_x = (x_min + x_max) / 2
            center_y = (y_min + y_max) / 2

            # Find corresponding points in the point cloud
            idx = np.where(
                (u + pixel_lim >= center_x)
                & (u - pixel_lim <= center_x)
                & (v + pixel_lim >= center_y)
                & (v - pixel_lim <= center_y)
            )[0]

            if idx.size > 0:
                for i in [idx[0]]:  # Only publish one object for each detection
                    center_3d.append(
                        [pc_arr_pick[0][i], pc_arr_pick[1][i], pc_arr_pick[2][i], 1]
                    )
                    label.append([class_id])
        self.get_logger().info(f"Found {len(center_3d)} camera detections.")

        # Publishing the bounding boxes
        bbox_array = BoundingBoxArray()
        center_3d = np.array(center_3d)

        # Filtering out duplicate camera detections
        unique_camera_indices = []
        if center_3d.size > 0:  # Check if center_3d is empty
            close_distance_threshold_camera = 1
            db = DBSCAN(eps=close_distance_threshold_camera, min_samples=1).fit(
                center_3d[:, :3]
            )

            for lab in set(db.labels_):
                indices = np.where(db.labels_ == lab)[0]
                unique_camera_indices.append(
                    indices[0]
                )  # Taking the first point in the cluster

        self.get_logger().info(f"Found {len(unique_camera_indices)} unique camera detections.")

        # Finding matching detections b/w camera and radar
        camera_detections = unique_camera_indices
        radar_detections = msgRadar.tracks
        matched_pairs = []
        for i in camera_detections:
            for j, rad_det in enumerate(radar_detections):
                cam_position = center_3d[i][:3]
                # Calculate the center of the radar track shape by averaging the points
                track_shape = rad_det.track_shape.points
                rad_position = np.array(
                    [
                        np.mean(
                            [point.x for point in track_shape]
                        ),  # Averaging x-coordinates
                        np.mean(
                            [point.y for point in track_shape]
                        ),  # Averaging y-coordinates
                        np.mean(
                            [point.z for point in track_shape]
                        ),  # Averaging z-coordinates
                    ]
                )
                distance = np.linalg.norm(cam_position - rad_position)
                if rad_position[0] > 75 and distance < CLOSE_DISTANCE_THRESHOLD:
                    matched_pairs.append((i, j))

        self.get_logger().info(f"Matched pairs: {matched_pairs}")
        matched_dict = defaultdict(list)
        for i, j in matched_pairs:
            matched_dict[i].append(j)

        # Add 3D centers to bounding box array
        for i, box in enumerate(center_3d):  # camera detections
            if i in unique_camera_indices:
                class_ = int(label[i][0])
                bbox = BoundingBox()
                bbox.header.stamp = msgLidar.header.stamp
                bbox.header.frame_id = msgLidar.header.frame_id
                bbox.pose.position.x, bbox.pose.position.y, bbox.pose.position.z = box[
                    :3
                ]
                # bbox dimensions
                bbox.dimensions.x, bbox.dimensions.y, bbox.dimensions.z = (
                    average_dimensions[class_]["dimensions"][2],
                    average_dimensions[class_]["dimensions"][1],
                    average_dimensions[class_]["dimensions"][0],
                )
                bbox.pose.orientation.w = 1
                bbox.value = 1
                bbox.label = int(label[i][0])  # class number
                bbox_array.boxes.append(bbox)

        # Add radar objects to bounding box array
        for i, obj in enumerate(msgRadar.tracks):  # radar detections
            if i not in [g for f, g in matched_pairs]:
                # Calculate the center of the radar track shape by averaging the points
                track_shape = (
                    obj.track_shape.points
                )  # Assuming 'track_shape' is part of 'obj'
                rad_position = np.array(
                    [
                        np.mean(
                            [point.x for point in track_shape]
                        ),  # Averaging x-coordinates
                        np.mean(
                            [point.y for point in track_shape]
                        ),  # Averaging y-coordinates
                        np.mean(
                            [point.z for point in track_shape]
                        ),  # Averaging z-coordinates
                    ]
                )

                # Apply radar limit check based on x-coordinate of the radar position
                if rad_position[0] > RADAR_LIMIT:
                    bbox = BoundingBox()
                    bbox.header = msgRadar.header
                    bbox.pose.position.x = rad_position[
                        0
                    ]  # Using the computed radar position
                    bbox.pose.position.y = rad_position[1]
                    bbox.pose.position.z = rad_position[2]
                    bbox.dimensions.x = 1.5
                    bbox.dimensions.y = 1.5
                    bbox.dimensions.z = 1.5
                    bbox.label = 9999  # Label for radar detection
                    bbox_array.boxes.append(bbox)

        bbox_array.header.frame_id = msgLidar.header.frame_id
        self.bbox_publish.publish(bbox_array)
        self.get_logger().info(f"Published {len(bbox_array.boxes)} fused bounding boxes.\n")

    def compute_distance_matrix(self, camera_detections, radar_detections):
        """
        Calculate the distance matrix between camera and radar detections.

        Args:
            camera_detections (np.ndarray): Camera detection coordinates.
            radar_detections (list): List of radar detections.

        Returns:
            np.ndarray: Distance matrix.
        """
        num_camera = camera_detections.shape[0]
        num_radar = len(radar_detections)

        distance_matrix = np.zeros((num_camera, num_radar))
        for i in range(num_camera):
            for j in range(num_radar):
                distance_matrix[i, j] = np.linalg.norm(
                    camera_detections[i][0] - radar_detections[j].pose.pose.position.x
                )
        return distance_matrix


def main(args=None):
    rclpy.init(args=args)
    node = ObjectsTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
