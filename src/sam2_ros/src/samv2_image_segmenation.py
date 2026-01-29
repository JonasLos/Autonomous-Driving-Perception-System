#!/usr/bin/env python3
# type: ignore

import os

import cv2
import numpy as np
import ros2_numpy as ros_numpy
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import torch
import yaml
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
from sensor_msgs.msg import Image

from sam2_ros.msg import DetectedRoadArea
from src.configs import T1
from src.utils import inverse_rigid_transform, timer

TOPICS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "topics.yaml"
)

# Load YAML config
with open(TOPICS_PATH, "r") as f:
    config = yaml.safe_load(f)

# === Initialize topics ===
CAMERA_TOPIC = config["topics"]["raw"]["camera"]
YOLO_BBOX_TOPIC = config["topics"]["yolo"]["bbox"]
SAM_SEGMENTATION_MASK_TOPIC = config["topics"]["sam"]["segmentation_mask"]
SAM_LEFT_CONTOUR_TOPIC = config["topics"]["sam"]["left_contour"]
SAM_RIGHT_CONTOUR_TOPIC = config["topics"]["sam"]["right_contour"]

# Set device to GPU if available, otherwise use CPU
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
torch.cuda.set_per_process_memory_fraction(0.3, device=device)
print(f"Using device: {device}")

# Load SAM2 model
base_path = os.path.dirname(os.path.abspath(__file__))
checkpoints_dir = os.path.join(base_path, "segment-anything-2", "checkpoints")
sam2_checkpoint = os.path.join(checkpoints_dir, "sam2_hiera_base_plus.pt")
model_cfg = "sam2_hiera_b+.yaml"
sam2_model = build_sam2(model_cfg, sam2_checkpoint, device=device)
predictor = SAM2ImagePredictor(sam2_model)

# === Resize Configuration ===
RESIZED_WIDTH = 400
RESIZED_HEIGHT = 400
ORIGINAL_WIDTH = 2048.0  # Original image width
ORIGINAL_HEIGHT = 1544.0  # Original image height

# Define points for initial segmentation prompt
point_coords = np.array(
    [
        [
            400 * RESIZED_WIDTH / ORIGINAL_WIDTH * 2,
            700 * RESIZED_HEIGHT / ORIGINAL_HEIGHT * 2,
        ],
        [
            550 * RESIZED_WIDTH / ORIGINAL_WIDTH * 2,
            700 * RESIZED_HEIGHT / ORIGINAL_HEIGHT * 2,
        ],
        [
            650 * RESIZED_WIDTH / ORIGINAL_WIDTH * 2,
            700 * RESIZED_HEIGHT / ORIGINAL_HEIGHT * 2,
        ],
    ]
)
input_labels = [1, 1, 1]
MIN_CONTOUR_AREA = 1000.0

T_vel_cam = inverse_rigid_transform(T1)


@timer
def process_image(image, detected_objects, publish_image=False):
    image = cv2.resize(image, (RESIZED_WIDTH, RESIZED_HEIGHT))
    center_x = int(RESIZED_WIDTH * 0.75)

    with torch.inference_mode(), torch.cuda.amp.autocast():
        predictor.set_image(image)
        masks, _, _ = predictor.predict(
            point_coords=point_coords, point_labels=input_labels, multimask_output=False
        )

    road_mask = (masks[0] > 0).astype(np.uint8)
    # Create a unified mask for road and bounding boxes
    unified_mask = road_mask.copy()

    # Iterate over detected objects to adjust the mask
    for x_min, y_min, x_max, y_max in detected_objects:
        x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)

        # Create a bounding box mask
        bbox_mask = np.zeros_like(road_mask, dtype=np.uint8)
        cv2.rectangle(bbox_mask, (x_min, y_min), (x_max, y_max), 255, -1)

        # Ensure bounding boxes do not override road regions
        unified_mask = cv2.bitwise_and(unified_mask, cv2.bitwise_not(bbox_mask))

    # Fill gaps in the road mask
    kernel = np.ones((6, 6), np.uint8)
    road_mask_cleaned = cv2.morphologyEx(unified_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        road_mask_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )

    if not contours:
        print("[WARN] No contours found.")
        return None, None, None

    road_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(road_contour) < MIN_CONTOUR_AREA:
        print("[WARN] The contour area is below the threshold.")
        return None, None, None

    # Filter road contour points
    MIN_Y_COORD = int(0.8 * RESIZED_HEIGHT)
    road_contour = [point for point in road_contour if point[0][1] < MIN_Y_COORD]

    if len(road_contour) < 2:
        print("[WARN] Filtered road contour is too small after removing bottom points.")
        return None, None, None

    contour_points = np.array(road_contour).reshape(-1, 2)
    horizon_point = contour_points[np.argmin(contour_points[:, 1])]

    (
        left_boundary_points,
        right_boundary_points,
    ) = classify_boundaries_using_horizontal_bins(
        contour_points[contour_points[:, 1] < center_x], RESIZED_HEIGHT
    )

    for point in contour_points[contour_points[:, 1] >= center_x]:
        (
            left_boundary_points
            if point[0] < horizon_point[0]
            else right_boundary_points
        ).append(point)
    left_boundary_points, right_boundary_points = np.array(
        left_boundary_points
    ), np.array(right_boundary_points)

    overlay = create_overlay(
        image,
        road_mask,
        left_boundary_points,
        right_boundary_points,
        publish_image,
        point_coords,
    )

    return overlay, left_boundary_points, right_boundary_points


def classify_boundaries_using_horizontal_bins(
    contour_points, frame_height, num_bins=50
):
    if len(contour_points) == 0:
        return [], []

    y_min = contour_points[:, 1].min()
    bin_height = (frame_height - y_min) // num_bins
    left_boundary_points, right_boundary_points = [], []

    for i in range(num_bins):
        y_lower_limit = y_min + i * bin_height
        y_upper_limit = y_min + (i + 1) * bin_height

        bin_points = contour_points[
            (contour_points[:, 1] > y_lower_limit)
            & (contour_points[:, 1] <= y_upper_limit)
        ]

        if len(bin_points) > 0:
            mean_x = bin_points[:, 0].mean()
            for point in bin_points:
                (
                    left_boundary_points if point[0] < mean_x else right_boundary_points
                ).append(point)

    return left_boundary_points, right_boundary_points


def create_overlay(
    image,
    binary_mask_np,
    left_boundary_points,
    right_boundary_points,
    publish_image,
    point_coords,
):
    if not publish_image:
        return None

    overlay = image.copy()
    mask = binary_mask_np.reshape(image.shape[0], image.shape[1], 1).repeat(3, axis=2)
    overlay = cv2.addWeighted(
        overlay.astype(np.uint8),
        0.8,
        mask * np.array([0, 255, 0], dtype=np.uint8),
        0.2,
        0,
    )

    for point in point_coords:
        cv2.circle(
            overlay, tuple(map(int, point)), radius=10, color=(255, 0, 0), thickness=-1
        )

    for point in left_boundary_points:
        cv2.circle(overlay, tuple(point), radius=3, color=(0, 255, 0), thickness=-2)
    for point in right_boundary_points:
        cv2.circle(overlay, tuple(point), radius=3, color=(0, 0, 255), thickness=-2)

    return overlay


class Samv2ImageSegmentation(Node):
    def __init__(self):
        super().__init__("samv2_image_segmentation")
        self.get_logger().info("Initializing RoadSegmentation class.")

        self.bridge = CvBridge()

        self.create_subscription(Image, CAMERA_TOPIC, self.callback, 1)

        self.image_pub = self.create_publisher(Image, SAM_SEGMENTATION_MASK_TOPIC, 1)
        self.left_boundary_pub = self.create_publisher(DetectedRoadArea, SAM_LEFT_CONTOUR_TOPIC, 1)
        self.right_boundary_pub = self.create_publisher(DetectedRoadArea, SAM_RIGHT_CONTOUR_TOPIC, 1)

        self.ros_image = None
        self.publish_image = True
        self.detected_objects = []
        self.centerline_points = None
        self.create_timer(0.1, self.process_loop)

    def callback(self, ros_image):
        self.ros_image = ros_image

    def process_loop(self):
        """Process the image if available, called periodically by a timer."""
        if self.ros_image:
            self.image_callback()

    @timer
    def image_callback(self):

        img = self.bridge.imgmsg_to_cv2(self.ros_image, desired_encoding="bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        overlay, left_boundary, right_boundary = process_image(
            img,
            self.detected_objects,
            self.publish_image,
        )

        if self.publish_image and overlay is not None:
            self.publish_image_topic(self.ros_image, overlay)

        # Publish boundary points
        if (left_boundary is not None) and left_boundary.size > 0:
            # Remove points that are at the image boundary (x == width or y == height or 0)
            mask = (
                (left_boundary[:, 0] != 0)
                & (left_boundary[:, 0] != img.shape[1])
                & (left_boundary[:, 1] != 0)
                & (left_boundary[:, 1] != img.shape[0])
            )
            left_boundary = left_boundary[mask]
            # Resize the left boundary points to match the original image size
            left_boundary = left_boundary.astype(np.float32)
            left_boundary[:, 0] *= ORIGINAL_WIDTH / RESIZED_WIDTH
            left_boundary[:, 1] *= ORIGINAL_HEIGHT / RESIZED_HEIGHT
            self.publish_boundary(left_boundary, self.left_boundary_pub, self.ros_image.header.stamp)

        if (right_boundary is not None) and right_boundary.size > 0:
            # Remove points that are at the image boundary (x == width or y == height or 0)
            mask = (
                (right_boundary[:, 0] != 0)
                & (right_boundary[:, 0] != img.shape[1])
                & (right_boundary[:, 1] != 0)
                & (right_boundary[:, 1] != img.shape[0])
            )
            right_boundary = right_boundary[mask]
            # Resize the right boundary points to match the original image size
            right_boundary = right_boundary.astype(np.float32)
            right_boundary[:, 0] *= ORIGINAL_WIDTH / RESIZED_WIDTH
            right_boundary[:, 1] *= ORIGINAL_HEIGHT / RESIZED_HEIGHT
            self.publish_boundary(right_boundary, self.right_boundary_pub, self.ros_image.header.stamp)

    def publish_image_topic(self, ros_image, overlay):
        msg = Image()
        msg.header.stamp = ros_image.header.stamp
        msg.height, msg.width = overlay.shape[:2]
        msg.encoding = "rgb8"
        msg.data = overlay.tobytes()
        self.image_pub.publish(msg)

    def publish_boundary(self, boundary, publisher, stamp):
        boundary_msg = DetectedRoadArea()
        boundary_msg.header.stamp = stamp
        boundary_msg.road_area.data = [float(point) for point in boundary.flatten()]
        publisher.publish(boundary_msg)


if __name__ == "__main__":
    rclpy.init()
    node = Samv2ImageSegmentation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
