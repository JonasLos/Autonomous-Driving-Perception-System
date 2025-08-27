#!/usr/bin/env python3
# type: ignore

import os
from typing import List

import cv2
import numpy as np
import ros_numpy
import rospy
import torch
import yaml
from PIL import Image as PILImage
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from ultralytics import YOLO

from yolov9_ros.msg import Bbox, BboxList

# Initialize CUDA device early
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

if device == "cpu":
    print("Using CPU and not GPU")
if device != torch.device("cpu"):
    torch.cuda.init()  # Ensure CUDA is initialized early
torch.cuda.set_per_process_memory_fraction(0.05, device=torch.device("cuda:0"))

# Get the paths of the current script and other required files
BASE_PATH = os.path.dirname(os.path.abspath(__file__))
WEIGHTS_PATH = os.path.join(BASE_PATH, "..", "best.pt")
CLASS_AVERAGES_PATH = os.path.join(BASE_PATH, "class_averages.yaml")
SUPPRESSED_CLASSES_PATH = os.path.join(BASE_PATH, "suppressed_classes.yaml")
TOPICS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "topics.yaml"
)
with open(TOPICS_PATH, "r") as f:
    topic_config = yaml.safe_load(f)

# Assign topic variables
CAMERA_TOPIC = topic_config["topics"]["raw"]["camera"]
YOLO_BBOX_TOPIC = topic_config["topics"]["yolo"]["bbox"]
YOLO_IMAGE_TOPIC = topic_config["topics"]["yolo"]["image"]

# Configuration parameters
img_size = 640
conf_thres = 0.4
view_img = True

# Average Class Dimensions
with open(CLASS_AVERAGES_PATH, "r", encoding="utf-8") as file:
    average_dimensions = yaml.safe_load(file)

with open(SUPPRESSED_CLASSES_PATH, "r", encoding="utf-8") as file:
    suppressed_classes = yaml.safe_load(file)["suppressed_classes"]


class Yolov9ObjectDetection:
    def __init__(self) -> None:
        self.model = YOLO(WEIGHTS_PATH).to(device)
        self.model.conf = 0.5
        self.names: List[str] = self.model.names

        # Subscribers
        self.image_sub = rospy.Subscriber(
            CAMERA_TOPIC,
            Image,
            self.callback,
            queue_size=1,
        )

        # Publishers
        self.image_pub = rospy.Publisher(YOLO_IMAGE_TOPIC, Image, queue_size=1)
        self.bboxInfo_pub = rospy.Publisher(YOLO_BBOX_TOPIC, BboxList, queue_size=1)

    # Add the classify_traffic_light function
    def classify_traffic_light(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Define color ranges for yellow, green, red
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])

        green_lower = np.array([40, 50, 50])
        green_upper = np.array([90, 255, 255])

        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([160, 100, 100])
        red_upper2 = np.array([180, 255, 255])

        # Masking
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Count the number of pixels for each color
        yellow_pixels = cv2.countNonZero(yellow_mask)
        green_pixels = cv2.countNonZero(green_mask)
        red_pixels = cv2.countNonZero(red_mask)

        # Determine the color with the maximum number of pixels
        if yellow_pixels > green_pixels and yellow_pixels > red_pixels:
            return "Yellow"
        elif green_pixels > yellow_pixels and green_pixels > red_pixels:
            return "Green"
        elif red_pixels > yellow_pixels and red_pixels > green_pixels:
            return "Red"
        else:
            return "Unknown"

    def callback(self, data: Image) -> None:
        img: np.ndarray = ros_numpy.numpify(data)  # Image size is (772, 1032, 3)
        img_resized: np.ndarray = cv2.resize(
            img, (img_size, img_size)
        )  # Image resized to (640, 640)
        img_without_green_box = img_resized.copy()
        img_rgb: np.ndarray = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)

        # Normalize and prepare the tensor
        img_tensor: torch.Tensor = (
            torch.from_numpy(img_rgb).to(device, non_blocking=True).float()
        )
        img_tensor = img_tensor.permute(2, 0, 1).unsqueeze(0) / 255.0

        with torch.no_grad(), torch.cuda.amp.autocast():
            detections = self.model(img_tensor)[0]
            bboxes: np.ndarray = detections.boxes.xyxy.cpu().numpy().astype(int)
            class_ids: np.ndarray = detections.boxes.cls.cpu().numpy().astype(int)
            confidences: np.ndarray = detections.boxes.conf.cpu().numpy()

            # Filter out detections below the confidence threshold
            filtered_indices = [
                i for i, conf in enumerate(confidences) if conf > conf_thres
            ]
            filtered_bboxes = bboxes[filtered_indices]
            filtered_class_ids = class_ids[filtered_indices]
            filtered_confidences = confidences[filtered_indices]

            for bbox, class_id, conf in zip(
                filtered_bboxes, filtered_class_ids, filtered_confidences
            ):
                x1, y1, x2, y2 = bbox
                label: str = f"{self.names[class_id]}: {conf:.2f}"

                # Suppress irrelevant classes
                if self.names[class_id] in suppressed_classes:
                    continue

                # If the detected object is a traffic light and confidence is greater than 50%
                # Then make the bounding box
                if self.names[class_id] == "traffic light" and conf > 0.5:
                    cv2.rectangle(img_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    roi = img_without_green_box[y1:y2, x1:x2]
                    color = self.classify_traffic_light(roi)
                    cv2.putText(
                        img_resized,
                        color,
                        (x1, y2 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 0),  # Black
                        2,
                    )

                cv2.rectangle(img_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    img_resized,
                    label,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 225),
                    2,
                )

            self.publish_bboxes(
                detections.boxes.data[filtered_indices],
                data.header.stamp,
            )
            if view_img:
                self.publish_image(img_resized, data.header.stamp)

    def publish_bboxes(self, detections: torch.Tensor, stamp: rospy.Time) -> None:
        # Ensure the detections data is in the expected format
        msg = BboxList()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.Bboxes = []

        for bbox in detections:
            # Parse detection data
            x1, y1, x2, y2, conf, cls = bbox  # Convert tensor to list

            if conf > conf_thres:  # Filter detections based on confidence
                # Scale bounding box coordinates back to the original image dimensions
                x_min = x1 * (1032 * 2 / 640)
                y_min = y1 * (772 * 2 / 640)
                x_max = x2 * (1032 * 2 / 640)
                y_max = y2 * (772 * 2 / 640)

                # Create a BboxCenter message for the bounding box
                bbox_msg = Bbox()
                bbox_msg.x_min = x_min
                bbox_msg.y_min = y_min
                bbox_msg.x_max = x_max
                bbox_msg.y_max = y_max
                bbox_msg.confidence = float(conf)  # Confidence as float
                bbox_msg.class_id = int(cls)  # Class ID as integer

                msg.Bboxes.append(bbox_msg)  # Append the bbox message to the list

        # Publish the message
        self.bboxInfo_pub.publish(msg)

    def publish_image(self, img: np.ndarray, stamp: rospy.Time) -> None:
        img_pil: PILImage = PILImage.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        msg: Image = Image()
        msg.header.stamp = stamp
        msg.height = img_pil.height
        msg.width = img_pil.width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * img_pil.width
        msg.data = np.array(img_pil).tobytes()
        self.image_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("YOLOv9 Object Detection", anonymous=True)
    Yolov9ObjectDetection()
    rospy.spin()
