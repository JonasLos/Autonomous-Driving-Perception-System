#!/usr/bin/env python3
# type: ignore

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

import cv2
import ros2_numpy as ros_numpy
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import torch
import yaml
from clrernet.libs.utils.visualizer import visualize_lanes
from inference import inference_one_image
from mmdet.apis import init_detector
from sensor_msgs.msg import Image

from clrernet_msgs.msg import LanePoint, LanePoints

device = "cuda:0" if torch.cuda.is_available() else "cpu"

BASE_PATH = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(
    BASE_PATH,
    "clrernet",
    "configs",
    "clrernet",
    "culane",
    "clrernet_culane_dla34_ema.py",
)
CHECKPOINT_PATH = os.path.join(BASE_PATH, "clrernet_culane_dla34_ema.pth")
TOPICS_PATH = os.path.join(
    get_package_share_directory("perception_common"), "topics.yaml"
)
CONF_THRESHOLD = 0.45

# Load YAML config
with open(TOPICS_PATH, "r") as f:
    config = yaml.safe_load(f)

# === Initialize topics ===
CAMERA_TOPIC = config["topics"]["raw"]["camera"]
CLRERNET_LANE_MASK_TOPIC = config["topics"]["clrernet"]["lane_mask"]
CLRERNET_ALL_LANES_TOPIC = config["topics"]["clrernet"]["all_lanes"]


class ClrernetLaneDetection(Node):
    def __init__(self):
        super().__init__("clrernet_lane_detection")
        dummy_file = Path("dataset/culane/list/test.txt")
        dummy_file.parent.mkdir(parents=True, exist_ok=True)
        if not dummy_file.exists():
            dummy_file.write_text("dummy/path/to/image.jpg\n")

        self.model = init_detector(CONFIG_PATH, CHECKPOINT_PATH, device=device)
        self.model.cfg.model.bbox_head.test_cfg.conf_threshold = CONF_THRESHOLD
        self.w, self.h = None, None

        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(Image, CAMERA_TOPIC, self.callback, 1)

        # Publishers
        self.lane_mask_pub = self.create_publisher(Image, CLRERNET_LANE_MASK_TOPIC, 1)
        self.all_lanes_pub = self.create_publisher(LanePoints, CLRERNET_ALL_LANES_TOPIC, 1)

    def callback(self, data: Image):
        self.handle_image(data)

    def handle_image(self, data: Image):
        # Convert ROS Image to OpenCV image
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.w, self.h = img.shape[1], img.shape[0]
        img_resized = cv2.resize(img, (1640, 590))

        src, preds = inference_one_image(self.model, img_resized)
        debug_img = visualize_lanes(src, preds)
        result_image_rescaled = cv2.resize(debug_img, (self.w, self.h))
        result_image_msg = self.convert_image_to_ros(
            result_image_rescaled, data.header.stamp
        )

        self.lane_mask_pub.publish(result_image_msg)
        self.publish_all_lanes(data.header, preds, is_front=True)

    def convert_image_to_ros(self, img, stamp):
        img_msg = Image()
        img_msg.header.stamp = stamp
        img_msg.height, img_msg.width = img.shape[:2]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = False
        img_msg.step = 3 * img.shape[1]
        img_msg.data = img.tobytes()
        return img_msg

    def publish_all_lanes(self, header, lanes, is_front: bool):
        all_points = []
        scale_x = self.w / 1640
        scale_y = self.h / 590
        for lane_id, lane in enumerate(lanes):
            for pt in lane:
                if pt[0] != 0 or pt[1] != 0:
                    all_points.append(
                        LanePoint(
                            x=int(pt[0] * scale_x),
                            y=int(pt[1] * scale_y),
                            lane_id=lane_id,
                        )
                    )

        msg = LanePoints()
        msg.header = header
        msg.points = all_points

        self.all_lanes_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClrernetLaneDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
