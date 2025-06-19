#!/usr/bin/env python3
# type: ignore


from pathlib import Path

import cv2
import ros_numpy
import rospy
import torch
from clrernet.libs.utils.visualizer import visualize_lanes
from inference import inference_one_image
from mmdet.apis import init_detector
from sensor_msgs.msg import Image

from src.configs import (  # Assuming you're using same topic config
    LEFT_LANE_TOPIC,
    RIGHT_LANE_TOPIC,
)
from src.utils import timer
from ultrafastv2_ros.msg import LanePoint, LanePoints

# Configuration parameters
score_thr = 0.3
device = "cuda:0" if torch.cuda.is_available() else "cpu"
config = "/home/dev/Documents/Autonomous-Driving-Perception-System/src/clrernet_ros/src/clrernet/configs/clrernet/culane/clrernet_culane_dla34_ema.py"
checkpoint = "/home/dev/Documents/Autonomous-Driving-Perception-System/src/clrernet_ros/src/clrernet_culane_dla34_ema.pth"


# Lane Detection Class
class CLLanes:
    def __init__(self):

        # This path is relative to the script's runtime CWD, which is usually the project root or the directory from which you run the launch command
        cwd_relative_dummy_file = Path("dataset/culane/list/test.txt")

        # Create full path on disk
        cwd_relative_dummy_file.parent.mkdir(parents=True, exist_ok=True)
        if not cwd_relative_dummy_file.exists():
            cwd_relative_dummy_file.write_text("dummy/path/to/image.jpg\n")

        self.model = init_detector(config, checkpoint, device=device)

        # Subscribers
        self.image_sub = rospy.Subscriber(
            "/camera_fl/image_color", Image, self.image_callback
        )

        # Publishers
        self.image_pub = rospy.Publisher(
            "/clrernet_detection/output_image", Image, queue_size=1
        )
        self.w, self.h = None, None
        self.left_lane_boundary_pub = rospy.Publisher(
            LEFT_LANE_TOPIC, LanePoints, queue_size=1
        )
        self.right_lane_boundary_pub = rospy.Publisher(
            RIGHT_LANE_TOPIC, LanePoints, queue_size=1
        )
        self.all_lane_pub = rospy.Publisher(LEFT_LANE_TOPIC, LanePoints, queue_size=1)

    @timer
    def image_callback(self, data: Image):
        rospy.loginfo("Received new image data for processing")
        img = ros_numpy.numpify(data)
        self.w, self.h = img.shape[1], img.shape[0]
        img_resized = cv2.resize(img, (1640, 590))  # Resize image for model

        # Run inference on resized image
        src, preds = inference_one_image(self.model, img_resized)

        # Visualize the lanes and prepare to publish
        result_image = visualize_lanes(src, preds)

        result_image_rescaled = cv2.resize(result_image, (self.w, self.h))

        # Convert result image to ROS Image message
        result_image_msg = self.convert_image_to_ros(
            result_image_rescaled, data.header.stamp
        )

        # Publish processed image
        self.image_pub.publish(result_image_msg)

        self.publish_lane_boundaries(preds, data.header)

    def convert_image_to_ros(self, img, stamp):
        # Convert OpenCV image to ROS Image message
        rospy.loginfo(f"Converting image to ROS message with shape: {img.shape}")
        img_msg = Image()
        img_msg.header.stamp = stamp
        img_msg.height, img_msg.width = img.shape[:2]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = False
        img_msg.step = 3 * img.shape[1]
        img_msg.data = img.tobytes()
        return img_msg

    def publish_lane_boundaries(self, preds, header):
        scale_x = self.w / 1640
        scale_y = self.h / 590

        all_lane_points = []
        for lane_id, lane in enumerate(preds):
            lane_points = [
                LanePoint(x=int(x * scale_x), y=int(y * scale_y), lane_id=lane_id)
                for (x, y) in lane
                if x != 0 or y != 0
            ]
            all_lane_points.extend(lane_points)

        self.all_lane_pub.publish(LanePoints(header=header, points=all_lane_points))
        rospy.loginfo(f"Published {len(all_lane_points)} total lane points.")


if __name__ == "__main__":
    print("Calling rospy.init_node...")
    rospy.init_node("test", anonymous=False)
    CLLanes()
    rospy.spin()
