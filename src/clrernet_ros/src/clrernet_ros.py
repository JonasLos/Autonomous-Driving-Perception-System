#!/usr/bin/env python3
# type: ignore


import rospy
import cv2
import torch
from mmdet.apis import init_detector
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from inference import inference_one_image
from clrernet.libs.utils.visualizer import visualize_lanes
import ros_numpy
from functools import wraps
from mmengine.config import Config
import os
from pathlib import Path


# Configuration parameters
score_thr = 0.3
device = "cuda:0" if torch.cuda.is_available() else "cpu"
config = "/home/dev/Documents/Autonomous-Driving-Perception-System/src/clrernet_ros/src/clrernet/configs/clrernet/culane/clrernet_culane_dla34_ema.py"
checkpoint = "/home/dev/Documents/Autonomous-Driving-Perception-System/src/clrernet_ros/src/clrernet_culane_dla34_ema.pth"


def timer(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = rospy.Time.now().to_sec()
        result = func(*args, **kwargs)
        end_time = rospy.Time.now().to_sec()
        print(f"{func.__name__} executed in {end_time - start_time:.4f} seconds")
        return result

    return wrapper


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

        self.image_sub = rospy.Subscriber(
            "resized/camera_fl/image_color", Image, self.image_callback
        )
        self.image_pub = rospy.Publisher(
            "/clrernet_detection/output_image", Image, queue_size=1
        )

    @timer
    def image_callback(self, data: Image):
        try:
            rospy.loginfo("Received new image data for processing")
            img = ros_numpy.numpify(data)
            img_resized = cv2.resize(img, (1640, 590))  # Resize image for model

            # Run inference on resized image
            src, preds = inference_one_image(self.model, img_resized)
            print(len(preds))
            # Visualize the lanes and prepare to publish
            result_image = visualize_lanes(src, preds)

            # Convert result image to ROS Image message
            result_image_msg = self.convert_image_to_ros(
                result_image, data.header.stamp
            )
            
            # Publish processed image
            self.image_pub.publish(result_image_msg)

            # Publish bounding boxes or lanes as needed
            # self.publish_lane_bboxes(preds, data.header.stamp)

        except Exception as e:
            rospy.logerr(f"Error in image callback: {e}")

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

    def publish_lane_bboxes(self, preds, stamp):
        # Create and publish bounding boxes for detected lanes
        rospy.loginfo("Publishing lane bounding boxes (not implemented yet)")
        msg = Header()
        msg.stamp = stamp
        msg.frame_id = "lane_detection"

        # Publish detection data (bounding boxes or other relevant data from preds)
        self.lane_pub.publish(msg)


if __name__ == "__main__":
    print("Calling rospy.init_node...")
    rospy.init_node("test", anonymous=False)
    print("rospy node initialized")

    print("Creating CLLanes instance...")
    CLLanes()
    print("CLLanes instance created")

    print("Calling rospy.spin()...")
    rospy.spin()
    print("rospy.spin() completed")
