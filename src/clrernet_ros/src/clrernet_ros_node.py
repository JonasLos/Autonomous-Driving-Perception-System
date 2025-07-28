#!/usr/bin/env python3
# type: ignore

import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import ros_numpy
import rospy
import torch

# from clrernet.libs.utils.visualizer import visualize_lanes
from inference import inference_one_image
from mmdet.apis import init_detector
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from clrernet_ros.msg import LanePoint, LanePoints

# Configuration parameters
score_thr = 0.35
device = "cuda:0" if torch.cuda.is_available() else "cpu"

# Create these paths relative to the script's runtime CWD
base_path = os.path.dirname(os.path.abspath(__file__))
config_path = os.path.join(
    base_path,
    "clrernet",
    "configs",
    "clrernet",
    "culane",
    "clrernet_culane_dla34_ema.py",
)
checkpoint_path = os.path.join(base_path, "clrernet_culane_dla34_ema.pth")
reference_points = [
    (700, 585),
    (800, 585),
    (900, 585),
    (1000, 585),
    (1100, 585),
    (1200, 585),
]


# Lane Detection Class
class CLLanes:
    def __init__(self):

        rospy.loginfo("🟡 Inside CLLanes init")

        # This path is relative to the script's runtime CWD, which is usually the project root or the directory from which you run the launch command
        cwd_relative_dummy_file = Path("dataset/culane/list/test.txt")

        # Create full path on disk
        cwd_relative_dummy_file.parent.mkdir(parents=True, exist_ok=True)
        if not cwd_relative_dummy_file.exists():
            cwd_relative_dummy_file.write_text("dummy/path/to/image.jpg\n")

        self.model = init_detector(config_path, checkpoint_path, device=device)

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
            "/lane_detection/current_lane_left_boundary", LanePoints, queue_size=1
        )
        self.right_lane_boundary_pub = rospy.Publisher(
            "/lane_detection/current_lane_right_boundary", LanePoints, queue_size=1
        )
        # self.all_lane_pub = rospy.Publisher(
        #     "/lane_detection/current_lane_left_boundary", LanePoints, queue_size=1
        # )

    # @timer
    # def image_callback(self, data: Image):
    #     rospy.loginfo("Received new image data for processing")
    #     img = ros_numpy.numpify(data)
    #     self.w, self.h = img.shape[1], img.shape[0]
    #     img_resized = cv2.resize(img, (1640, 590))  # Resize image for model
    #     # Apply image enhancement to improve lane detection
    #     # img_resized = self.adjust_gamma(img_resized)
    #     # Optionally also use: img_resized = gray_highlight(img_resized)

    #     # Run inference on resized image
    #     src, preds = inference_one_image(self.model, img_resized)

    #     left_lane_points, right_lane_points = self.get_closest_lane_boundaries(
    #         preds, reference_points
    #     )

    #     # === Debug Visualization of All Lanes ===

    #     # Colors for visualization
    #     LEFT_COLOR = (0, 255, 0)  # Green
    #     RIGHT_COLOR = (255, 0, 0)  # Blue
    #     OTHER_COLOR = (0, 165, 255)  # Orange

    #     # Copy image to draw debug output
    #     debug_img = src.copy()

    #     # Extract selected lane IDs
    #     left_lane_id = left_lane_points[0].lane_id if left_lane_points else -1
    #     right_lane_id = right_lane_points[0].lane_id if right_lane_points else -1

    #     for lane_id, lane in enumerate(preds):
    #         if lane is None or len(lane) == 0:
    #             continue

    #         lane_np = np.array(lane, dtype=int)

    #         # Choose color
    #         if lane_id == left_lane_id:
    #             color = LEFT_COLOR
    #         elif lane_id == right_lane_id:
    #             color = RIGHT_COLOR
    #         else:
    #             color = OTHER_COLOR

    #         # Draw lane points
    #         for pt in lane_np:
    #             cv2.circle(debug_img, tuple(pt), 4, color, -1)

    #         # Draw lane ID text
    #         if len(lane_np) > 0:
    #             mid_idx = len(lane_np) // 2
    #             cv2.putText(
    #                 debug_img,
    #                 f"ID:{lane_id}",
    #                 tuple(lane_np[mid_idx]),
    #                 cv2.FONT_HERSHEY_SIMPLEX,
    #                 0.6,
    #                 color,
    #                 2,
    #             )

    #     # Visualize only left and right lanes
    #     # result_image = visualize_lanes(src, filtered_preds)
    #     result_image = debug_img

    #     result_image_rescaled = cv2.resize(result_image, (self.w, self.h))
    #     # === Add reference points to the image ===
    #     for x, y in reference_points:
    #         x_scaled = int(x * (self.w) / 1640)
    #         y_scaled = int(y * (self.h) / 590)
    #         cv2.circle(
    #             result_image_rescaled,
    #             (x_scaled, y_scaled),
    #             15,
    #             (0, 255, 255),
    #             -1,
    #         )  # yellow dot

    #     # Convert result image to ROS Image message
    #     result_image_msg = self.convert_image_to_ros(
    #         result_image_rescaled, data.header.stamp
    #     )

    #     # Publish processed image
    #     self.image_pub.publish(result_image_msg)
    #     # Publish lane boundaries
    #     self.publish_lane_points(data.header, left_lane_points, right_lane_points)

    def image_callback(self, data: Image):
        try:
            img = ros_numpy.numpify(data)
            self.w, self.h = img.shape[1], img.shape[0]
            img_resized = cv2.resize(img, (1640, 590))

            # Inference
            src, preds = inference_one_image(self.model, img_resized)

            # Lane selection
            left_lane_points, right_lane_points = self.get_closest_lane_boundaries(
                preds, reference_points
            )
            left_id = left_lane_points[0].lane_id if left_lane_points else -1
            right_id = right_lane_points[0].lane_id if right_lane_points else -1

            # Draw
            debug_img = src.copy()
            LEFT_COLOR, RIGHT_COLOR, OTHER_COLOR = (
                (0, 255, 0),
                (255, 0, 0),
                (0, 165, 255),
            )

            for lane_id, lane in enumerate(preds):
                # lane can be list of (x,y) or np.ndarray; handle both
                if lane is None:
                    continue
                if isinstance(lane, np.ndarray):
                    if lane.size == 0:
                        continue
                    lane_np = lane.astype(int)
                else:
                    if len(lane) == 0:
                        continue
                    lane_np = np.asarray(lane, dtype=int)

                color = (
                    LEFT_COLOR
                    if lane_id == left_id
                    else (RIGHT_COLOR if lane_id == right_id else OTHER_COLOR)
                )

                for pt in lane_np:
                    cv2.circle(debug_img, tuple(pt), 4, color, -1)

                mid_idx = len(lane_np) // 2
                cv2.putText(
                    debug_img,
                    f"ID:{lane_id}",
                    tuple(lane_np[mid_idx]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )

            # Rescale back to original size
            result_image_rescaled = cv2.resize(debug_img, (self.w, self.h))

            # Reference dots
            for x, y in reference_points:
                x_scaled = int(x * self.w / 1640.0)
                y_scaled = int(y * self.h / 590.0)
                cv2.circle(
                    result_image_rescaled, (x_scaled, y_scaled), 15, (0, 255, 255), -1
                )

            # Publish image
            result_image_msg = self.convert_image_to_ros(
                result_image_rescaled, data.header.stamp
            )
            self.image_pub.publish(result_image_msg)

            # Publish lane points (safe if either side empty)
            self.publish_lane_points(data.header, left_lane_points, right_lane_points)

        except Exception as e:
            rospy.logerr("image_callback failed: %s", repr(e))

    def adjust_gamma(self, image, gamma=0.5):
        table = np.array(
            [((i / 255.0) ** (1.0 / gamma)) * 255 for i in np.arange(0, 256)]
        ).astype("uint8")
        return cv2.LUT(image, table)

    def apply_clahe_bgr(self, image):
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(8, 8))
        cl = clahe.apply(l)
        merged = cv2.merge((cl, a, b))
        return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)

    def gray_highlight(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

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

    def publish_lane_points(self, header, left_lane_points, right_lane_points):
        scale_x = self.w / 1640
        scale_y = self.h / 590

        scaled_left = [
            LanePoint(x=int(p.x * scale_x), y=int(p.y * scale_y), lane_id=p.lane_id)
            for p in left_lane_points
        ]

        scaled_right = [
            LanePoint(x=int(p.x * scale_x), y=int(p.y * scale_y), lane_id=p.lane_id)
            for p in right_lane_points
        ]

        new_header = Header()
        new_header.stamp = header.stamp

        self.left_lane_boundary_pub.publish(
            LanePoints(header=new_header, points=scaled_left)
        )
        self.right_lane_boundary_pub.publish(
            LanePoints(header=new_header, points=scaled_right)
        )
        rospy.loginfo("Published scaled left and right lane boundaries.")

    # def get_closest_lane_boundaries(self, lanes_points, reference_points):
    #     left_lane_id = None
    #     right_lane_id = None
    #     min_left_dist = float("inf")
    #     min_right_dist = float("inf")

    #     ref_xs = [x for x, _ in reference_points]
    #     center_x = sum(ref_xs) / len(ref_xs)  # Approximate vehicle center

    #     for lane_id, lane in enumerate(lanes_points):
    #         if lane is None or len(lane) == 0:
    #             continue
    #         lane_xs = [p[0] for p in lane if p[0] != 0 or p[1] != 0]
    #         if not lane_xs:
    #             continue

    #         avg_lane_x = sum(lane_xs) / len(lane_xs)
    #         dist = abs(avg_lane_x - center_x)

    #         if avg_lane_x < center_x and dist < min_left_dist:
    #             min_left_dist = dist
    #             left_lane_id = lane_id
    #         elif avg_lane_x > center_x and dist < min_right_dist:
    #             min_right_dist = dist
    #             right_lane_id = lane_id

    #     left_lane_points = []
    #     right_lane_points = []

    #     if left_lane_id is not None:
    #         left_lane_points = [
    #             LanePoint(x=int(p[0]), y=int(p[1]), lane_id=left_lane_id)
    #             for p in lanes_points[left_lane_id]
    #             if p[0] != 0 or p[1] != 0
    #         ]

    #     if right_lane_id is not None:
    #         right_lane_points = [
    #             LanePoint(x=int(p[0]), y=int(p[1]), lane_id=right_lane_id)
    #             for p in lanes_points[right_lane_id]
    #             if p[0] != 0 or p[1] != 0
    #         ]

    #     return left_lane_points, right_lane_points

    def get_closest_lane_boundaries(self, lanes_points, reference_points):
        def lane_distance(lane, refs):
            if lane is None or len(lane) == 0:
                return float("inf")
            total_dist = 0
            for rx, ry in refs:
                min_dist = float("inf")
                for lx, ly in lane:
                    dist = math.hypot(rx - lx, ry - ly)
                    if dist < min_dist:
                        min_dist = dist
                total_dist += min_dist
            return total_dist / len(refs)

        best_left = None
        best_right = None
        best_left_dist = float("inf")
        best_right_dist = float("inf")

        center_x = sum([x for x, _ in reference_points]) / len(reference_points)

        for lane_id, lane in enumerate(lanes_points):
            if lane is None or len(lane) == 0:
                continue

            lane_np = np.array(lane)
            avg_x = np.mean(lane_np[:, 0])

            dist = lane_distance(lane, reference_points)

            if avg_x < center_x and dist < best_left_dist:
                best_left_dist = dist
                best_left = (lane_id, lane)
            elif avg_x > center_x and dist < best_right_dist:
                best_right_dist = dist
                best_right = (lane_id, lane)

        def format_lane(lane_id, lane):
            return [
                LanePoint(x=int(x), y=int(y), lane_id=lane_id)
                for x, y in lane
                if x != 0 or y != 0
            ]

        left_lane_points = format_lane(*best_left) if best_left else []
        right_lane_points = format_lane(*best_right) if best_right else []

        return left_lane_points, right_lane_points


if __name__ == "__main__":
    print("Calling rospy.init_node...")
    rospy.init_node("lane_detection_node", anonymous=True)
    print("after rospy.init_node")
    CLLanes()
    print("CLLanes initialized, spinning...")
    rospy.spin()
