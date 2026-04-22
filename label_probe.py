#!/usr/bin/env python3
import yaml, os
from ament_index_python.packages import get_package_share_directory
from collections import Counter
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sphereformer_ros import SphereformerLidarSegmentation, CONFIG_PATH, CHECKPOINT_PATH
import sphereformer_ros as s

cfg = yaml.safe_load(open(os.path.join(
    get_package_share_directory("sphereformer_ros"),
    "SphereFormer_changes", "semantic-kitti.yaml")))
label_inv = cfg["learning_map_inv"]
names = cfg["labels"]

class P(Node):
    def __init__(self):
        super().__init__("label_probe")
        self.m = SphereformerLidarSegmentation(CONFIG_PATH, CHECKPOINT_PATH)
        self.sub = self.create_subscription(PointCloud2, "/lidar_tc/velodyne_points", self.cb, 1)
        self.done = False

    def cb(self, msg):
        print("CROP", s.lim_x, s.lim_y, s.lim_z)
        _, labels = self.m.inference_from_ros_message(msg, self.m.model)
        lab = labels.detach().cpu().numpy()
        top = Counter(lab.tolist()).most_common(12)
        total = sum(c for _, c in top)
        print("TOTAL", total)
        for learned, cnt in top:
            raw = label_inv.get(int(learned))
            name = names.get(raw, "unknown")
            pct = 100.0 * cnt / total
            print("  learned=%2d  count=%5d  pct=%4.1f%%  raw=%s  name=%s" % (learned, cnt, pct, raw, name))
        self.done = True

rclpy.init()
n = P()
try:
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=5.0)
finally:
    n.destroy_node()
    n.m.destroy_node()
    rclpy.shutdown()
