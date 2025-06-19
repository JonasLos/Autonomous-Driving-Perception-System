#!/usr/bin/env python3

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial import KDTree
from sensor_msgs.msg import PointCloud2

from src.configs import LEFT_LANE_BOUNDARY_TOPIC, LEFT_LANE_TOPIC, LIDAR_2D_PROJ_TOPIC
from src.utils import timer
from ultrafastv2_ros.msg import LanePoints

pixel_lim = 5  # pixels


class LaneTo3D:
    def __init__(self):
        rospy.init_node("lane_2d_to_3d")

        # Subscribers
        self.sub_proj = rospy.Subscriber(
            LIDAR_2D_PROJ_TOPIC, PointCloud2, self.proj_callback
        )
        self.sub_lanes = rospy.Subscriber(
            LEFT_LANE_TOPIC, LanePoints, self.lanes_callback
        )

        self.lane_3d_pub = rospy.Publisher(
            LEFT_LANE_BOUNDARY_TOPIC, PointCloud2, queue_size=1
        )

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

        self.pc_arr = None
        self.uv = None
        self.header = None

    def proj_callback(self, msg):
        points_list = list(
            pc2.read_points(msg, field_names=("x", "y", "z", "u", "v"), skip_nans=True)
        )
        if not points_list:
            return
        points_np = np.array(points_list)
        self.pc_arr = points_np[:, :3]
        self.uv = points_np[:, 3:5]
        self.header = msg.header

    @timer
    def lanes_callback(self, msg: LanePoints):
        if self.pc_arr is None or self.uv is None:
            rospy.logwarn("Projected LiDAR data not yet received.")
            return

        lane_uv = np.array([[p.x, p.y] for p in msg.points])
        if lane_uv.shape[0] == 0:
            rospy.logwarn("No lane points received.")
            return

        # Match using KDTree
        tree = KDTree(self.uv)
        matched_indices = []
        for uv in lane_uv:
            matches = tree.query_ball_point(uv, pixel_lim)
            matched_indices.extend(matches)

        matched_indices = list(set(matched_indices))
        matched_3d = self.pc_arr[matched_indices]

        if matched_3d.size == 0:
            rospy.logwarn("No 3D points matched.")
            return

        intensity = np.ones((matched_3d.shape[0], 1), dtype=np.float32)
        cloud_data = np.hstack((matched_3d, intensity))
        cloud_msg = pc2.create_cloud(self.header, self.fields, cloud_data)
        self.lane_3d_pub.publish(cloud_msg)
        rospy.loginfo(f"Published {cloud_data.shape[0]} 3D lane points.")


if __name__ == "__main__":
    LaneTo3D()
    rospy.spin()
