"""Publishes lidar_tc -> ego, the vehicle-longitudinal frame this stack outputs in.

A NEW leaf edge, never a second publisher of lidar_tc -> base_link. The repo does not own
/tf_static -- it comes from the vehicle driver stack or from bag replay -- and republishing
an edge someone else owns is a genuine TF conflict. base_link is left alone and left wrong.

Nothing existing looks up `ego`, so with this node down the edge simply does not exist and
every current consumer is unaffected.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from object_fusion.frames import EGO_YAW_IN_LIDAR_DEG, ego_tf_yaw_rad


class EgoFramePublisher(Node):
    def __init__(self):
        super().__init__("ego_frame_publisher")
        self._parent = self.declare_parameter("parent_frame", "lidar_tc").value
        self._child = self.declare_parameter("child_frame", "ego").value
        # Default is the measured value; 0.0 is the rollback to today's behaviour.
        yaw_deg = float(self.declare_parameter("ego_yaw_correction_deg",
                                               EGO_YAW_IN_LIDAR_DEG).value)
        # Deliberately NOT guessed. Whether the vehicle frame's origin belongs at road level
        # or at the rear axle is a vehicle-platform decision, not a perception one.
        z = float(self.declare_parameter("ego_z_offset", 0.0).value)

        self._bc = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._parent
        t.child_frame_id = self._child
        t.transform.translation.z = z
        half = 0.5 * ego_tf_yaw_rad(yaw_deg)
        t.transform.rotation.z = math.sin(half)
        t.transform.rotation.w = math.cos(half)
        self._bc.sendTransform(t)

        self.get_logger().info(
            f"{self._parent} -> {self._child}: yaw {yaw_deg:+.3f} deg (pose of {self._child} "
            f"in {self._parent}; rotating a POINT the other way uses {-yaw_deg:+.3f}), z={z:.3f}. "
            "base_link is NOT this frame."
        )


def main():
    rclpy.init()
    node = EgoFramePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
