#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):
    def __init__(self):
        super().__init__("odom_to_path")

        self.path = Path()
        self.path.header.frame_id = "odom"

        self.max_len = 2000

        self.sub_odom = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.pub_path = self.create_publisher(
            Path,
            "/path",
            10
        )

        self.get_logger().info("odom_to_path started")
        self.get_logger().info("Subscribe: /odom")
        self.get_logger().info("Publish: /path")

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # 确保 Path 坐标系是 odom
        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = "odom"

        self.path.poses.append(pose)

        if len(self.path.poses) > self.max_len:
            self.path.poses = self.path.poses[-self.max_len:]

        self.pub_path.publish(self.path)


def main():
    rclpy.init()
    node = OdomToPath()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
