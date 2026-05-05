#!/usr/bin/env python3
import math
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState

from tf2_ros import TransformBroadcaster


HELP = """
RViz2 keyboard fake drive
-------------------------
i : forward
, : backward
j : turn left
l : turn right
k : stop

q/z : increase/decrease both speed and turn
w/x : increase/decrease linear speed
e/c : increase/decrease angular speed

CTRL-C to quit
"""


def get_key(timeout=0.1):
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key


def yaw_to_quat(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return 0.0, 0.0, qz, qw


class Rviz2KeyboardDrive(Node):
    def __init__(self):
        super().__init__("rviz2_keyboard_drive")

        self.tf_broadcaster = TransformBroadcaster(self)

        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_joint = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_path = self.create_publisher(Path, "/path", 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.v = 0.0
        self.w = 0.0

        self.speed = 0.4
        self.turn = 0.8

        self.wheel_angle = 0.0
        self.wheel_radius = 0.105

        # RViz2 专用 URDF 里面是不带 _joint 的名字
        self.wheel_joint_names = [
            "front_right_wheel",
            "front_left_wheel",
            "rare_left_wheel",
            "rare_right_wheel",
        ]

        self.path = Path()
        self.path.header.frame_id = "odom"

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.update)

        self.get_logger().info("rviz2_keyboard_drive started")
        self.get_logger().info("Publish: /tf odom->base_footprint, /odom, /joint_states, /path, /cmd_vel")
        print(HELP)

    def handle_key(self, key):
        if key == "i":
            self.v = self.speed
            self.w = 0.0
        elif key == ",":
            self.v = -self.speed
            self.w = 0.0
        elif key == "j":
            self.v = 0.0
            self.w = self.turn
        elif key == "l":
            self.v = 0.0
            self.w = -self.turn
        elif key == "k" or key == " ":
            self.v = 0.0
            self.w = 0.0
        elif key == "q":
            self.speed *= 1.1
            self.turn *= 1.1
        elif key == "z":
            self.speed *= 0.9
            self.turn *= 0.9
        elif key == "w":
            self.speed *= 1.1
        elif key == "x":
            self.speed *= 0.9
        elif key == "e":
            self.turn *= 1.1
        elif key == "c":
            self.turn *= 0.9

        print(f"speed={self.speed:.2f}, turn={self.turn:.2f}, cmd=({self.v:.2f}, {self.w:.2f})")

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 0.2:
            dt = 0.02

        # 更新位姿
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        # 更新轮子角度
        self.wheel_angle += (self.v / self.wheel_radius) * dt

        qx, qy, qz, qw = yaw_to_quat(self.yaw)

        # TF: odom -> base_footprint
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        # Odom
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.pub_odom.publish(odom)

        # Joint states
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = self.wheel_joint_names
        js.position = [
            self.wheel_angle,
            self.wheel_angle,
            self.wheel_angle,
            self.wheel_angle,
        ]
        self.pub_joint.publish(js)

        # Path
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = "odom"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.path.header.stamp = now.to_msg()
        self.path.poses.append(pose)

        if len(self.path.poses) > 1000:
            self.path.poses = self.path.poses[-1000:]

        self.pub_path.publish(self.path)

        # 同时发布 cmd_vel，方便调试
        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = self.w
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = Rviz2KeyboardDrive()

    try:
        while rclpy.ok():
            key = get_key(0.05)
            if key == "\x03":
                break
            if key:
                node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass

    node.v = 0.0
    node.w = 0.0
    node.update()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
