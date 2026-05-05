#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def quat_to_yaw(q):
    # quaternion -> yaw
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GazeboJointStateFix(Node):
    def __init__(self):
        super().__init__("gazebo_joint_state_fix")

        self.pub_joint = self.create_publisher(JointState, "/joint_states", 10)

        self.sub_odom = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.sub_cmd = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_callback,
            10
        )

        # Gazebo URDF 的 wheel joint 名字，带 _joint
        self.wheel_joint_names = [
            "front_right_wheel_joint",
            "front_left_wheel_joint",
            "rare_left_wheel_joint",
            "rare_right_wheel_joint",
        ]

        # 要和 gazebo_chassis_diff.urdf 里保持一致
        self.wheel_radius = 0.105
        self.wheel_separation = 0.55

        # 如果 RViz 轮子视觉方向反了，就改这里
        # 顺序：
        # front_right, front_left, rare_left, rare_right
        self.sign_front_right = -1.0
        self.sign_front_left = 1.0
        self.sign_rear_left = 1.0
        self.sign_rear_right = -1.0

        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.last_stamp_sec = None

        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.left_angle = 0.0
        self.right_angle = 0.0

        self.last_publish_stamp = None

        self.get_logger().info("gazebo_joint_state_fix started")
        self.get_logger().info("Subscribe: /odom, /cmd_vel")
        self.get_logger().info("Publish: /joint_states with smoothed differential wheel motion")

    def cmd_callback(self, msg):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def odom_callback(self, msg):
        stamp = msg.header.stamp
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)

        if self.last_x is None:
            self.last_x = x
            self.last_y = y
            self.last_yaw = yaw
            self.last_stamp_sec = stamp_sec
            self.publish_joint(stamp)
            return

        dt = stamp_sec - self.last_stamp_sec
        if dt <= 0.0 or dt > 0.2:
            dt = 0.02

        dx = x - self.last_x
        dy = y - self.last_y

        # 用车头方向判断前进还是后退
        forward_dir_x = math.cos(self.last_yaw)
        forward_dir_y = math.sin(self.last_yaw)
        signed_ds = dx * forward_dir_x + dy * forward_dir_y

        # yaw 变化
        dyaw = yaw - self.last_yaw
        while dyaw > math.pi:
            dyaw -= 2.0 * math.pi
        while dyaw < -math.pi:
            dyaw += 2.0 * math.pi

        # 用 odom 算出的真实线速度 / 角速度
        odom_v = signed_ds / dt
        odom_w = dyaw / dt

        # 如果 cmd_vel 很新，则角速度主要用 cmd_w，减少 odom 抖动
        cmd_age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if cmd_age < 0.5:
            use_v = odom_v
            use_w = self.cmd_w
        else:
            use_v = odom_v
            use_w = odom_w

        # 小量过滤，避免停下时轮子微抖
        if abs(use_v) < 0.005:
            use_v = 0.0
        if abs(use_w) < 0.005:
            use_w = 0.0

        # 差速模型：
        # 左轮线速度 = v - w * L / 2
        # 右轮线速度 = v + w * L / 2
        v_left = use_v - use_w * self.wheel_separation * 0.5
        v_right = use_v + use_w * self.wheel_separation * 0.5

        self.left_angle += (v_left / self.wheel_radius) * dt
        self.right_angle += (v_right / self.wheel_radius) * dt

        self.last_x = x
        self.last_y = y
        self.last_yaw = yaw
        self.last_stamp_sec = stamp_sec

        self.publish_joint(stamp)

    def publish_joint(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.wheel_joint_names

        msg.position = [
            self.sign_front_right * self.right_angle,
            self.sign_front_left * self.left_angle,
            self.sign_rear_left * self.left_angle,
            self.sign_rear_right * self.right_angle,
        ]

        self.pub_joint.publish(msg)


def main():
    rclpy.init()
    node = GazeboJointStateFix()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
