#!/usr/bin/env python3
import math
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QPushButton,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QDoubleSpinBox,
)
from PyQt5.QtCore import QTimer


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class Rviz2GuiDriveNode(Node):
    def __init__(self):
        super().__init__("rviz2_gui_drive")

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

        # RViz2 专用 URDF：joint 名字不带 _joint
        self.wheel_joint_names = [
            "front_right_wheel",
            "front_left_wheel",
            "rare_left_wheel",
            "rare_right_wheel",
        ]

        self.path = Path()
        self.path.header.frame_id = "odom"
        self.max_path_len = 2000

        self.last_time = self.get_clock().now()

        self.get_logger().info("rviz2_gui_drive started")
        self.get_logger().info("Publish: /tf, /odom, /joint_states, /path, /cmd_vel")

    def set_cmd(self, v, w):
        self.v = v
        self.w = w

    def set_speed(self, speed):
        self.speed = speed

    def set_turn(self, turn):
        self.turn = turn

    def reset_pose(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.wheel_angle = 0.0
        self.path.poses.clear()
        self.v = 0.0
        self.w = 0.0

    def update_state(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 0.2:
            dt = 0.02

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

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
        self.path.header.frame_id = "odom"
        self.path.poses.append(pose)

        if len(self.path.poses) > self.max_path_len:
            self.path.poses = self.path.poses[-self.max_path_len:]

        self.pub_path.publish(self.path)

        # Cmd vel
        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = self.w
        self.pub_cmd.publish(cmd)


class Rviz2GuiDriveWindow(QWidget):
    def __init__(self, node):
        super().__init__()

        self.node = node

        self.setWindowTitle("RViz2 GUI Drive")
        self.resize(360, 300)

        self.status_label = QLabel("cmd: v=0.00 m/s, w=0.00 rad/s")
        self.pose_label = QLabel("pose: x=0.00, y=0.00, yaw=0.00 deg")

        self.speed_box = QDoubleSpinBox()
        self.speed_box.setRange(0.0, 3.0)
        self.speed_box.setSingleStep(0.1)
        self.speed_box.setValue(self.node.speed)
        self.speed_box.setSuffix(" m/s")
        self.speed_box.valueChanged.connect(self.on_speed_changed)

        self.turn_box = QDoubleSpinBox()
        self.turn_box.setRange(0.0, 5.0)
        self.turn_box.setSingleStep(0.1)
        self.turn_box.setValue(self.node.turn)
        self.turn_box.setSuffix(" rad/s")
        self.turn_box.valueChanged.connect(self.on_turn_changed)

        self.btn_forward = QPushButton("前进 i")
        self.btn_backward = QPushButton("后退 ,")
        self.btn_left = QPushButton("左转 j")
        self.btn_right = QPushButton("右转 l")
        self.btn_stop = QPushButton("停止 k")
        self.btn_reset = QPushButton("重置位姿")

        self.btn_forward.clicked.connect(self.forward)
        self.btn_backward.clicked.connect(self.backward)
        self.btn_left.clicked.connect(self.left)
        self.btn_right.clicked.connect(self.right)
        self.btn_stop.clicked.connect(self.stop)
        self.btn_reset.clicked.connect(self.reset_pose)

        grid = QGridLayout()
        grid.addWidget(self.btn_forward, 0, 1)
        grid.addWidget(self.btn_left, 1, 0)
        grid.addWidget(self.btn_stop, 1, 1)
        grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_backward, 2, 1)

        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("线速度"))
        speed_layout.addWidget(self.speed_box)

        turn_layout = QHBoxLayout()
        turn_layout.addWidget(QLabel("角速度"))
        turn_layout.addWidget(self.turn_box)

        layout = QVBoxLayout()
        layout.addWidget(QLabel("RViz2 假运动 GUI 控制"))
        layout.addLayout(speed_layout)
        layout.addLayout(turn_layout)
        layout.addLayout(grid)
        layout.addWidget(self.btn_reset)
        layout.addWidget(self.status_label)
        layout.addWidget(self.pose_label)

        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.loop)
        self.timer.start(20)

    def on_speed_changed(self, value):
        self.node.set_speed(value)

    def on_turn_changed(self, value):
        self.node.set_turn(value)

    def forward(self):
        self.node.set_cmd(self.node.speed, 0.0)

    def backward(self):
        self.node.set_cmd(-self.node.speed, 0.0)

    def left(self):
        self.node.set_cmd(0.0, self.node.turn)

    def right(self):
        self.node.set_cmd(0.0, -self.node.turn)

    def stop(self):
        self.node.set_cmd(0.0, 0.0)

    def reset_pose(self):
        self.node.reset_pose()

    def keyPressEvent(self, event):
        key = event.text()

        if key == "i":
            self.forward()
        elif key == ",":
            self.backward()
        elif key == "j":
            self.left()
        elif key == "l":
            self.right()
        elif key == "k" or key == " ":
            self.stop()

    def loop(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.node.update_state()

        self.status_label.setText(
            f"cmd: v={self.node.v:.2f} m/s, w={self.node.w:.2f} rad/s"
        )
        self.pose_label.setText(
            f"pose: x={self.node.x:.2f}, y={self.node.y:.2f}, "
            f"yaw={math.degrees(self.node.yaw):.1f} deg"
        )


def main():
    rclpy.init()

    node = Rviz2GuiDriveNode()

    app = QApplication(sys.argv)
    window = Rviz2GuiDriveWindow(node)
    window.show()

    try:
        app.exec_()
    finally:
        node.set_cmd(0.0, 0.0)
        node.update_state()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
