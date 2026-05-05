#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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


class GazeboGuiDriveNode(Node):
    def __init__(self):
        super().__init__("gazebo_gui_drive")

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self.speed = 0.4
        self.turn = 0.8

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.timer = self.create_timer(0.05, self.publish_cmd)

        self.get_logger().info("gazebo_gui_drive started")
        self.get_logger().info("Publish: /cmd_vel")

    def set_cmd(self, linear_x, angular_z):
        self.linear_x = float(linear_x)
        self.angular_z = float(angular_z)

    def set_speed(self, speed):
        self.speed = float(speed)

    def set_turn(self, turn):
        self.turn = float(turn)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_z
        self.pub_cmd.publish(msg)


class GazeboGuiDriveWindow(QWidget):
    def __init__(self, node):
        super().__init__()

        self.node = node

        self.setWindowTitle("Gazebo GUI Drive")
        self.resize(360, 260)

        self.title_label = QLabel("Gazebo 差速车 GUI 控制")
        self.status_label = QLabel("cmd: linear.x=0.00, angular.z=0.00")

        self.speed_box = QDoubleSpinBox()
        self.speed_box.setRange(0.0, 3.0)
        self.speed_box.setSingleStep(0.1)
        self.speed_box.setValue(self.node.speed)
        self.speed_box.setSuffix(" m/s")
        self.speed_box.valueChanged.connect(lambda value: self.node.set_speed(value))

        self.turn_box = QDoubleSpinBox()
        self.turn_box.setRange(0.0, 5.0)
        self.turn_box.setSingleStep(0.1)
        self.turn_box.setValue(self.node.turn)
        self.turn_box.setSuffix(" rad/s")
        self.turn_box.valueChanged.connect(lambda value: self.node.set_turn(value))

        self.btn_forward = QPushButton("前进 i")
        self.btn_backward = QPushButton("后退 m / s")
        self.btn_left = QPushButton("左转 j")
        self.btn_right = QPushButton("右转 l")
        self.btn_stop = QPushButton("停止 k / Space")

        # 用 lambda，避免 left/right 和 Qt 内置函数冲突
        self.btn_forward.clicked.connect(lambda: self.cmd_forward())
        self.btn_backward.clicked.connect(lambda: self.cmd_backward())
        self.btn_left.clicked.connect(lambda: self.cmd_left())
        self.btn_right.clicked.connect(lambda: self.cmd_right())
        self.btn_stop.clicked.connect(lambda: self.cmd_stop())

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
        layout.addWidget(self.title_label)
        layout.addLayout(speed_layout)
        layout.addLayout(turn_layout)
        layout.addLayout(grid)
        layout.addWidget(self.status_label)

        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.loop)
        self.timer.start(20)

    def cmd_forward(self):
        self.node.set_cmd(-self.node.speed, 0.0)

    def cmd_backward(self):
        self.node.set_cmd(self.node.speed, 0.0)

    def cmd_left(self):
        self.node.set_cmd(0.0, -self.node.turn)

    def cmd_right(self):
        self.node.set_cmd(0.0, self.node.turn)

    def cmd_stop(self):
        self.node.set_cmd(0.0, 0.0)

    def keyPressEvent(self, event):
        key = event.text()

        if key == "i":
            self.cmd_forward()
        elif key == "," or key == "m" or key == "s":
            self.cmd_backward()
        elif key == "j":
            self.cmd_left()
        elif key == "l":
            self.cmd_right()
        elif key == "k" or key == " ":
            self.cmd_stop()

    def loop(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

        self.status_label.setText(
            f"cmd: linear.x={self.node.linear_x:.2f}, "
            f"angular.z={self.node.angular_z:.2f}"
        )


def main():
    rclpy.init()

    node = GazeboGuiDriveNode()

    app = QApplication(sys.argv)
    window = GazeboGuiDriveWindow(node)
    window.show()

    try:
        app.exec_()
    finally:
        node.set_cmd(0.0, 0.0)
        node.publish_cmd()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
