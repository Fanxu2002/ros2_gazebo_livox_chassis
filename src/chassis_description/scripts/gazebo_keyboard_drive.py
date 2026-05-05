#!/usr/bin/env python3
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
Gazebo keyboard drive
---------------------
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


class GazeboKeyboardDrive(Node):
    def __init__(self):
        super().__init__("gazebo_keyboard_drive")

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self.speed = 0.4
        self.turn = 0.8

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.timer = self.create_timer(0.05, self.publish_cmd)

        self.get_logger().info("gazebo_keyboard_drive started")
        self.get_logger().info("Publish: /cmd_vel")
        print(HELP)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.pub_cmd.publish(msg)

    def handle_key(self, key):
        if key == "i":
            self.linear_x = -self.speed
            self.angular_z = 0.0
        elif key == ",":
            self.linear_x = self.speed
            self.angular_z = 0.0
        elif key == "j":
            self.linear_x = 0.0
            self.angular_z = -self.turn
        elif key == "l":
            self.linear_x = 0.0
            self.angular_z = self.turn
        elif key == "k" or key == " ":
            self.linear_x = 0.0
            self.angular_z = 0.0
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

        print(
            f"speed={self.speed:.2f}, turn={self.turn:.2f}, cmd=({self.linear_x:.2f}, {self.angular_z:.2f})"
        )


def main():
    rclpy.init()
    node = GazeboKeyboardDrive()

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

    stop = Twist()
    node.pub_cmd.publish(stop)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
