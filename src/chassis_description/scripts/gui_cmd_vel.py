#!/usr/bin/env python3
# GUI 发布 /cmd_vel
# 点 Forward / Backward / Left / Right 后会持续发布速度
# 点 Stop 停止

import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class GuiCmdVel(Node):
    def __init__(self):
        super().__init__('gui_cmd_vel')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.speed = 0.2
        self.turn = 0.6

        self.timer = self.create_timer(0.05, self.publish_cmd)

        self.get_logger().info('gui_cmd_vel started')
        self.get_logger().info('Publish: /cmd_vel')

    def set_cmd(self, linear_x, angular_z):
        self.linear_x = float(linear_x)
        self.angular_z = float(angular_z)
        self.get_logger().info(
            f'cmd_vel: linear.x={self.linear_x:.2f}, angular.z={self.angular_z:.2f}'
        )

    def stop(self):
        self.set_cmd(0.0, 0.0)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GuiCmdVel()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    root.title('ROS2 cmd_vel GUI')

    status = tk.StringVar(value='Stop')

    def forward():
        node.set_cmd(node.speed, 0.0)
        status.set('Forward')

    def backward():
        node.set_cmd(-node.speed, 0.0)
        status.set('Backward')

    def left():
        node.set_cmd(0.0, node.turn)
        status.set('Left')

    def right():
        node.set_cmd(0.0, -node.turn)
        status.set('Right')

    def stop():
        node.stop()
        status.set('Stop')

    def forward_left():
        node.set_cmd(node.speed, node.turn)
        status.set('Forward Left')

    def forward_right():
        node.set_cmd(node.speed, -node.turn)
        status.set('Forward Right')

    def back_left():
        node.set_cmd(-node.speed, node.turn)
        status.set('Backward Left')

    def back_right():
        node.set_cmd(-node.speed, -node.turn)
        status.set('Backward Right')

    def update_speed(value):
        node.speed = float(value)

    def update_turn(value):
        node.turn = float(value)

    tk.Label(root, text='ROS2 /cmd_vel GUI', font=('Arial', 18, 'bold')).grid(
        row=0, column=0, columnspan=3, padx=10, pady=10
    )

    tk.Button(root, text='Forward Left', width=16, height=2, command=forward_left).grid(row=1, column=0, padx=5, pady=5)
    tk.Button(root, text='Forward', width=16, height=2, command=forward).grid(row=1, column=1, padx=5, pady=5)
    tk.Button(root, text='Forward Right', width=16, height=2, command=forward_right).grid(row=1, column=2, padx=5, pady=5)

    tk.Button(root, text='Left', width=16, height=2, command=left).grid(row=2, column=0, padx=5, pady=5)
    tk.Button(root, text='Stop', width=16, height=2, command=stop, bg='red', fg='white').grid(row=2, column=1, padx=5, pady=5)
    tk.Button(root, text='Right', width=16, height=2, command=right).grid(row=2, column=2, padx=5, pady=5)

    tk.Button(root, text='Back Left', width=16, height=2, command=back_left).grid(row=3, column=0, padx=5, pady=5)
    tk.Button(root, text='Backward', width=16, height=2, command=backward).grid(row=3, column=1, padx=5, pady=5)
    tk.Button(root, text='Back Right', width=16, height=2, command=back_right).grid(row=3, column=2, padx=5, pady=5)

    tk.Label(root, text='Linear speed').grid(row=4, column=0, sticky='e', padx=5)
    speed_scale = tk.Scale(root, from_=0.0, to=1.0, resolution=0.05, orient=tk.HORIZONTAL, command=update_speed)
    speed_scale.set(node.speed)
    speed_scale.grid(row=4, column=1, columnspan=2, sticky='we', padx=5)

    tk.Label(root, text='Angular speed').grid(row=5, column=0, sticky='e', padx=5)
    turn_scale = tk.Scale(root, from_=0.0, to=2.0, resolution=0.1, orient=tk.HORIZONTAL, command=update_turn)
    turn_scale.set(node.turn)
    turn_scale.grid(row=5, column=1, columnspan=2, sticky='we', padx=5)

    tk.Label(root, textvariable=status, font=('Arial', 14)).grid(row=6, column=0, columnspan=3, pady=10)

    def on_key(event):
        key = event.char
        if key == 'i':
            forward()
        elif key == ',':
            backward()
        elif key == 'j':
            left()
        elif key == 'l':
            right()
        elif key == 'u':
            forward_left()
        elif key == 'o':
            forward_right()
        elif key == 'm':
            back_left()
        elif key == '.':
            back_right()
        elif key == 'k':
            stop()

    root.bind('<KeyPress>', on_key)

    def on_close():
        node.stop()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)

    try:
        root.mainloop()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
