#!/usr/bin/env python3
import math
import random
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class LivoxLikeFilter(Node):
    def __init__(self):
        super().__init__("livox_like_filter")

        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/lidar_like")

        # 保留比例，越大点越多
        self.declare_parameter("keep_ratio", 0.70)

        # 单帧最多点数
        self.declare_parameter("max_points", 30000)

        # MID-360S 近似垂直 FOV
        self.declare_parameter("vertical_min_deg", -7.0)
        self.declare_parameter("vertical_max_deg", 52.0)

        # 是否加入轻微空间扰动，让 Gazebo 规则点云不那么机械
        self.declare_parameter("jitter_enable", True)
        self.declare_parameter("jitter_std", 0.0015)

        # 是否使用弱 Livox-like 扫描纹理
        self.declare_parameter("texture_enable", True)
        self.declare_parameter("texture_strength", 0.25)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.keep_ratio = float(self.get_parameter("keep_ratio").value)
        self.max_points = int(self.get_parameter("max_points").value)

        self.vmin = math.radians(float(self.get_parameter("vertical_min_deg").value))
        self.vmax = math.radians(float(self.get_parameter("vertical_max_deg").value))

        self.jitter_enable = bool(self.get_parameter("jitter_enable").value)
        self.jitter_std = float(self.get_parameter("jitter_std").value)

        self.texture_enable = bool(self.get_parameter("texture_enable").value)
        self.texture_strength = float(self.get_parameter("texture_strength").value)

        self.frame_count = 0

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10
        )

        self.get_logger().info("livox_like_filter 360-stable version started")
        self.get_logger().info(f"Subscribe: {self.input_topic}")
        self.get_logger().info(f"Publish:   {self.output_topic}")
        self.get_logger().info("Output fields: x, y, z, intensity")

    def get_field_offsets(self, msg):
        offsets = {}
        for f in msg.fields:
            offsets[f.name] = f.offset
        return offsets

    def callback(self, msg):
        offsets = self.get_field_offsets(msg)

        if "x" not in offsets or "y" not in offsets or "z" not in offsets:
            self.get_logger().warn("PointCloud2 has no x/y/z fields")
            return

        ox = offsets["x"]
        oy = offsets["y"]
        oz = offsets["z"]

        total = msg.width * msg.height
        if total <= 0:
            return

        self.frame_count += 1
        phase = self.frame_count * 0.173

        points = []

        # 不要每帧都从同一个 index 开始，避免固定纹理
        start_index = random.randint(0, total - 1)

        for n in range(total):
            i = (start_index + n) % total
            base = i * msg.point_step

            try:
                x = struct.unpack_from("f", msg.data, base + ox)[0]
                y = struct.unpack_from("f", msg.data, base + oy)[0]
                z = struct.unpack_from("f", msg.data, base + oz)[0]
            except Exception:
                continue

            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue

            r_xy = math.sqrt(x * x + y * y)
            r = math.sqrt(x * x + y * y + z * z)

            if r < 0.1:
                continue

            elev = math.atan2(z, r_xy)

            # 保留 MID-360S 近似垂直范围
            if elev < self.vmin or elev > self.vmax:
                continue

            az = math.atan2(y, x)

            az_norm = (az + math.pi) / (2.0 * math.pi)
            el_norm = (elev - self.vmin) / (self.vmax - self.vmin)

            # ===== 关键：360° 稳定保留 =====
            # 不做强花瓣筛选，避免某个方向完全没点。
            keep_prob = self.keep_ratio

            if self.texture_enable:
                # 弱纹理，只改变密度，不切断方向
                texture = math.sin(17.0 * az_norm + 9.0 * el_norm + phase)
                texture += 0.5 * math.sin(31.0 * az_norm - 5.0 * el_norm + phase * 0.7)
                texture = 0.5 + 0.5 * math.tanh(texture)

                # texture_strength 越小越均匀
                keep_prob = self.keep_ratio * (
                    1.0 - self.texture_strength + self.texture_strength * texture
                )

            keep_prob = max(0.05, min(1.0, keep_prob))

            if random.random() > keep_prob:
                continue

            # ===== intensity =====
            # 距离近一点更亮，远一点更暗
            intensity = 255.0 * (1.0 - min(r, 70.0) / 70.0)

            # 加一点角度纹理，让显示更像雷达强度而不是纯随机
            intensity += 25.0 * (
                0.5 + 0.5 * math.sin(8.0 * az + 3.0 * elev + phase)
            )

            # 加一点随机反射强度扰动
            intensity += random.uniform(-10.0, 10.0)
            intensity = max(0.0, min(255.0, intensity))

            # 轻微扰动，让点云不像 Gazebo 规则网格
            if self.jitter_enable:
                x += random.gauss(0.0, self.jitter_std)
                y += random.gauss(0.0, self.jitter_std)
                z += random.gauss(0.0, self.jitter_std * 0.5)

            points.append((x, y, z, intensity))

            if len(points) >= self.max_points:
                break

        out = self.make_cloud(msg.header, points)
        self.pub.publish(out)

    def make_cloud(self, header, points):
        out = PointCloud2()
        out.header = Header()
        out.header.stamp = header.stamp
        out.header.frame_id = header.frame_id

        out.height = 1
        out.width = len(points)
        out.is_bigendian = False
        out.is_dense = False

        # x, y, z, intensity = 4 float32 = 16 bytes
        out.point_step = 16
        out.row_step = out.point_step * out.width

        out.fields = [
            PointField(
                name="x",
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name="y",
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name="z",
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name="intensity",
                offset=12,
                datatype=PointField.FLOAT32,
                count=1
            ),
        ]

        buff = bytearray()
        for x, y, z, intensity in points:
            buff.extend(struct.pack("ffff", x, y, z, intensity))

        out.data = bytes(buff)
        return out


def main():
    rclpy.init()
    node = LivoxLikeFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
