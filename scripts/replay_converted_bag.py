#!/usr/bin/env python3

import argparse
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu

from liwins.msg import Wheel
from livox_ros_driver2.msg import CustomMsg


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", default="/home/right/wheel_parser/data/20260413_170850")
    parser.add_argument("--imu-topic", default="/livox/imu")
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--wheel-topic", default="/serial/wheels")
    parser.add_argument("--imu-output-topic", default="/livox/imu")
    parser.add_argument("--lidar-output-topic", default="/livox/lidar")
    parser.add_argument("--wheel-output-topic", default="/wheel")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument(
        "--wheel-timestamp-source",
        choices=("aligned_device_us", "bag", "raw_device_us"),
        default="aligned_device_us",
    )
    return parser.parse_args()


def sensor_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=200,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class ConvertedBagReplayer(Node):
    def __init__(self, args):
        super().__init__("fins_converted_bag_replayer")
        self.args = args
        self.reader = SequentialReader()
        self.reader.open(
            StorageOptions(uri=str(Path(args.bag).expanduser().resolve()), storage_id="sqlite3"),
            ConverterOptions("", ""),
        )

        topic_types = {topic.name: topic.type for topic in self.reader.get_all_topics_and_types()}
        self.imu_type = get_message(topic_types[args.imu_topic])
        self.wheel_type = get_message(topic_types[args.wheel_topic])
        self.lidar_type = get_message(topic_types[args.lidar_topic])
        self.wheel_time_anchor = self._detect_wheel_time_anchor() if args.wheel_timestamp_source == "aligned_device_us" else None

        qos = sensor_qos()
        self.imu_pub = self.create_publisher(Imu, args.imu_output_topic, qos)
        self.lidar_pub = self.create_publisher(CustomMsg, args.lidar_output_topic, qos)
        self.wheel_pub = self.create_publisher(Wheel, args.wheel_output_topic, qos)

        self.get_logger().info(
            "replay bag=%s imu=%s->%s lidar=%s->%s wheel=%s->%s timestamp=%s rate=%.3f"
            % (
                args.bag,
                args.imu_topic,
                args.imu_output_topic,
                args.lidar_topic,
                args.lidar_output_topic,
                args.wheel_topic,
                args.wheel_output_topic,
                args.wheel_timestamp_source,
                args.rate,
            )
        )
        if self.wheel_time_anchor is not None:
            self.get_logger().info(
                "wheel time anchor device=%.6f bag=%.6f"
                % (self.wheel_time_anchor[0], self.wheel_time_anchor[1])
            )

    def _wheel_device_time_seconds(self, msg):
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-6

    def _detect_wheel_time_anchor(self):
        anchor_reader = SequentialReader()
        anchor_reader.open(
            StorageOptions(uri=str(Path(self.args.bag).expanduser().resolve()), storage_id="sqlite3"),
            ConverterOptions("", ""),
        )

        device_times = []
        bag_times = []
        while anchor_reader.has_next():
            topic, raw_data, bag_time_ns = anchor_reader.read_next()
            if topic != self.args.wheel_topic:
                continue
            msg = deserialize_message(raw_data, self.wheel_type)
            device_times.append(self._wheel_device_time_seconds(msg))
            bag_times.append(float(bag_time_ns) * 1e-9)

        diffs = [
            device_times[index + 1] - device_times[index]
            for index in range(len(device_times) - 1)
            if device_times[index + 1] > device_times[index]
        ]
        anchor_index = 0
        if diffs:
            diffs.sort()
            median_diff = diffs[len(diffs) // 2]
            for index in range(len(device_times) - 1):
                delta = device_times[index + 1] - device_times[index]
                if delta > 100.0 * median_diff:
                    anchor_index = index + 1
                    break

        return device_times[anchor_index], bag_times[anchor_index]

    def _wheel_timestamp_seconds(self, msg, bag_time_ns):
        if self.args.wheel_timestamp_source == "raw_device_us":
            return self._wheel_device_time_seconds(msg)
        if self.args.wheel_timestamp_source == "aligned_device_us":
            device_anchor, bag_anchor = self.wheel_time_anchor
            return bag_anchor + (self._wheel_device_time_seconds(msg) - device_anchor)
        return float(bag_time_ns) * 1e-9

    def replay(self):
        first_bag_time_ns = None
        wall_start = time.monotonic()
        imu_count = 0
        lidar_count = 0
        wheel_count = 0

        while rclpy.ok() and self.reader.has_next():
            topic, raw_data, bag_time_ns = self.reader.read_next()

            if topic == self.args.imu_topic:
                msg = deserialize_message(raw_data, self.imu_type)
                imu_count += 1
            elif topic == self.args.lidar_topic:
                msg = deserialize_message(raw_data, self.lidar_type)
                lidar_count += 1
            elif topic == self.args.wheel_topic:
                msg = deserialize_message(raw_data, self.wheel_type)
                wheel_count += 1
            else:
                continue

            if first_bag_time_ns is None:
                first_bag_time_ns = bag_time_ns

            elapsed_bag = (bag_time_ns - first_bag_time_ns) * 1e-9 / self.args.rate
            elapsed_wall = time.monotonic() - wall_start
            sleep_time = elapsed_bag - elapsed_wall
            if sleep_time > 0.0:
                time.sleep(sleep_time)

            if topic == self.args.imu_topic:
                self.imu_pub.publish(msg)
            elif topic == self.args.lidar_topic:
                self.lidar_pub.publish(msg)
            else:
                wheel_msg = Wheel()
                wheel_msg.timestamp = self._wheel_timestamp_seconds(msg, bag_time_ns)
                wheel_msg.encoder1 = float(msg.linear_acceleration.x)
                wheel_msg.encoder2 = float(msg.linear_acceleration.y)
                self.wheel_pub.publish(wheel_msg)

            rclpy.spin_once(self, timeout_sec=0.0)

        self.get_logger().info(
            "replay finished imu=%d lidar=%d wheel=%d"
            % (imu_count, lidar_count, wheel_count)
        )


def main():
    args = parse_args()
    rclpy.init()
    node = ConvertedBagReplayer(args)
    node.replay()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
