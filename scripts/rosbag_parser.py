from __future__ import annotations

import argparse
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_BAG = "/home/right/wheel_parser/data/20260407_112344"
DEFAULT_OUTPUT_DIR = str(ROOT_DIR / "Log/preprocess")
DEFAULT_IMU_TOPIC = "/livox/imu"
DEFAULT_WHEEL_TOPIC = "/serial/wheels"


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", default=DEFAULT_BAG)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    parser.add_argument("--wheel-topic", default=DEFAULT_WHEEL_TOPIC)
    return parser.parse_args()


def export_topics(bag_path, imu_topic, wheel_topic, output_dir):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}

    imu_type = get_message(topic_types[imu_topic])
    wheel_type = get_message(topic_types[wheel_topic])

    imu_output_path = output_dir / "imu_raw.txt"
    wheel_output_path = output_dir / "wheel_raw.txt"
    imu_output_path.parent.mkdir(parents=True, exist_ok=True)
    wheel_output_path.parent.mkdir(parents=True, exist_ok=True)

    imu_count = 0
    wheel_count = 0

    with (
        imu_output_path.open("w", encoding="utf-8") as imu_file,
        wheel_output_path.open("w", encoding="utf-8") as wheel_file,
    ):
        while reader.has_next():
            topic, raw_data, _ = reader.read_next()

            if topic == imu_topic:
                msg = deserialize_message(raw_data, imu_type)
                timestamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                imu_file.write(
                    f"{timestamp_s:.6f} "
                    f"{msg.angular_velocity.x:.6f} {msg.angular_velocity.y:.6f} {msg.angular_velocity.z:.6f} "
                    f"{msg.linear_acceleration.x:.6f} {msg.linear_acceleration.y:.6f} {msg.linear_acceleration.z:.6f}\n"
                )
                imu_count += 1

            elif topic == wheel_topic:
                msg = deserialize_message(raw_data, wheel_type)
                timestamp_us = float(msg.header.stamp.sec) * 1e6 + float(msg.header.stamp.nanosec)
                wheel_file.write(
                    f"{timestamp_us:.6f} "
                    f"{msg.linear_acceleration.x:.6f} {msg.linear_acceleration.y:.6f}\n"
                )
                wheel_count += 1

    return imu_output_path, wheel_output_path, imu_count, wheel_count


def main():
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()

    imu_output_path, wheel_output_path, imu_count, wheel_count = export_topics(
        bag_path,
        args.imu_topic,
        args.wheel_topic,
        output_dir,
    )

    print(f"imu_output={imu_output_path} rows={imu_count}")
    print(f"wheel_output={wheel_output_path} rows={wheel_count}")


if __name__ == "__main__":
    main()
