#!/usr/bin/env python3
"""Validate the S1 rosbag2 sensor contract and semantic digest."""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_msgs.msg import TFMessage

from bag_semantics import update_digest

MESSAGE_TYPES = {
    "/clock": Clock,
    "/imu/data": Imu,
    "/points_raw": PointCloud2,
    "/ground_truth": Odometry,
    "/tf_static": TFMessage,
}
EXPECTED_FIELDS = [
    ("x", 0, PointField.FLOAT32),
    ("y", 4, PointField.FLOAT32),
    ("z", 8, PointField.FLOAT32),
    ("intensity", 12, PointField.FLOAT32),
    ("ring", 16, PointField.UINT16),
    ("time", 20, PointField.FLOAT32),
]
POINT_DTYPE = np.dtype(
    {
        "names": ["x", "y", "z", "intensity", "ring", "time"],
        "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
        "offsets": [0, 4, 8, 12, 16, 20],
        "itemsize": 24,
    }
)


def header_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def check(condition, description):
    if not condition:
        raise AssertionError(description)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.bag / "manifest.json").read_text(encoding="utf-8"))
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topics = reader.get_all_topics_and_types()
    metadata = {topic.name: topic.type for topic in topics}
    check(metadata == {name: f"{cls.__module__.split('.')[0]}/msg/{cls.__name__}" for name, cls in MESSAGE_TYPES.items()}, "topic types")
    check(len(next(topic for topic in topics if topic.name == "/tf_static").offered_qos_profiles) == 1, "static TF QoS profile")
    counts = {name: 0 for name in MESSAGE_TYPES}
    previous = {}
    digest = hashlib.sha256()
    truth = []
    stationary_imu = []
    cloud_widths = []
    static_frames = None

    while reader.has_next():
        name, payload, timestamp_ns = reader.read_next()
        check(name in MESSAGE_TYPES, f"unexpected topic {name}")
        check(timestamp_ns >= previous.get(name, -1), f"timestamp regressed on {name}")
        previous[name] = timestamp_ns
        counts[name] += 1
        msg = deserialize_message(payload, MESSAGE_TYPES[name])
        update_digest(digest, name, timestamp_ns, msg)
        if name == "/clock":
            check(msg.clock.sec * 1_000_000_000 + msg.clock.nanosec == timestamp_ns, "clock stamp")
        elif name == "/tf_static":
            static_frames = {(tf.header.frame_id, tf.child_frame_id) for tf in msg.transforms}
            check(len(msg.transforms) == 2, "static transform count")
        else:
            check(header_ns(msg.header) == timestamp_ns, f"header stamp on {name}")
            if name == "/imu/data":
                check(msg.header.frame_id == "imu_link", "IMU frame")
                check(math.isfinite(msg.angular_velocity.z), "IMU angular velocity")
                if timestamp_ns >= manifest["start_time_ns"] + 12_000_000_000:
                    stationary_imu.append((msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
            elif name == "/ground_truth":
                check(msg.header.frame_id == "map" and msg.child_frame_id == "base_link", "truth frames")
                truth.append((timestamp_ns, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.twist.twist.linear.x))
            elif name == "/points_raw":
                check(msg.header.frame_id == "velodyne", "LiDAR frame")
                check(msg.height == 1 and msg.width > 10_000, "point count")
                check(msg.point_step == 24 and msg.row_step == 24 * msg.width, "cloud layout")
                check(len(msg.data) == msg.row_step, "cloud data length")
                check(not msg.is_bigendian and msg.is_dense, "cloud byte order/density")
                check([(field.name, field.offset, field.datatype) for field in msg.fields] == EXPECTED_FIELDS, "cloud fields")
                points = np.frombuffer(msg.data, dtype=POINT_DTYPE)
                check(np.all(np.isfinite(points["x"])) and np.all(np.isfinite(points["y"])) and np.all(np.isfinite(points["z"])), "finite point coordinates")
                check(np.all(points["ring"] < 16), "ring range")
                check(np.all((points["time"] >= 0.0) & (points["time"] < 0.1)), "point relative time")
                check(np.all(np.diff(points["time"]) >= 0.0), "point order")
                cloud_widths.append(msg.width)

    duration = manifest["duration_s"]
    check(counts == manifest["counts"], "message counts")
    check(counts["/clock"] == round(duration * 200), "clock rate")
    check(counts["/imu/data"] == round(duration * 200), "IMU rate")
    check(counts["/points_raw"] == round(duration * 10), "LiDAR rate")
    check(counts["/ground_truth"] == round(duration * 10), "truth rate")
    check(static_frames == {("base_link", "velodyne"), ("base_link", "imu_link")}, "TF frames")
    check(digest.hexdigest() == manifest["semantic_sha256"], "semantic digest")
    check(len(stationary_imu) >= 200, "stationary IMU samples")
    mean_accel = np.mean(stationary_imu, axis=0)
    check(abs(mean_accel[0]) < 0.05 and abs(mean_accel[1]) < 0.05 and abs(mean_accel[2] - 9.81) < 0.05, "stationary specific force")
    check(truth[49][1] > 1.0 and abs(truth[49][3]) < 1e-8, "initial straight motion")
    check(truth[100][3] > 0.3, "turn occurred")
    check(all(abs(item[4]) < 1e-10 for item in truth[-20:]), "final stop")
    print(json.dumps({"status": "OK", "bag": str(args.bag), "counts": counts, "cloud_points_min": min(cloud_widths), "cloud_points_max": max(cloud_widths), "stationary_accel_mean": mean_accel.tolist(), "semantic_sha256": digest.hexdigest()}))


if __name__ == "__main__":
    main()
