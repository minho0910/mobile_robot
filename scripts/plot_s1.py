#!/usr/bin/env python3
"""Plot LiDAR returns and ground-truth motion from an S1 rosbag2 directory."""

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

from validate_s1 import POINT_DTYPE


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def load(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    poses = []
    clouds = []
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic == "/ground_truth":
            msg = deserialize_message(payload, Odometry)
            q = msg.pose.pose.orientation
            poses.append(
                (
                    stamp_ns(msg.header.stamp),
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    2.0 * math.atan2(q.z, q.w),
                    msg.twist.twist.linear.x,
                )
            )
        elif topic == "/points_raw" and len(clouds) in (0, 75):
            clouds.append(deserialize_message(payload, PointCloud2))
        elif topic == "/points_raw":
            clouds.append(None)
    return np.asarray(poses, dtype=np.float64), clouds[0], clouds[75]


def map_points(cloud, poses):
    points = np.frombuffer(cloud.data, dtype=POINT_DTYPE)
    times = stamp_ns(cloud.header.stamp) + points["time"].astype(np.float64) * 1e9
    yaw = np.interp(times, poses[:, 0], poses[:, 3])
    x = np.interp(times, poses[:, 0], poses[:, 1])
    y = np.interp(times, poses[:, 0], poses[:, 2])
    local_x = points["x"].astype(np.float64) + 0.2
    local_y = points["y"].astype(np.float64)
    global_x = x + np.cos(yaw) * local_x - np.sin(yaw) * local_y
    global_y = y + np.sin(yaw) * local_x + np.cos(yaw) * local_y
    global_z = points["z"].astype(np.float64) + 0.5
    wall_slice = (global_z > 0.6) & (global_z < 3.0)
    return global_x[wall_slice], global_y[wall_slice]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    poses, first_cloud, turn_cloud = load(args.bag)
    start = poses[0, 0]
    time_s = (poses[:, 0] - start) / 1e9
    first_x, first_y = map_points(first_cloud, poses)
    turn_x, turn_y = map_points(turn_cloud, poses)

    fig, (map_ax, motion_ax) = plt.subplots(1, 2, figsize=(13, 5.6), layout="constrained")
    map_ax.scatter(first_x, first_y, s=1.0, alpha=0.22, color="#0284c7", label="LiDAR at 0 s")
    map_ax.scatter(turn_x, turn_y, s=1.0, alpha=0.22, color="#ea580c", label="LiDAR at 7.5 s")
    map_ax.plot(poses[:, 1], poses[:, 2], color="#111827", linewidth=2.2, label="Ground-truth path")
    map_ax.scatter(poses[0, 1], poses[0, 2], marker="o", s=65, color="#16a34a", zorder=5, label="Start")
    map_ax.scatter(poses[-1, 1], poses[-1, 2], marker="s", s=65, color="#7c3aed", zorder=5, label="Stop")
    map_ax.set_title("S1 scene and ground-truth path")
    map_ax.set_xlabel("Map x [m]")
    map_ax.set_ylabel("Map y [m]")
    map_ax.set_aspect("equal", adjustable="box")
    map_ax.set_xlim(-2.8, 12.8)
    map_ax.set_ylim(-5.8, 5.8)
    map_ax.grid(alpha=0.2)
    map_ax.legend(loc="lower right", bbox_to_anchor=(0.98, 0.15), fontsize=8, frameon=False)

    motion_ax.plot(time_s, poses[:, 4], color="#0284c7", linewidth=2, label="Forward speed")
    motion_ax.set_xlabel("Time since start [s]")
    motion_ax.set_ylabel("Forward speed [m/s]", color="#0284c7")
    motion_ax.set_xlim(0, 15)
    motion_ax.set_ylim(-0.04, 0.58)
    motion_ax.tick_params(axis="y", colors="#0284c7")
    yaw_ax = motion_ax.twinx()
    yaw_ax.plot(time_s, np.degrees(poses[:, 3]), color="#ea580c", linewidth=2, label="Heading")
    yaw_ax.set_ylabel("Heading [deg]", color="#ea580c")
    yaw_ax.tick_params(axis="y", colors="#ea580c")
    motion_ax.axvspan(0, 5, color="#94a3b8", alpha=0.11)
    motion_ax.axvspan(5, 10, color="#f97316", alpha=0.08)
    motion_ax.axvspan(11, 15, color="#94a3b8", alpha=0.11)
    motion_ax.set_title("Commanded motion in recorded truth")
    motion_ax.grid(alpha=0.2)
    motion_ax.text(2.5, 0.54, "Straight", ha="center", fontsize=9)
    motion_ax.text(7.5, 0.54, "Turn", ha="center", fontsize=9)
    motion_ax.text(13, 0.54, "Stopped", ha="center", fontsize=9)
    fig.suptitle("ROS 2 Jazzy S1 pilot bag — recorded data", fontsize=14, fontweight="bold")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    print(args.output)


if __name__ == "__main__":
    main()
