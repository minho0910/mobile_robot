#!/usr/bin/env python3
"""Write a deterministic, headless ROS 2 S1 LiDAR/IMU pilot bag."""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_msgs.msg import TFMessage

from bag_semantics import update_digest


START_NS = 1_700_000_000_000_000_000
IMU_HZ = 200
LIDAR_HZ = 10
AZIMUTH_STEP_DEG = 0.4
RINGS = 16
ROOM_MIN = np.array([-2.0, -5.0, 0.0])
ROOM_MAX = np.array([12.0, 5.0, 4.0])
PILLARS = [
    (np.array([3.0, -2.5, 0.0]), np.array([3.6, -1.9, 3.0])),
    (np.array([7.0, 1.8, 0.0]), np.array([7.8, 2.6, 3.0])),
]
POINT_DTYPE = np.dtype(
    {
        "names": ["x", "y", "z", "intensity", "ring", "time"],
        "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
        "offsets": [0, 4, 8, 12, 16, 20],
        "itemsize": 24,
    }
)


def smoothstep(value):
    u = np.clip(value, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def smoothstep_derivative(value):
    u = np.clip(value, 0.0, 1.0)
    return np.where((value > 0.0) & (value < 1.0), 6.0 * u * (1.0 - u), 0.0)


def motion(t):
    """Planar body speed, acceleration, yaw rate and yaw acceleration."""
    t = np.asarray(t)
    speed = 0.5 * smoothstep(t)
    speed = np.where(t >= 1.0, 0.5, speed)
    speed = np.where(t >= 10.0, 0.5 * (1.0 - smoothstep(t - 10.0)), speed)
    speed = np.where(t >= 11.0, 0.0, speed)
    acceleration = np.where(t < 1.0, 0.5 * smoothstep_derivative(t), 0.0)
    acceleration = np.where(
        (t >= 10.0) & (t < 11.0), -0.5 * smoothstep_derivative(t - 10.0), acceleration
    )
    yaw_rate = np.where(t < 5.0, 0.0, 0.25 * smoothstep(t - 5.0))
    yaw_rate = np.where(t >= 6.0, 0.25, yaw_rate)
    yaw_rate = np.where(t >= 9.0, 0.25 * (1.0 - smoothstep(t - 9.0)), yaw_rate)
    yaw_rate = np.where(t >= 10.0, 0.0, yaw_rate)
    yaw_acceleration = np.where(
        (t >= 5.0) & (t < 6.0), 0.25 * smoothstep_derivative(t - 5.0), 0.0
    )
    yaw_acceleration = np.where(
        (t >= 9.0) & (t < 10.0), -0.25 * smoothstep_derivative(t - 9.0), yaw_acceleration
    )
    return speed, acceleration, yaw_rate, yaw_acceleration


def trajectory(duration):
    steps = round(duration * IMU_HZ)
    t = np.arange(steps + 1, dtype=np.float64) / IMU_HZ
    speed, acceleration, yaw_rate, yaw_acceleration = motion(t)
    yaw = np.zeros_like(t)
    x = np.zeros_like(t)
    y = np.zeros_like(t)
    dt = 1.0 / IMU_HZ
    yaw[1:] = np.cumsum((yaw_rate[:-1] + yaw_rate[1:]) * (0.5 * dt))
    vx = speed * np.cos(yaw)
    vy = speed * np.sin(yaw)
    x[1:] = np.cumsum((vx[:-1] + vx[1:]) * (0.5 * dt))
    y[1:] = np.cumsum((vy[:-1] + vy[1:]) * (0.5 * dt))
    return t, x, y, yaw, speed, acceleration, yaw_rate, yaw_acceleration


def stamp(ns):
    msg = Time()
    msg.sec = int(ns // 1_000_000_000)
    msg.nanosec = int(ns % 1_000_000_000)
    return msg


def quaternion(yaw):
    from geometry_msgs.msg import Quaternion

    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def box_intersection(origin, direction, lower, upper):
    with np.errstate(divide="ignore", invalid="ignore"):
        entry = (lower - origin) / direction
        exit_ = (upper - origin) / direction
    near = np.max(np.minimum(entry, exit_), axis=1)
    far = np.min(np.maximum(entry, exit_), axis=1)
    return np.where(far >= np.maximum(near, 0.0), np.where(near > 0.0, near, far), np.inf)


def cloud(scan_index, track, rng, azimuth_step):
    t_grid, x_track, y_track, yaw_track = track[:4]
    scan_start = scan_index / LIDAR_HZ
    azimuths = np.deg2rad(np.arange(round(360.0 / azimuth_step)) * azimuth_step)
    elevations = np.deg2rad(np.linspace(-15.0, 15.0, RINGS))
    relative_time = np.repeat(np.arange(len(azimuths)) / len(azimuths) / LIDAR_HZ, RINGS)
    azimuth = np.repeat(azimuths, RINGS)
    elevation = np.tile(elevations, len(azimuths))
    acquisition_time = scan_start + relative_time
    yaw = np.interp(acquisition_time, t_grid, yaw_track)
    px = np.interp(acquisition_time, t_grid, x_track)
    py = np.interp(acquisition_time, t_grid, y_track)
    origin = np.column_stack((px + 0.2 * np.cos(yaw), py + 0.2 * np.sin(yaw), np.full_like(px, 0.5)))
    local_direction = np.column_stack(
        (np.cos(elevation) * np.cos(azimuth), np.cos(elevation) * np.sin(azimuth), np.sin(elevation))
    )
    global_direction = np.column_stack(
        (
            np.cos(elevation) * np.cos(azimuth + yaw),
            np.cos(elevation) * np.sin(azimuth + yaw),
            np.sin(elevation),
        )
    )
    distance = box_intersection(origin, global_direction, ROOM_MIN, ROOM_MAX)
    for lower, upper in PILLARS:
        distance = np.minimum(distance, box_intersection(origin, global_direction, lower, upper))
    valid = np.isfinite(distance) & (distance >= 0.5) & (distance <= 25.0)
    distance = distance[valid] + rng.normal(0.0, 0.01, int(np.count_nonzero(valid)))
    points = np.zeros(int(np.count_nonzero(valid)), dtype=POINT_DTYPE)
    points["x"] = local_direction[valid, 0] * distance
    points["y"] = local_direction[valid, 1] * distance
    points["z"] = local_direction[valid, 2] * distance
    points["intensity"] = 100.0
    points["ring"] = np.tile(np.arange(RINGS, dtype=np.uint16), len(azimuths))[valid]
    points["time"] = relative_time[valid]
    msg = PointCloud2()
    msg.header.stamp = stamp(START_NS + round(scan_start * 1e9))
    msg.header.frame_id = "velodyne"
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = POINT_DTYPE.itemsize
    msg.row_step = msg.point_step * msg.width
    msg.data = points.tobytes()
    msg.is_dense = True
    return msg


def imu(index, track, rng, gyro_bias, accel_bias, estimated_yaw):
    _, _, _, _, speed, acceleration, yaw_rate, _ = track
    ns = START_NS + round(index / IMU_HZ * 1e9)
    gyro_z = yaw_rate[index] + gyro_bias + rng.normal(0.0, 0.002)
    measured_accel = np.array([acceleration[index], speed[index] * yaw_rate[index], 9.81])
    measured_accel += accel_bias + rng.normal(0.0, 0.02, 3)
    estimated_yaw += gyro_z / IMU_HZ if index > 0 else 0.0
    msg = Imu()
    msg.header.stamp = stamp(ns)
    msg.header.frame_id = "imu_link"
    msg.orientation = quaternion(estimated_yaw)
    msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.05]
    msg.angular_velocity.z = float(gyro_z)
    msg.angular_velocity_covariance = [4e-6, 0.0, 0.0, 0.0, 4e-6, 0.0, 0.0, 0.0, 4e-6]
    msg.linear_acceleration.x = float(measured_accel[0])
    msg.linear_acceleration.y = float(measured_accel[1])
    msg.linear_acceleration.z = float(measured_accel[2])
    msg.linear_acceleration_covariance = [4e-4, 0.0, 0.0, 0.0, 4e-4, 0.0, 0.0, 0.0, 4e-4]
    return msg, estimated_yaw


def truth(index, track):
    _, x, y, yaw, speed, _, yaw_rate, _ = track
    msg = Odometry()
    msg.header.stamp = stamp(START_NS + round(index / IMU_HZ * 1e9))
    msg.header.frame_id = "map"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x = float(x[index])
    msg.pose.pose.position.y = float(y[index])
    msg.pose.pose.orientation = quaternion(float(yaw[index]))
    msg.twist.twist.linear.x = float(speed[index])
    msg.twist.twist.angular.z = float(yaw_rate[index])
    return msg


def static_transforms():
    result = []
    for child, x, z in (("velodyne", 0.2, 0.5), ("imu_link", 0.0, 0.3)):
        transform = TransformStamped()
        transform.header.stamp = stamp(START_NS)
        transform.header.frame_id = "base_link"
        transform.child_frame_id = child
        transform.transform.translation.x = x
        transform.transform.translation.z = z
        transform.transform.rotation.w = 1.0
        result.append(transform)
    return TFMessage(transforms=result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New rosbag2 directory")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--azimuth-step", type=float, default=AZIMUTH_STEP_DEG)
    args = parser.parse_args()
    if args.duration < 15.0 or abs(args.duration * IMU_HZ - round(args.duration * IMU_HZ)) > 1e-9:
        parser.error("S1 duration must be at least 15 seconds and a multiple of 0.005 seconds")
    if args.azimuth_step <= 0 or abs(360.0 / args.azimuth_step - round(360.0 / args.azimuth_step)) > 1e-9:
        parser.error("azimuth step must divide 360 degrees")
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")
    args.output.parent.mkdir(parents=True, exist_ok=True)

    track = trajectory(args.duration)
    rng = np.random.default_rng(args.seed)
    gyro_bias = float(rng.normal(0.0, 0.0005))
    accel_bias = rng.normal(0.0, 0.005, 3)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(args.output), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {
        "/clock": "rosgraph_msgs/msg/Clock",
        "/imu/data": "sensor_msgs/msg/Imu",
        "/points_raw": "sensor_msgs/msg/PointCloud2",
        "/ground_truth": "nav_msgs/msg/Odometry",
        "/tf_static": "tf2_msgs/msg/TFMessage",
    }
    for name, type_name in topic_types.items():
        qos = [rosbag2_py._storage.QoS(1).reliable().transient_local()] if name == "/tf_static" else []
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=0,
                name=name,
                type=type_name,
                serialization_format="cdr",
                offered_qos_profiles=qos,
            )
        )
    digest = hashlib.sha256()
    counts = {name: 0 for name in topic_types}

    def write(name, msg, timestamp_ns):
        payload = serialize_message(msg)
        writer.write(name, payload, timestamp_ns)
        update_digest(digest, name, timestamp_ns, msg)
        counts[name] += 1

    write("/tf_static", static_transforms(), START_NS)
    estimated_yaw = 0.0
    steps = round(args.duration * IMU_HZ)
    for index in range(steps):
        timestamp_ns = START_NS + round(index / IMU_HZ * 1e9)
        clock = Clock(clock=stamp(timestamp_ns))
        write("/clock", clock, timestamp_ns)
        measurement, estimated_yaw = imu(index, track, rng, gyro_bias, accel_bias, estimated_yaw)
        write("/imu/data", measurement, timestamp_ns)
        if index % (IMU_HZ // LIDAR_HZ) == 0:
            scan_index = index // (IMU_HZ // LIDAR_HZ)
            write("/points_raw", cloud(scan_index, track, rng, args.azimuth_step), timestamp_ns)
            write("/ground_truth", truth(index, track), timestamp_ns)
    del writer
    manifest = {
        "scenario": "S1",
        "seed": args.seed,
        "duration_s": args.duration,
        "start_time_ns": START_NS,
        "lidar_hz": LIDAR_HZ,
        "imu_hz": IMU_HZ,
        "rings": RINGS,
        "azimuth_step_deg": args.azimuth_step,
        "gyro_bias_rad_s": gyro_bias,
        "accel_bias_m_s2": accel_bias.tolist(),
        "range_noise_std_m": 0.01,
        "gyro_noise_std_rad_s": 0.002,
        "accel_noise_std_m_s2": 0.02,
        "counts": counts,
        "semantic_sha256": digest.hexdigest(),
        "ground_truth_is_input": False,
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"bag": str(args.output), "counts": counts, "semantic_sha256": digest.hexdigest()}))


if __name__ == "__main__":
    main()
