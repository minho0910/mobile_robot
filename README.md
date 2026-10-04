# Mobile Robot LiDAR–IMU SLAM experiments

This repository will host the ROS 2 Jazzy environment and reproducible LiDAR–IMU SLAM experiments. The local experiment plan is a proposal; no algorithm comparison results are available yet.

## Development environment

- Host: Windows with Docker Desktop using Linux containers, or Ubuntu 24.04 with Docker.
- Container: official ROS 2 Jazzy `ros:jazzy-ros-base-noble` image plus rosbag2, TF tools, rosdep, and colcon.
- Data and results stay on the host under ignored `data/` and `results/` directories. Do not commit large bags or generated results.

Start Docker Desktop first on Windows. From the repository root:

```sh
docker compose up -d --build
docker compose exec ros bash scripts/smoke.sh
docker compose exec ros bash
```

Inside the container, source ROS in each new interactive shell before using its tools:

```sh
source /opt/ros/jazzy/setup.bash
ros2 --help
```

Stop the development container with `docker compose down`. The bind-mounted repository files remain on the host.

## First implementation milestones

1. Generate and validate the S1 pilot bag using the commands below.
2. Run KISS-ICP on that bag. Then choose and pin a ROS 2 Jazzy compatible FAST-LIO2 port and the LIO-SAM ROS 2 branch, verify each on its own official example, and connect them to the same bag. Record exact commits and any message adapters.
3. Implement trajectory export and verified ATE, RPE, failure, and processing-time calculations. Run the 27 baseline cases only after the single-bag pilot passes.

Keep ground truth isolated from algorithm input topics. Disable LIO-SAM loop closure and GPS correction when comparing local odometry. Any algorithm-specific point-field conversion belongs in a versioned adapter, not in an undocumented manual bag edit.

## S1 pilot bag

The headless generator models a rectangular room with two pillars and a planar robot that moves straight, turns, and stops. It writes a 16-ring, 10 Hz rotating cloud (0.4-degree azimuth spacing), 200 Hz IMU, 10 Hz ground truth, `/clock`, and two static sensor transforms. A fixed random seed controls range and IMU noise. The IMU orientation is obtained by integrating its noisy gyro reading; it is not copied from ground truth.

Run these commands after `docker compose up -d --build`. Choose a new output directory each time; the generator intentionally refuses to overwrite an existing bag.

```sh
docker compose exec ros bash -c "source /opt/ros/jazzy/setup.bash; python3 scripts/generate_s1.py --output data/s1_seed001 --seed 1"
docker compose exec ros bash -c "source /opt/ros/jazzy/setup.bash; python3 scripts/validate_s1.py data/s1_seed001"
docker compose exec ros bash -c "source /opt/ros/jazzy/setup.bash; ros2 bag info data/s1_seed001"
```

The validator checks topic counts and types, timestamps, point-field layout, per-point relative time, TF frame names, stationary IMU specific force, motion phases, and the semantic SHA-256 in `manifest.json`. It hashes message values because serialized CDR padding and SQLite metadata can differ between otherwise identical runs. A 15-second bag contains 150 clouds, 3,000 IMU samples, 150 truth poses, and 3,000 clock messages; expect roughly 50 MiB of bag data.

The fixed bag timestamp is synthetic and is not a data collection date. This pilot uses a simplified raycast and noise model, so it is an interface and pipeline check rather than a sensor-fidelity benchmark. Ground truth is recorded only on `/ground_truth`; no `map -> base_link` truth TF is published.

## Hardware transition

The same topic and frame contract should be used when physical sensors arrive. Before recording comparison data, measure LiDAR–IMU time offset, extrinsic transform, and stationary IMU noise. Replace simulation assumptions with measured values in a versioned configuration; retain the original synthetic bags for regression checks.

The Docker Compose setup is intended for offline development and bag replay. When connecting live sensors, check DDS discovery and device access on the target Ubuntu machine; Docker Desktop networking on Windows may need a different runtime configuration.

## Current status

The Jazzy container, sensor contract, and S1 pilot bag generator are available. No SLAM algorithm port, physical sensor driver, or trajectory evaluation package has been added yet.
