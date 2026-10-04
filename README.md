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

1. Implement a deterministic headless scene and trajectory generator. Emit `/points_raw` at 10 Hz, `/imu/data` at 200 Hz, `/ground_truth`, `/clock`, and static sensor transforms according to `config/sensor_contract.yaml`.
2. Validate a short S1 bag containing straight motion, a turn, and a stop. Check point fields and relative point times, frame IDs, TF connectivity, IMU gravity direction, timestamp monotonicity, message rates, and reproducibility across identical seeds.
3. Run KISS-ICP on that bag. Then choose and pin a ROS 2 Jazzy compatible FAST-LIO2 port and the LIO-SAM ROS 2 branch, verify each on its own official example, and connect them to the same bag. Record exact commits and any message adapters.
4. Implement trajectory export and verified ATE, RPE, failure, and processing-time calculations. Run the 27 baseline cases only after the single-bag pilot passes.

Keep ground truth isolated from algorithm input topics. Disable LIO-SAM loop closure and GPS correction when comparing local odometry. Any algorithm-specific point-field conversion belongs in a versioned adapter, not in an undocumented manual bag edit.

## Hardware transition

The same topic and frame contract should be used when physical sensors arrive. Before recording comparison data, measure LiDAR–IMU time offset, extrinsic transform, and stationary IMU noise. Replace simulation assumptions with measured values in a versioned configuration; retain the original synthetic bags for regression checks.

The Docker Compose setup is intended for offline development and bag replay. When connecting live sensors, check DDS discovery and device access on the target Ubuntu machine; Docker Desktop networking on Windows may need a different runtime configuration.

## Current status

The Jazzy container and interface contract are the starting point. No simulator, algorithm port, driver, or evaluation package has been added yet.

\n
