# OMOROBOT R1 v2 navigation and LiDAR–IMU research

This repository hosts a ROS 2 Jazzy development environment. The immediate target is 2D navigation for an OMOROBOT R1 v2 with a YDLIDAR G6. The LiDAR–IMU comparison remains a later research track.

The confirmed setup is R1 v2, G6 2D LiDAR, no external IMU, and no installed 3D LiDAR. The [R1 v2 and Nav2 development roadmap](docs/r1_jazzy_roadmap.md) gives the staged plan. The current S1 bag is a generic 3D sensor pilot and does not model R1 hardware.

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

## Next implementation milestones

1. Record the R1 v2 motor and wheel-odometry interface, G6 scan interface, TF frames, and measured geometry.
2. Build a Jazzy simulation with differential drive, 2D scan, wheel odometry, and an RViz view.
3. Add SLAM Toolbox mapping, saved-map AMCL localization, and Nav2 goal navigation. Validate these in simulation before hardware use.
4. When 3D LiDAR and IMU hardware is selected, resume the S1 algorithm-comparison track below.

For the later 3D comparison, keep ground truth isolated from algorithm input topics. Disable LIO-SAM loop closure and GPS correction when comparing local odometry. Any algorithm-specific point-field conversion belongs in a versioned adapter, not in an undocumented manual bag edit.

## S1 pilot bag

The headless generator models a rectangular room with two pillars and a planar robot that moves straight, turns, and stops. It writes a 16-ring, 10 Hz rotating cloud (0.4-degree azimuth spacing), 200 Hz IMU, 10 Hz ground truth, `/clock`, and two static sensor transforms. A fixed random seed controls range and IMU noise. The IMU orientation is obtained by integrating its noisy gyro reading; it is not copied from ground truth.

Run these commands after `docker compose up -d --build`. Choose a new output directory each time; the generator intentionally refuses to overwrite an existing bag.

```sh
docker compose exec ros bash -c "source /opt/ros/jazzy/setup.bash; python3 scripts/generate_s1.py --output data/s1_seed001 --seed 1"
docker compose exec ros bash -c "source /opt/ros/jazzy/setup.bash; python3 scripts/validate_s1.py data/s1_seed001"
docker compose exec ros bash -c "source /opt/ros/jazzy/setup.bash; ros2 bag info data/s1_seed001"
docker compose exec ros bash -c "source /opt/ros/jazzy/setup.bash; python3 scripts/plot_s1.py data/s1_seed001 --output results/s1_preview.png"
```

The validator checks topic counts and types, timestamps, point-field layout, per-point relative time, TF frame names, stationary IMU specific force, motion phases, and the semantic SHA-256 in `manifest.json`. It hashes message values because serialized CDR padding and SQLite metadata can differ between otherwise identical runs. A 15-second bag contains 150 clouds, 3,000 IMU samples, 150 truth poses, and 3,000 clock messages; expect roughly 50 MiB of bag data.

The fixed bag timestamp is synthetic and is not a data collection date. This pilot uses a simplified raycast and noise model, so it is an interface and pipeline check rather than a sensor-fidelity benchmark. Ground truth is recorded only on `/ground_truth`; no `map -> base_link` truth TF is published.

The preview reads the recorded bag. It plots two LiDAR scans in map coordinates using the recorded ground-truth pose to place the points, alongside the true trajectory, forward speed, and heading. This is a visualization of the simulated input and truth, not a SLAM result.

## Hardware transition for the later 3D research track

When the 3D LiDAR and IMU are installed, measure their time offset, extrinsic transform, and stationary IMU noise before recording comparison data. Replace simulation assumptions with measured values in a versioned configuration; retain the original synthetic bags for regression checks.

The Docker Compose setup is intended for offline development and bag replay. When connecting live sensors, check DDS discovery and device access on the target Ubuntu machine; Docker Desktop networking on Windows may need a different runtime configuration.

## Current status

The Jazzy container, 3D sensor contract, and S1 pilot bag generator are available. The R1 v2 simulation, G6 driver integration, SLAM Toolbox/Nav2 configuration, and physical robot interface are not implemented yet.
