# Furbo — Autonomous Exploration Robot

A differential-drive mobile robot simulated in Gazebo (Harmonic) with ROS 2 Humble, capable of autonomously exploring and mapping unknown environments using a custom frontier-exploration algorithm built on top of SLAM and Nav2.

## Overview

This project implements a full autonomous exploration pipeline from scratch:

- Simulated diff-drive robot with lidar, IMU, and camera sensors (Gazebo Harmonic + `ros2_control`)
- Sensor fusion (wheel odometry + IMU) via an Extended Kalman Filter (`robot_localization`)
- Real-time SLAM (`slam_toolbox`) for mapping and localization
- Autonomous navigation (Nav2) with a custom-tuned costmap/controller configuration
- A custom frontier-exploration node (written from scratch, not a pre-built package) that autonomously drives the robot to fully explore a map with no manual input
- Quantitative comparison of two exploration strategies

## Architecture

Sensors (lidar, IMU, camera)
|
v
robot_localization (EKF) --odometry_filtered--> slam_toolbox (SLAM)
| |
v v
Nav2 (planning, control, recovery) <------- map -----
|
v
frontier_explorer.py (custom) --sends goals--> Nav2 action server


## Results: exploration strategy comparison

| Strategy | Time | Distance traveled | Coverage |
|---|---|---|---|
| Nearest frontier | 246.1 s | 13.73 m | 97.5% |
| Largest frontier | 68.1 s | 8.97 m | 96.6% |

The "largest frontier" strategy explored **3.6x faster** and traveled **35% less distance**, with equivalent final coverage. This matches known findings in frontier-exploration literature: pure nearest-neighbor selection tends to zigzag inefficiently, while information-gain-based selection (favoring larger frontiers) produces more efficient sweeping behavior.

## Key engineering challenges solved

- **TF frame mismatches** — Gazebo's default sensor frame naming didn't match the URDF's TF tree, silently breaking SLAM's ability to use lidar data. Diagnosed via `tf2` message-filter logs, fixed with static transform publishers.
- **Sensor fusion clock sync** — a missing `/clock` bridge caused the diff-drive controller to treat every velocity command as expired, silently discarding motion commands.
- **Self-collision on sensors** — the simulated lidar was embedded inside its own collision geometry, returning zero-range readings on every ray.
- **Accelerometer integration drift** — fusing raw IMU linear acceleration into the EKF produced an unobservable, uncorrected velocity bias. Resolved by fusing orientation/angular velocity only.
- **Wheel slip during rotation** — open-loop odometry has no way to detect physical wheel slip, causing yaw drift and duplicated map geometry. Mitigated by tuning wheel-ground friction.
- **"Chasing its own shadow"** — the frontier-exploration algorithm initially kept re-selecting phantom frontiers created by the robot's own body occluding its sensor. Fixed with a minimum-distance filter and a failure-blacklist mechanism.

## Stack

- ROS 2 Humble
- Gazebo Harmonic (`gz-sim`)
- `ros2_control` / `gz_ros2_control`
- `robot_localization` (EKF)
- `slam_toolbox`
- `navigation2`
- Custom Python frontier-exploration node

## Running it

```bash
colcon build
source install/setup.bash
ros2 launch furbo_description gazebo.launch.py
```

In a separate terminal, once the simulation, SLAM, and Nav2 are running:

```bash
ros2 run furbo_description frontier_explorer.py
```