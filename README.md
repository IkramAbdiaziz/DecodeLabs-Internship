# DecodeLabs Robotics and Computer Vision Internship Repository

This repository contains the projects completed during the DecodeLabs engineering internship. The work spans multi-disciplinary robotics software development, physical physics simulation, and real-time computer vision processing.

Developer: Ikram Abdiaziz

---

## Tasks Directory

This workspace is organized into three core tasks, each fully self-contained with its own configuration files, launch setups, virtual execution environments, and visual evidence:

* [Task 1: Robotic Arm Kinematics and Path Planning](Task-1-IkramAbdiaziz/)
* [Task 2: Automated Quality Inspection](Task-2-IkramAbdiaziz/)
* [Task 3: Autonomous Mobile Robot Navigation](Task-3-IkramAbdiaziz/)

---

## Technical Overview

### Task 1: Robotic Arm Kinematics and Path Planning

A ROS 2 package executing path planning and kinematic state resolution for a 6-DOF robotic manipulator. The system computes joint trajectories to navigate the arm to targets in simulated environments.

* **Core Features**:
  * Real-time analytical and numerical Inverse Kinematics (IK) solvers.
  * Direct collision scene description in the Unified Robot Description Format (URDF).
  * Trajectory splining for smooth joint space transitions.
  * Gazebo physics engine integration with ROS 2 control interfaces.
* **Technology Stack**: C++, ROS 2 Humble, Gazebo Sim, RViz2, KDL Kinematics Library, Docker.

### Task 2: Automated Quality Inspection (Computer Vision)

An industrial-grade computer vision pipeline designed to inspect manufactured gears for structural anomalies (cracks, missing teeth, chipped profiles) using geometric and contour features.

* **Core Features**:
  * Real-time contour isolation and Convex Hull defect mapping.
  * Programmable verification thresholds with 100% classification accuracy.
  * Automated generation of synthetic control and defective datasets.
  * Logged industrial PLC fail trigger outputs (`PLC_FAIL_TRIGGER` 0 or 1).
* **Technology Stack**: Python, OpenCV, NumPy, Matplotlib.
* **Performance Benchmark**: ~11 ms latency per frame (~90 Frames Per Second throughput).

### Task 3: Autonomous Mobile Robot (AMR) Navigation

An integrated ROS 2 package simulating a differential-drive AMR navigating autonomously inside a corridor maze using active mapping, state estimation, path planning, and safety deceleration.

* **Core Features**:
  * **Planar State Estimation**: extended Kalman Filter (EKF) fusing wheel odometry and IMU signals via `robot_localization` to publish `/odometry/filtered`.
  * **Ceres SLAM Mapping**: Asynchronous occupancy grid mapping and loop-closure tracking via the `slam_toolbox`.
  * **Custom Pathfinder**: Obstacle dilation grid with an 8-connected A* search node and a proportional lookahead steering controller.
  * **Reflex Safety Decelerator**: Front-facing LiDAR sweep analyzing safety zones to scale velocities dynamically or command emergency brakes.
* **Technology Stack**: Python, ROS 2 Humble, Gazebo Sim, RViz2, robot_localization, slam_toolbox, Docker.

---

## Verification and Execution Summary

Detailed launch instructions and operational walkthroughs are provided in the README file of each respective task folder. Below is a quick execution command table:

| Task Name | Primary Directory | Launch Command (Container Setup) |
| :--- | :--- | :--- |
| **Task 1: Robotic Manipulator** | `Task-1-IkramAbdiaziz` | `./run_docker.sh` |
| **Task 2: Computer Vision Gear Inspector** | `Task-2-IkramAbdiaziz` | `python3 Task-2-IkramAbdiaziz/generate_dataset.py` <br> `python3 Task-2-IkramAbdiaziz/src/batch_process.py` |
| **Task 3: Autonomous AMR** | `Task-3-IkramAbdiaziz` | `./run_docker.sh` |

---

For detailed code structures, sensor diagrams, math formulations, and video/image results of each project, navigate to the specific task directory links.
