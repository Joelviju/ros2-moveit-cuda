# ROS2 MoveIt CUDA Acceleration

GPU-accelerated motion planning evaluation for industrial robots using ROS 2 and MoveIt 2.

This project investigates CUDA-based parallel state and collision evaluation to identify motion planning bottlenecks and improve performance in cluttered environments while maintaining compatibility with existing planners.

---

## Overview

Motion planning for industrial robotic manipulators is computationally expensive, especially in cluttered workcells where collision checking and state evaluation dominate planning time. Standard ROS 2 MoveIt pipelines rely primarily on CPU-based evaluation, which limits scalability and responsiveness in complex environments.

This project explores whether GPU acceleration using CUDA can provide measurable and reproducible performance improvements by accelerating the state evaluation stage of the planning pipeline, without modifying core planner algorithms.

---

## Key Goals

- Identify performance bottlenecks in ROS 2 MoveIt motion planning
- Design a CUDA-based state evaluation backend
- Integrate GPU acceleration without altering existing planner logic
- Compare CPU vs GPU performance across varying environment complexity
- Maintain correctness, determinism, and compatibility with MoveIt 2

---

## Architecture

- **Frontend:** ROS 2 Humble + MoveIt 2
- **Planners:** OMPL-based sampling planners (e.g., RRTConnect, PRM)
- **Acceleration Target:** State and collision evaluation
- **GPU Backend:** CUDA (batched parallel evaluation)
- **Fallback:** CPU evaluation path for validation
- **Robots:** Universal Robots (UR series)
- **Environment:** Docker + VS Code Devcontainers with NVIDIA GPU passthrough

---

## Software Stack

- ROS 2 Humble
- MoveIt 2
- OMPL
- CUDA
- C++
- Docker / VS Code Devcontainers
- NVIDIA Docker (GPU passthrough)

---

## Repository Structure

```text
ros2-moveit-cuda/
├── .devcontainer/                 # Reproducible development environment
├── Dockerfile                     # CUDA + ROS 2 + MoveIt image
├── src/
│   ├── my_robot_description/      # Custom robot description
│   ├── my_robot_moveit_config/    # MoveIt configuration
│   └── Universal_Robots_ROS2_Description/
├── README.md
└── .gitignore
```
## Development Environment
```
This project uses VS Code Devcontainers for reproducibility.

Requirements

Docker

NVIDIA GPU

NVIDIA Container Toolkit

VS Code + Dev Containers extension
```
## Getting Started

```bash
git clone https://github.com/Joelviju/ros2-moveit-cuda.git
cd ros2-moveit-cuda

```
Open the folder in VS Code and select “Reopen in Container”.
Build
```bash

cd /workspaces/ros2_CUDA
colcon build
source install/setup.bash
```

## Status

Active development / research project

GPU acceleration is currently focused on evaluation and benchmarking. Planner integration and extended GPU kernels are ongoing.

## Future Work

GPU-accelerated distance field computation

Batched collision checking kernels

Hybrid CPU–GPU scheduling strategies

Dynamic environment support

Real hardware validation

Detailed benchmarking and analysis

## Motivation

This project emphasizes system level performance analysis and practical integration over algorithmic reinvention aiming to provide realistic insights into GPU acceleration for real world robotic motion planning pipelines.

<gazebo>
  <plugin
    name="ign_ros2_control::IgnitionROS2ControlPlugin"
    filename="ign_ros2_control-system">
    <parameters>
      $(find my_robot_moveit_config)/config/controllers.yaml
    </parameters>
  </plugin>
</gazebo>

```bash
## Gazebo (Ignition) + ros2_control Setup (ROS 2 Humble)

This project uses **Ignition Gazebo (Fortress)** with **ros2_control** for simulation.

---

### 1. Required Packages

Install Ignition Gazebo + ROS 2 control integrations:

```bash
sudo apt update
sudo apt install -y \
  ignition-fortress \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ign-ros2-control \
  ros-humble-controller-manager \
  ros-humble-ros2controlcli
```
