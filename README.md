# Experimental Robotics Laboratory - Assignment 1
**Author:** Rubin Khadka Chhetri  
**Student ID:** 6558048  
**Course:** Experimental Robotics Laboratory 

## Table of Contents
- [Introduction](#introduction)
- [Robot Platforms & Simulation Environment](#robot-platforms--simulation-environment)
- [Getting Started](#getting-started-read-before-action)
    - [Prerequisites](#prerequisites)
    - [Setup](#setup)
- [Launching the System](#launching-the-system)
    - [Launch the Robot with Gazebo Environment](#launch-the-robot-with-gazebo-environment)
    - [Launch the Marker Detection Node](#launch-the-marker-detection-node)
- [Implementation Details](#implementation-details)

## Introduction

This assignment implements a comprehensive marker detection and visual servoing system using **ROS 2**, **OpenCV**, and **ArUco markers**. The system is capable of operating with two different robot configurations to detect, identify, and navigate to markers in a structured environment.

## Robot Platforms & Simulation Environment

### Differential Drive Robot
<img src="assets/robot_caster.png" width="50%" alt="Two-wheeled robot"> <br>
*Features two driven wheels at front and a caster wheel at back*

### Skid-Steer Robot
<img src="assets/robot_diff.png" width="50%" alt="Two-wheeled robot"> <br>
*Features four wheels with skid-steer drive controller*

### Gazebo environment
<img src="assets/gazeboenv.png" width="50%" alt="gazebo env"> <br> 
Custom world featuring 5 ArUco markers arranged in a circle around the robot spawn point

## Getting Started (Read Before Action)

### Prerequisites
Before proceeding, make sure that **`ROS2 Jazzy`** is installed on your system.<br>
If you haven’t set up ROS2 yet, refer to the official installation guide for ROS2 Jazzy on Ubuntu:<br>
[Install ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) <br>

**Additional Required Packages:**
- Gazebo
- OpenCV (with ArUco module)
- `ros_gz_bridge`
- `cv_bridge`
- `robot_state_publisher`
- `robot_localization`

### Setup 

#### 1. Set up your ROS workspace
Create a new workspace (or use an existing one) and navigate to its `src` directory:
```bash
mkdir -p ~/aruco_ws/src
cd ~/aruco_ws/src
```

#### 2. Clone this repository
Clone this repository into your workspace’s `src` folder:
```bash
git clone https://github.com/rubin-khadka/aruco_marker_robot.git
```

#### 3. Build the workspace
Navigate back to the root of your workspace and build the packages using `colcon build`:
```bash
cd ~/aruco_ws
colcon build
```

#### 4. Source the workspace
After building, source the workspace manually for the first time in the current terminal session:
```bash
source ~/aruco_ws/install/setup.bash
```

#### 5. Add the Workspace to your ROS Environment
To ensure that your workspace is sourced automatically every time you start a new terminal session, add it to your `.bashrc` file:
```bash
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Launching the System

### Launch the Robot with Gazebo Environment

You can run the assignment with two different robot configurations:

#### Option 1: Two-Wheeled Caster Robot
```bash
ros2 launch aruco_marker_robot aruco_world.launch.py 
```
#### Option 2: Four-Wheeled Skid-Steer Robot
```bash
ros2 launch aruco_marker_robot diff_aruco_world.launch.py 
```
**Both configurations will:**
- Spawn the robot in a Gazebo world with 5 ArUco markers
- Launch RViz for visualization

### Launch the Marker Detection Node
After launching the simulation, run the marker processor:
```bash
ros2 run aruco_marker_robot aruco_marker_processor 
```
**This node will:**
- Automatically rotate the robot 360° to detect all markers
- Sort markers by ID (lowest to highest)
- Center each marker in the camera view using visual servoing
- Publish processed images with circles to `/processed_image` topic
- Wait 10 seconds at each centered marker before moving to the next

## Implementation Details
