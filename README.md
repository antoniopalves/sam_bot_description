# sam_bot navigation Setup

## Overview

This package provides a complete ROS 2 navigation and vision pipeline for the sam_bot differential-drive robot in Gazebo Classic.
It integrates:

- Full robot simulation in Gazebo.

- Visualization in RViz 2.

- Autonomous navigation using Nav2.

- Real-time RGB cone detection via OpenCV

## Prerequisites
Ensure that:
- ROS 2 Humble (or newer) is installed.
- Gazebo Classic and gazebo_ros_pkgs are installed.
- The workspace has been built and sourced:

```bash
cd ~/ros2_ws
colcon build --packages-select sam_bot_description
source install/setup.bash
````

## 1. Launch the Robot in Gazebo + RViz

```bash
ros2 launch sam_bot_description display.launch.py use_sim_time:=true
````

Purpose:
- Spawns the sam_bot in Gazebo using the custom my_world.sdf.
- Opens RViz2 preconfigured for sensor and TF visualization.
- Starts the robot_state_publisher and joint_state_publisher.

## 2. Run Nav2 Navigation Stack  (Online Mapping)

```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true \
map:=/home/antoniopalves/trsa_two_rooms_map.yaml
````

Purpose:
- Starts real-time SLAM (Simultaneous Localization and Mapping).
- Builds a map from the LiDAR /scan topic while the robot moves.
- Publishes map and pose transforms (/map → /odom → /base_footprint).

## 3. Run the Navigation Node

```bash
ros2 run sam_bot_description navigation_node.py
````

Purpose:
- Automatically publishes the initial pose (/initialpose).

- Provides the /start_navigation service that sends a predefined goal.

- Start navigation by calling:

```bash
ros2 service call /start_navigation std_srvs/srv/Trigger "{}"
````

## 4. Run the Cone Detection Node

```bash
ros2 run sam_bot_description cone_detector.py
````

Purpose:
- Subscribes to the RGB camera topic (/depth_camera/image_raw).

- Uses HSV color segmentation to detect orange-white striped cones.

- Publishes boolean detections to /cone_detected.

- Displays OpenCV windows:

- Mask Orange, Mask White, Combined, and View with bounding boxes.

## 5. Control the Robot Manually

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/sam_bot/cmd_vel
````

Purpose:
- Allows manual movement of the robot using keyboard commands (W, A, S, D).
- Movement data goes to the same topic Nav2 uses (/sam_bot/cmd_vel).

Expected Behavior

The robot loads into the environment and localizes itself using AMCL.

Calling /start_navigation triggers Nav2 to plan and follow a path to the goal.

The cone detector highlights orange-white cones and reports detections.

The vision node can later be extended to stop or reroute the robot upon cone detection.


## Topics Overview
/scan — LiDAR LaserScan data

/depth_camera/* — Depth camera images & point cloud

/sam_bot/cmd_vel — Velocity commands (manual or Nav2)

/sam_bot/odom — Odometry from differential drive

/map — Occupancy grid map (SLAM or static)

/tf, /tf_static — Frame transforms for robot & world


## Author
António Alves — MSc in Electrical and Computer Engineering (FCT NOVA)
Project: Autonomous Navigation and Mapping with ROS2 and Gazebo
