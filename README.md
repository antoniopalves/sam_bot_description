sam_bot_navigation_setup

Overview
This package sets up a complete ROS 2 navigation pipeline for the sam_bot differential drive robot in Gazebo Classic.
It includes visualization (RViz2), SLAM mapping (SLAM Toolbox), navigation (Nav2), costmap visualization, and manual teleoperation.

Prerequisites
Ensure that:
- ROS 2 Humble (or newer) is installed.
- Gazebo Classic and gazebo_ros_pkgs are installed.
- The workspace has been built and sourced:

cd ~/ros2_ws
colcon build
source install/setup.bash

1. Launch the Robot in Gazebo + RViz
ros2 launch sam_bot_description display.launch.py

Purpose:
- Spawns the sam_bot in Gazebo using the custom my_world.sdf.
- Opens RViz2 preconfigured for sensor and TF visualization.
- Starts the robot_state_publisher and joint_state_publisher.

2. Run SLAM Toolbox (Online Mapping)
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

Purpose:
- Starts real-time SLAM (Simultaneous Localization and Mapping).
- Builds a map from the LiDAR /scan topic while the robot moves.
- Publishes map and pose transforms (/map → /odom → /base_footprint).

3. Launch the Navigation Stack (Nav2)
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

Purpose:
- Initializes Nav2 components:
  - planner_server (path planning)
  - controller_server (velocity control)
  - behavior_server (navigation behavior trees)
  - bt_navigator (goal execution)
  - costmaps (obstacle representation)
- Uses the SLAM map in real time.

4. Visualize Costmaps in RViz
ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker

Purpose:
- Publishes RViz visualization markers for the 3D voxel grid layers of the costmap.
- Useful to debug obstacle inflation, sensor coverage, and costmap generation.

5. Control the Robot Manually
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/sam_bot/cmd_vel

Purpose:
- Allows manual movement of the robot using keyboard commands (W, A, S, D).
- Movement data goes to the same topic Nav2 uses (/sam_bot/cmd_vel).

6. Saving the Map
Once you have explored the world and built the map:
ros2 run nav2_map_server map_saver_cli -f ~/mapa_sam_bot
This generates:
mapa_sam_bot.yaml
mapa_sam_bot.pgm

Expected Behavior
- SLAM Toolbox builds a live occupancy map.
- Nav2 consumes this map for planning and control.
- Teleop allows manual exploration.
- After saving, the robot can localize and navigate autonomously within the saved map.

Topics Overview
/scan — LiDAR LaserScan data
/depth_camera/* — Depth camera images & point cloud
/sam_bot/cmd_vel — Velocity commands (manual or Nav2)
/sam_bot/odom — Odometry from differential drive
/map — Occupancy grid map (SLAM or static)
/tf, /tf_static — Frame transforms for robot & world

Notes
- Use use_sim_time:=true whenever working with Gazebo simulation.
- If RViz shows “No map received”, ensure the map server is launched or SLAM is running.
- Gazebo Classic reaches end-of-life January 2025 — consider migrating to Gazebo (Ignition) if needed.

Author
António Alves — MSc in Electrical and Computer Engineering (FCT NOVA)
Project: Autonomous Navigation and Mapping with ROS2 and Gazebo
