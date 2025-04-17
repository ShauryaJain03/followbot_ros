## Human Following Robot using ROS2

This project implements a 4 Wheel Differential Drive QR Code Following Bot with LiDAR-based Obstacle Avoidance and Autonomous Return

### Key Features:
* QR Code Following: Uses computer vision (YOLOv8) to detect and follow QR codes.
* LiDAR-based Obstacle Avoidance: Avoids obstacles while following the QR code.
* Autonomous Return: The bot can autonomously return to its starting position after following a QR code.
* SLAM Integration: Built-in SLAM for mapping and localization.
* ROS2 Integration: Uses ros2_control, Nav2, and slam_toolbox for navigation, SLAM, and control.
* GPS waypoint navigation - under development

### Prerequisites: 
* ROS2 Humble
* Ubuntu 22.04
* Gazebo Fortress 6.16

### Setup

1. Visualize URDF in RViz (without launch file)
   ```sh
   ros2 launch urdf_tutorial display.launch.py model:=/home/shaurya/armybot_diff/src/bot_description/urdf/bot.urdf.xacro

   ```
2. Launch Gazebo Simulation in Custom World
   ```sh
   ros2 launch bot_description gazebo.launch.py world_name={test_new/small_house/small_warehouse/empty/room_with_walls}

   ```
2. Launch Gazebo Simulation in world with GPS enabled
```sh
ros2 launch bot_description gps.launch.py world_name=empty

```
3. Launch Keyboard Teleop with ros2_control
   ```js
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/bot_controller/cmd_vel_unstamped

   ```
4. Run Computer Vision Node (QR Code Detection) with LiDAR based obstacle avoidance
   ```sh
   ros2 run bot_vision obstacle_avoidance
   ```
5. SLAM with slam_toolbox
   ```sh
   ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/bot_description/config/mapper_params_online_async.yaml use_sim_time:=true
   ```
6. Switch from Mapping to Localization
   ```sh
   ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/bot_description/config/mapper_params_online_async.yaml use_sim_time:=true
   ```
7. Control the Robot with Twist Mux
   ```sh
   ros2 run twist_mux twist_mux --ros-args --params-file ./src/bot_description/config/twist_mux.yaml -r cmd_vel_out:=bot_controller/cmd_vel_unstamped

   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy

   ```
8. Navigation Using ROS2
   ```sh
   ros2 launch bot_description navigation_launch.py use_sim_time:=true
   ```


<!-- USAGE EXAMPLES -->
### Usage

Insert images

<!-- Contributing -->
### Contributing

This repository is not open to contributions.

<!-- CONTACT -->
### Contact

Shaurya Jain - Reach me at jainshaurya.sj@gmail.com

<!-- ACKNOWLEDGMENTS -->
### Acknowledgments

Centre of Intelligent Robotics IIITA, Dr. Surya Prakash, Mr. Rohit Kumar
