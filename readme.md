## Human Following Robot using ROS2

This project implements a 4 Wheel Differential Drive Human Following robot with Obstacle Avoidance and Autonomous Return

### Key Features:
* ROS2 based software stack.
* Human Following using Apriltags (tag36h11 with id=0 for following, use id=1 for other purposes)
* Mapless and Map based Autonomous Navigation.

### Features Under Development
* GPS waypoint navigation 

### Prerequisites: 
* ROS2 Humble
* Ubuntu 22.04
* Gazebo Fortress v6.16

### Setup

1. Visualize URDF in RViz (without launch file)
   ```sh
   ros2 launch urdf_tutorial display.launch.py model:=/home/shaurya/armybot_diff/src/bot_description/urdf/bot.urdf.xacro

   ```
2. Launch Gazebo Simulation in Custom World
   ```sh
   ros2 launch bot_description gazebo.launch.py world_name:={baylands/test_new/small_house/small_warehouse/empty/room_with_walls}

   ```
3. Launch robot controller 
   ```sh
   ros2 launch bot_controller controller.launch.py
   
   ```
4. Launch Gazebo Simulation in GPS Enabled World
   ```sh
   ros2 launch bot_description gps.launch.py world_name=empty
   
   ```
5. Launch Keyboard Teleop with ros2_control
   ```js
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/bot_controller/cmd_vel_unstamped

   ```
6. Run Apriltag detection and Human Following
   ```sh
   ros2 launch apriltag_navigation apriltag_navigation.launch.py
   ```
7. SLAM with slam_toolbox
   ```sh
   ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/bot_description/config/mapper_params_online_async.yaml use_sim_time:=true
   ```
8. Switch from Mapping to Localization - make change in the params file
   ```sh
   ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/bot_description/config/mapper_params_online_async.yaml use_sim_time:=true
   ```
9. Control the Robot with Twist Mux
   ```sh
   ros2 run twist_mux twist_mux --ros-args --params-file ./src/bot_description/config/twist_mux.yaml -r cmd_vel_out:=bot_controller/cmd_vel_unstamped

   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy

   ```
10. Navigation Using Nav2
      ```sh
      ros2 launch bot_description navigation_launch.py use_sim_time:=true
      ```
11. For Mapless Navigation - Launch Twist Mux node first and then proceed with following scripts
    ```sh
    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
   
    ros2 launch bot_description navigation_launch.py use_sim_time:=true
    
    ```   

<!-- USAGE EXAMPLES -->
### Usage

Images to be updated soon

<!-- Contributing -->
### Contributing

This repository is not open to contributions.

<!-- CONTACT -->
### Contact

Shaurya Jain - Reach me at jainshaurya.sj@gmail.com

<!-- ACKNOWLEDGMENTS -->
### Acknowledgments

Centre of Intelligent Robotics IIITA, Dr. Surya Prakash, Mr. Rohit Kumar
