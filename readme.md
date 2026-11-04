# FollowBot ROS

ROS 2 workspace for the FollowBot simulation and supporting perception,
localization, and terrain-mapping packages.

## Repository contents

| Package or directory | Contents |
|---|---|
| `bot_description` | FollowBot URDF/Xacro, Gazebo bridges, point-cloud transform node, RViz configuration, and simulation worlds. |
| `bot_controller` | `ros2_control` differential-drive controller configuration and launch file. |
| `bot_bringup` | Simulation bringup, controller launch, RViz launch, and Gazebo ground-truth odometry wiring. |
| `bot_bringup` | Simulation bringup and `robot_ground_truth_publisher`, which publishes `/robot_pose_gt`, `/odom`, and optionally `odom -> base_link` from Gazebo world poses. |
| `LIO-SAM` | LIO-SAM ROS 2 package with `run.launch.py`, configuration, messages, and services. |
| `human_detector/human_detector` | Python human-detector package using RGB, depth, camera information, and TF dependencies. |

## Simulation assets

`bot_description/worlds` contains these Gazebo worlds:

- `empty`
- `baylands`

The robot model includes RGB-D, IMU, 2D LiDAR, and 3D LiDAR sensors. The
Gazebo bridge exposes their ROS topics and starts `pointcloud_transform`, which
converts `/points` into `/velodyne_points` for LIO-SAM-compatible input.

## Build

Build the simulator stack:

```bash
cd ~/followbot_ws
colcon build --packages-select bot_description bot_controller bot_bringup --symlink-install
source install/setup.bash
```

## Ground-truth simulation launch

```bash
ros2 launch bot_bringup bot_bringup.launch.py \
  world_name:=empty \
  use_ground_truth_odom:=true \
  publish_ground_truth_odom:=true \
  use_rviz:=true
```

With ground-truth odometry enabled, `/odom` and `odom -> base_link` are
published by `robot_ground_truth_publisher`. In RViz, use `odom` as the fixed
frame.

## LIO-SAM launch mode

For LIO-SAM, launch the simulation without the ground-truth odometry publisher:

```bash
ros2 launch bot_bringup bot_bringup.launch.py \
  world_name:=baylands \
  use_ground_truth_odom:=true \
  publish_ground_truth_odom:=false \
  use_rviz:=false

ros2 launch lio_sam run.launch.py
```

In this mode, LIO-SAM publishes the localization transform chain. The expected
frames are `map -> odom -> base_link -> laser_link`.
