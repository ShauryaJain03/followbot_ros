# bot_terrain_follower

ROS 2 package for a self-contained terrain-capability-aware human-following proof of concept.

The demo compares two behaviors in the Baylands rough-terrain world:

- Baseline: direct human following. The robot tries to follow the human across stairs and should fail or lose the target.
- Terrain mapping: live LiDAR produces a rolling local grid; registered clouds accumulate into a map-frame terrain grid.

## Package Layout

- `human_pose_publisher.py`: temporary deterministic human target publisher for isolated tests.
- `robot_ground_truth_publisher.py`: extracts the robot pose from Gazebo pose info and publishes `/robot_pose_gt`.
- `naive_follower.py`: direct baseline follower that ignores terrain.
- `traversability_analyzer.py`: 2.5D elevation-grid terrain mapper with slope, step, roughness, and obstacle classification.
- `demo_metrics_logger.py`: reports current and maximum robot-human distance.
- `config/follower.yaml`: follower and metric parameters.
- `config/traversability.yaml`: terrain grid and capability thresholds.
- `launch/baseline_follow_demo.launch.py`: launches Baylands, direct follower, and metrics.
- `launch/terrain_mapping_demo.launch.py`: launches Baylands, the terrain mapper, and RViz.

## Build

From the workspace root:

```sh
colcon build --packages-select bot_terrain_follower
source install/setup.bash
```

## Launch Targets

Baseline:

```sh
ros2 launch bot_terrain_follower baseline_follow_demo.launch.py world_name:=baylands
```

Terrain-map visualization:

```sh
ros2 launch bot_terrain_follower terrain_mapping_demo.launch.py world_name:=baylands
```

## Topics

- `/human_pose` (`geometry_msgs/PoseStamped`): actual Gazebo actor pose from `gazebo_ros_actor_plugin`.
- `/human_path` (`nav_msgs/Path`): actor route received by `gazebo_ros_actor_plugin`.
- `/robot_pose_gt` (`geometry_msgs/PoseStamped`): robot ground truth pose from Gazebo.
- `/bot_controller/cmd_vel_unstamped` (`geometry_msgs/Twist`): velocity command.
- `/points` (`sensor_msgs/PointCloud2`): 3D lidar point cloud.
- `/terrain/local_grid` (`nav_msgs/OccupancyGrid`): rolling, `base_link`-frame terrain grid from the current cloud.
- `/terrain/global_grid` (`nav_msgs/OccupancyGrid`): map-frame grid accumulated from registered LiDAR clouds.
- `/terrain/costmap` (`nav_msgs/OccupancyGrid`): planner-facing fused map-frame terrain grid.
- `/terrain/markers` (`visualization_msgs/MarkerArray`): terrain classes for RViz (green/free, yellow/orange/costly, red/lethal).

## Next Implementation Steps

1. Verify the LiDAR cloud has a valid `map <- laser_link` transform from LIO-SAM before expecting the global grid.
   The RViz profile defaults to `base_link`, so the live local grid remains visible before LIO-SAM is started.
2. Tune slope, step, roughness, and obstacle thresholds against a recorded Baylands route.
3. Add terrain costmap inflation using the actual robot footprint.
4. Feed `/terrain/costmap` to the Smac Hybrid-A* planner.
5. Add a follow-goal manager that chooses a feasible slot near, rather than at, the human.
