# bot_terrain_follower

ROS 2 package for the terrain-capability-aware human-following proof of concept.

The demo compares two behaviors in the Baylands rough-terrain world:

- Baseline: direct human following. The robot tries to follow the human across stairs and should fail or lose the target.
- POC: terrain-aware following. The robot classifies stairs as infeasible, plans around them, rejoins the human, and keeps distance bounded.

## Package Layout

- `human_pose_publisher.py`: temporary deterministic human target publisher for isolated tests.
- `robot_ground_truth_publisher.py`: extracts the robot pose from Gazebo pose info and publishes `/robot_pose_gt`.
- `naive_follower.py`: direct baseline follower that ignores terrain.
- `traversability_analyzer.py`: 3D-lidar traversability grid builder and RViz marker publisher.
- `capability_aware_follower.py`: terrain-aware follower scaffold.
- `demo_metrics_logger.py`: reports current and maximum robot-human distance.
- `config/follower.yaml`: follower and metric parameters.
- `config/traversability.yaml`: terrain grid and capability thresholds.
- `launch/baseline_follow_demo.launch.py`: launches Baylands, direct follower, and metrics.
- `launch/capability_follow_demo.launch.py`: launches Baylands, traversability analyzer, terrain-aware follower, and metrics.

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

Capability-aware POC:

```sh
ros2 launch bot_terrain_follower capability_follow_demo.launch.py world_name:=baylands
```

## Topics

- `/human_pose` (`geometry_msgs/PoseStamped`): actual Gazebo actor pose from `gazebo_ros_actor_plugin`.
- `/human_path` (`nav_msgs/Path`): actor route received by `gazebo_ros_actor_plugin`.
- `/robot_pose_gt` (`geometry_msgs/PoseStamped`): robot ground truth pose from Gazebo.
- `/bot_controller/cmd_vel_unstamped` (`geometry_msgs/Twist`): velocity command.
- `/points` (`sensor_msgs/PointCloud2`): 3D lidar point cloud.
- `/traversability_grid` (`nav_msgs/OccupancyGrid`): terrain feasibility grid.
- `/terrain_follower/path` (`nav_msgs/Path`): planned terrain-aware rejoin path.
- `/traversability_markers` (`visualization_msgs/MarkerArray`): RViz terrain visualization.

## Next Implementation Steps

1. Verify `/human_pose` and `/human_path` align with the visible Baylands actor.
2. Refine `traversability_analyzer` into a planner-ready local map with better cell scoring and obstacle inflation.
3. Implement A* in `capability_aware_follower` over `/traversability_grid`.
4. Add RViz markers for infeasible stairs, selected alternate path, and rejoin goal.
5. Run the same actor path with baseline and POC launch files and compare maximum human distance plus stuck time.
