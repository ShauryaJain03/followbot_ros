# bot_terrain_follower

ROS 2 package for a self-contained terrain-capability-aware human-following proof of concept.

The demo compares two behaviors in rough-terrain worlds:

- Baseline: direct human following. The robot tries to follow the human across stairs and should fail or lose the target.
- Terrain mapping: LIO-SAM keyframes feed a layered local/global
  `traversability_mapping` + `grid_map` terrain map.

## Package Layout

- `human_pose_publisher.py`: temporary deterministic human target publisher for isolated tests.
- `robot_ground_truth_publisher.py`: extracts the robot pose from Gazebo pose info and publishes `/robot_pose_gt`.
- `naive_follower.py`: direct baseline follower that ignores terrain.
- `lio_traversability_keyframe_bridge.py`: converts LIO-SAM LiDAR/pose pairs into
  external traversability-map keyframes without publishing duplicate TF.
- `demo_metrics_logger.py`: reports current and maximum robot-human distance.
- `config/follower.yaml`: follower and metric parameters.
- `config/lio_traversability_core.yaml`: terrain sampling and capability thresholds.
- `config/lio_traversability_ros.yaml`: LIO-SAM topic and frame integration.
- `launch/baseline_follow_demo.launch.py`: launches Baylands, direct follower, and metrics.
- `launch/lio_traversability_mapping.launch.py`: launches the LIO-SAM bridge and
  external local/global terrain mapping nodes.

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

### Traversability mapping

`lio_traversability_mapping.launch.py` runs the external
`traversability_mapping` / `grid_map` terrain front end with LIO-SAM. It starts
a TF-safe keyframe bridge plus the external local/global mapping nodes; it does
not run either of the external package's ground-truth keyframe simulators.

```sh
ros2 launch bot_terrain_follower lio_traversability_mapping.launch.py
```

The bridge consumes `/velodyne_points` and `/lio_sam/mapping/odometry`, publishes
`/traversability_keyframe_additions`, and never publishes transforms. The
external map outputs are `/global_traversability_gridmap`,
`/global_traversability_occupancy`, `/local_traversability_gridmap`, and
`/local_traversability_occupancy`.

## Topics

- `/human_pose` (`geometry_msgs/PoseStamped`): actual Gazebo actor pose from `gazebo_ros_actor_plugin`.
- `/human_path` (`nav_msgs/Path`): actor route received by `gazebo_ros_actor_plugin`.
- `/robot_pose_gt` (`geometry_msgs/PoseStamped`): robot ground truth pose from Gazebo.
- `/bot_controller/cmd_vel_unstamped` (`geometry_msgs/Twist`): velocity command.
- `/velodyne_points` (`sensor_msgs/PointCloud2`): input LiDAR cloud.
- `/lio_sam/mapping/odometry` (`nav_msgs/Odometry`): LIO-SAM LiDAR pose input.
- `/traversability_keyframe_additions` (`traversability_msgs/KeyFrameAdditions`):
  immutable, map-frame LIO-SAM keyframes from the bridge.
- `/global_traversability_gridmap` (`grid_map_msgs/GridMap`): persistent map-frame
  elevation and terrain-hazard layers.
- `/global_traversability_occupancy` (`nav_msgs/OccupancyGrid`): global hazard
  layer mapped to 0--100 cost, with unknown cells remaining unknown.
- `/local_traversability_gridmap` and `/local_traversability_occupancy`: rolling
  terrain outputs near the robot.

## Next Implementation Steps

1. Tune the external map's hazard layers against recorded terrain routes.
2. Convert global hazard and unknown terrain to a planner costmap using the
   actual robot footprint.
3. Feed that costmap to the custom Hybrid-A* objective.
4. Add a follow-goal manager that chooses a feasible slot near, rather than at,
   the human.
