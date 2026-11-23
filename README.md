# FollowBot

ROS 2 Jazzy / Gazebo Harmonic simulation of a wheeled robot following a
ground-truth Gazebo actor across outdoor terrain. The current milestone is
safe **local** following on terrain that is physically traversable; it does
not yet plan a detour when the person takes an impossible route.

## Current pipeline

```text
Gazebo 3D LiDAR + IMU
  -> GSeg3D ground segmentation
  -> ground-consistency local costmap
  -> elevation map + analytic traversability cost layer
  -> Following Server projected-trajectory collision check
  -> Collision Monitor -> Gazebo DiffDrive

Gazebo actor ground truth -> /person_pose -> Following Server
```

The default RViz configuration shows the raw and filtered person poses, ground
and obstacle clouds, local/global costmaps, and the magenta costmap-checked
following trajectory.

## Build and source overlays

This project expects these overlays, in this order:

1. `/opt/ros/jazzy` — ROS 2 Jazzy.
2. `~/nav2_ws` — the Jazzy Nav2 tutorial dependencies plus the local
   `opennav_following` source patch that enables collision checking.
3. `~/ros2_ws` — Elevation Mapping CuPy observer stack.
4. `~/followbot_ws` — this repository and the actor plugin.

Clone the actor plugin beside this repository and apply the tracked
ground-truth pose patch once:

```bash
cd ~/followbot_ws/src
git clone --branch jazzy-harmonic \
  https://github.com/blackcoffeerobotics/gazebo-ros-actor-plugin.git
cd gazebo-ros-actor-plugin
git apply ../followbot_ros/patches/gazebo_ros_actor_plugin_ground_truth_pose.patch
```

Build the required Nav2 packages once (or after changing the Nav2 source):

```bash
cd ~/nav2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  ground_segmentation_ros2 \
  nav2_ground_consistency_costmap_plugin \
  opennav_following \
  --symlink-install
```

Build FollowBot after changing its source or the actor plugin:

```bash
cd ~/followbot_ws
source /opt/ros/jazzy/setup.bash
source ~/nav2_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
colcon build --packages-select \
  gazebo_ros_actor_plugin \
  bot_description \
  bot_bringup \
  --symlink-install
```

Build the elevation-map observer once:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
git clone https://github.com/SirErico/nav2_gridmap_traversability_layer.git src/nav2_gridmap_traversability_layer
rosdep install --from-paths src/nav2_gridmap_traversability_layer --ignore-src -r -y
colcon build --packages-select \
  elevation_map_msgs \
  semantic_sensor \
  elevation_mapping_cupy \
  nav2_gridmap_traversability_layer \
  --symlink-install
```

Its GPU runtime uses user-level Python packages. Install the CuPy 13.x and
PyTorch CUDA runtimes before building; the observer launch adds the required
CUDA-12 NVRTC library path automatically.

Use this in every terminal before invoking ROS commands:

```bash
source /opt/ros/jazzy/setup.bash
source ~/nav2_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/followbot_ws/install/setup.bash
```

Confirm that the patched Following Server overlay, not the system package, is
selected:

```bash
ros2 pkg prefix opennav_following
# Expected: /home/shaurya/nav2_ws/install/opennav_following
```

## Launch the terrain-aware following stack

Use four terminals, sourcing the overlays above in each one.

Terminal 1 — Gazebo, robot, actor, bridges, ground-truth odometry, TF, and RViz:

```bash
ros2 launch bot_bringup bot_bringup.launch.py world_name:=baylands
```

The Baylands GUI starts after a short delay so the spawned robot is included in
the GUI scene. The robot begins near the Baylands world coordinates
`(-73.090393, -112.385361)`; its ROS `odom` origin is set there.

Terminal 2 — ground segmentation:

```bash
ros2 launch bot_bringup ground_segmentation.launch.py
```

Terminal 3 — Nav2, ground-consistency costmaps, Collision Monitor, and
Following Server:

```bash
ros2 launch bot_bringup terrain_navigation.launch.py
```

Terminal 4 — elevation mapping plus the deterministic FollowBot
traversability observer:

```bash
ros2 launch bot_bringup elevation_mapping.launch.py
```

The elevation mapper publishes `/elevation_mapping_node/terrain_map`. The
FollowBot observer publishes `/followbot_traversability/terrain_map` with
`slope`, `step`, `roughness`, and deterministic `traversability` layers. In
RViz, add a `grid_map_rviz_plugin/GridMap` display for the FollowBot topic and
select each layer while validating. Nav2 consumes the deterministic
`traversability` layer as local graded terrain cost. The learned and
`combined_traversability` layers are diagnostic only.

The initial geometric limits are in
`bot_description/config/followbot_traversability.yaml`: 38 degrees of slope,
0.20 m step height, and 0.08 m roughness. They are deliberately permissive
for the 0.28 m FollowBot wheels and must be revalidated before real-world use.
Ground Consistency uses the same 0.20 m clearance, so it does not turn an
accepted low step into a local-costmap obstacle.
The deterministic step layer is measured relative to a local fitted plane, so
a continuous incline is scored by slope rather than misclassified as a ledge.

## Verify before driving

```bash
ros2 topic hz /points
ros2 topic hz /imu_raw
ros2 topic hz /ground_segmentation/ground_points
ros2 topic hz /ground_segmentation/obstacle_points
ros2 run tf2_ros tf2_echo odom base_link
ros2 lifecycle get /following_server
```

`/following_server` should be `active`. In RViz, set the fixed frame to `odom`
and confirm the ground/obstacle clouds plus the local costmap appear before
testing following.

## Drive the actor and start following

The actor is named `person`. It runs in path mode, and `/person/path` accepts
Gazebo/Baylands **world** coordinates, not local `odom` coordinates. Its
ground-truth pose is converted to local `odom` and published as `/person_pose`
for Nav2.

Send the actor through two waypoints:

```bash
ros2 topic pub --once /person/path geometry_msgs/msg/PoseArray \
"{poses: [
  {position: {x: -67.090393, y: -112.385361, z: 0.0}, orientation: {w: 1.0}},
  {position: {x: -63.090393, y: -109.385361, z: 0.0}, orientation: {w: 1.0}}
]}"
```

Start the Nav2 Following Server in another sourced terminal:

```bash
ros2 action send_goal /follow_object nav2_msgs/action/FollowObject \
"{pose_topic: '/person_pose', tracked_frame: '', max_duration: {sec: 0, nanosec: 0}}" \
--feedback
```

While following, use RViz to inspect:

- `Person Pose (raw)` — `/person_pose` from actor ground truth.
- `Person Pose (filtered)` — `/following_server/filtered_dynamic_pose`.
- `Following trajectory (costmap checked)` — projected local trajectory.
- `Local Costmap` — ground-consistency terrain safety field.

Use RViz's **2D Nav Goal** only for an independent Nav2 goal test; do not send
one while a `/follow_object` action is active.

## What is implemented

- Jazzy/Harmonic Gazebo migration with native Gazebo DiffDrive and ROS bridges.
- 3D LiDAR and IMU, GSeg3D ground/obstacle segmentation, and RViz displays.
- Ground-consistency rolling local costmap and obstacle-based global costmap.
- Ground-truth actor pose feeding upstream `opennav_following`.
- Local terrain-aware following: the Following Server checks its projected
  trajectory against `/local_costmap/costmap_raw`; Collision Monitor remains
  the final braking layer.

## Important current limits

- `map -> odom` is an identity transform for simulation. There is no SLAM or
  localization yet.
- The global costmap is a fixed obstacle layer, not persistent global terrain
  mapping or traversability analysis.
- Terrain safety is local: unsafe terrain makes following stop/retry; it does
  not yet replan a route around it.
- Baylands mesh collision is unreliable with the current DART physics backend.
  Do not use physical wall/terrain collision as a safety result; use the LiDAR
  and costmap evidence.
- The actor is kinematic and is used only as a ground-truth leader. It is not a
  physical person simulation. `/person/cmd_vel` is bridged but path mode is the
  configured actor control interface.
- RGB-D perception, person tracking, SLAM, and global traversability are
  deferred. The local elevation and deterministic traversability layer is the
  current terrain-aware navigation baseline.

## Next work

After validating that the local costmap consistently blocks unsafe terrain,
add global terrain knowledge: first localization/3D mapping, then an elevation
and traversability layer that persists beyond the local rolling map. Perception
can later replace `/person_pose` without changing the following interface.
