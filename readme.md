# FollowBot ROS

ROS 2 research workspace for a four-wheel differential-drive robot that follows
a person in rough terrain without blindly reproducing the person's path.

## V1 goal

A person can traverse terrain that a wheeled robot cannot. V1 therefore treats
the detected person as evidence of navigational intent, selects a safe
human-relative goal, and plans the robot's own feasible route.

The intended navigation objective is:

\[
J = w_f J_f + w_t J_t + w_o J_o
\]

where \(J_f\) maintains the human--robot relationship, \(J_t\) penalizes or
rejects terrain outside robot capability, and \(J_o\) handles collision and
clearance costs.

```text
RGB-D camera ──> human detection/localization ──> human-relative follow goal
                                                        │
3D LiDAR + IMU ──> LIO-SAM ──> terrain costmap ────────┤
                                                        ▼
                                             Smac Hybrid-A* / Nav2
                                                        ▼
                                            local controller -> robot
```

V1 is deliberately a conservative, explainable system. Unknown terrain is not
considered traversable; a feasible position near the person is preferable to
an infeasible goal at their exact position.

## Current implementation

- Gazebo Fortress simulation with the FollowBot URDF, 3D LiDAR, RGB-D camera,
  IMU, ros2_control, and Baylands/Rubicon worlds.
- Stable simulation clock and joint-state topology: ros2_control is the single
  wheel-joint publisher.
- `traversability_mapping` + `grid_map` integration for layered local/global
  terrain inference beneath the project planner.
- Per-cell terrain hazard from slope, step height, and roughness, with RViz
  GridMap and OccupancyGrid visualization.
- Gazebo ground-truth odometry for simulation validation, separated from the
  LIO-SAM TF configuration.
- Baseline direct human follower for later comparison.

The active terrain front end lives in `bot_terrain_follower`; its LIO-SAM
bridge publishes keyframes without publishing TF. The mapper publishes:

| Topic | Frame | Purpose |
|---|---|---|
| `/global_traversability_gridmap` | `map` | Persistent layered elevation and hazard map |
| `/global_traversability_occupancy` | `map` | Global hazard cost map; unknown remains unknown |
| `/local_traversability_gridmap` | `map` | Rolling layered terrain map near the robot |
| `/local_traversability_occupancy` | `map` | Rolling terrain hazard cost map |

Both map products require LIO-SAM's valid `map <- odom <- base_link` transform.

### External traversability-mapping experiment

The project can also evaluate the GPL-3.0 `traversability_mapping` library. It
uses `grid_map` for layered elevation/traversability grids, while FollowBot
retains the human-relative goal manager and planner. The LIO adapter publishes
only keyframe messages; it never publishes TF.

```bash
cd ~/followbot_ws/src/followbot_ros
mkdir -p third_party
git clone https://github.com/suchetanrs/traversability_mapping.git third_party/traversability_mapping
cd ~/followbot_ws
rosdep install --from-paths src/followbot_ros/third_party/traversability_mapping --ignore-src -r -y
colcon build --base-paths src/followbot_ros/third_party/traversability_mapping \
  --packages-up-to traversability_mapping_ros ground_truth_kfs traversability_grid_utils \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select bot_terrain_follower
source install/setup.bash
```

With the LIO-safe Rubicon simulation and LIO-SAM already running:

```bash
ros2 launch bot_terrain_follower lio_traversability_mapping.launch.py
```

The library publishes `/global_traversability_gridmap`,
`/global_traversability_occupancy`, `/local_traversability_gridmap`, and
`/local_traversability_occupancy`.

## Build

From this workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Simulation launch modes

Use one simulation launch per ROS domain. Every launch owns a Gazebo `/clock`
bridge; running two simulators together causes time jumps and invalid TF.

### Ground-truth odometry validation

Use this mode to validate robot motion in Gazebo. Ground truth publishes
`/odom` and owns `odom -> base_link`.

```bash
ros2 launch bot_bringup bot_bringup.launch.py \
  world_name:=baylands use_rviz:=false
```

For this mode, use `/odom` in RViz. `/bot_controller/odom` is wheel-integrated
odometry and will drift on rough terrain or skid-steer turns.

### Terrain-map visualization with LIO-SAM

Start the LIO-safe simulation mode below, then LIO-SAM in a second terminal.
In a third terminal, start the traversability mapper:

```bash
ros2 launch bot_terrain_follower lio_traversability_mapping.launch.py
```

In RViz, set the fixed frame to `map`. For a traversability view, set a GridMap
display's Color Layer to `hazard`, Height Layer to `elevation`, and intensity
range to 0--1. The default elevation colors represent height, not driveability.

Drive the robot with:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/bot_controller/cmd_vel_unstamped
```

### LIO-SAM mode

LIO-SAM must be the sole owner of `odom -> base_link`. Keep the controller TF
disabled and suppress the Gazebo ground-truth odometry publisher:

```bash
ros2 launch bot_bringup bot_bringup.launch.py \
  world_name:=baylands \
  use_ground_truth_odom:=true \
  publish_ground_truth_odom:=false \
  use_rviz:=false
```

Then start LIO-SAM in another terminal using the Baylands parameter file.
The expected tree is:

```text
map -> odom -> base_link -> laser_link
       LIO-SAM
```

## Immediate V1 work

1. Tune and validate terrain-hazard thresholds against recorded Rubicon/Baylands
   LiDAR data.
2. Convert hazard and unknown terrain to a custom planner costmap with the full
   robot footprint.
3. Integrate the costmap into the custom Hybrid-A* objective and prove a
   terrain-safe detour before adding dynamic human-follow goals.

## Main packages

- `bot_description`: robot model, worlds, Gazebo bridges, and point-cloud
  preprocessing.
- `bot_controller`: ros2_control differential-drive configuration.
- `bot_bringup`: simulation, clock, TF, controller, and odometry launch.
- `bot_terrain_follower`: terrain mapper, visualization, baseline follower,
  simulation ground-truth publisher, and metrics.
- `human_detector`: RGB-D human detection and `detected_human` TF.
- `LIO-SAM`: LiDAR-inertial odometry and mapping integration.
