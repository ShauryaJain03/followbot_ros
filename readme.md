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
- Self-contained 2.5D LiDAR traversability mapper; no third-party terrain-map
  package is required.
- Per-cell terrain classification from slope, step height, roughness, and
  vertical obstacle extent.
- RViz terrain visualization and planner-facing OccupancyGrid outputs.
- Gazebo ground-truth odometry for simulation validation, separated from the
  LIO-SAM TF configuration.
- Baseline direct human follower for later comparison.

The active terrain mapper lives in `bot_terrain_follower`. It publishes:

| Topic | Frame | Purpose |
|---|---|---|
| `/terrain/local_grid` | `base_link` | Rolling terrain grid from the latest LiDAR cloud |
| `/terrain/global_grid` | `map` | Terrain grid accumulated from registered clouds |
| `/terrain/costmap` | `map` | Planner-facing accumulated terrain costs |
| `/terrain/markers` | local/map | RViz terrain classes: green free, yellow/orange costly, red lethal |

The global grid requires a valid `map <- laser_link` transform, normally from
LIO-SAM. The local grid is available from raw LiDAR alone.

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

### Terrain-map visualization

The terrain demo includes robot bringup and RViz; do not start
`bot_bringup.launch.py` separately.

```bash
ros2 launch bot_terrain_follower terrain_mapping_demo.launch.py \
  world_name:=rubicon
```

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

1. Tune and validate terrain thresholds against recorded Rubicon/Baylands
   LiDAR data.
2. Run LIO-SAM with a stable TF tree and validate `/terrain/global_grid`.
3. Feed `/terrain/costmap` into Nav2 Smac Hybrid-A* and prove a terrain-safe
   detour before adding dynamic human-follow goals.

## Main packages

- `bot_description`: robot model, worlds, Gazebo bridges, and point-cloud
  preprocessing.
- `bot_controller`: ros2_control differential-drive configuration.
- `bot_bringup`: simulation, clock, TF, controller, and odometry launch.
- `bot_terrain_follower`: terrain mapper, visualization, baseline follower,
  simulation ground-truth publisher, and metrics.
- `human_detector`: RGB-D human detection and `detected_human` TF.
- `LIO-SAM`: LiDAR-inertial odometry and mapping integration.
