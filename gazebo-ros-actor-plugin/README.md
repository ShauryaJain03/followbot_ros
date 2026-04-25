# Gazebo ROS 2 Actor Plugin

`gazebo_ros_actor_plugin` is a ROS 2 package for controlling Gazebo actors from ROS topics. It supports:

- velocity control from `/cmd_vel`
- waypoint following from `/cmd_path`
- follower-facing actor route and actual actor pose publishing on `/human_path` and `/human_pose`
- optional terrain-aware height tracking over mesh terrain

The package includes a flat-ground sample world and a Baylands example world.

## Targets

- ROS 2 Humble
- Gazebo Fortress (`gz-sim6`)
- Ubuntu 22.04

## Features

- Actor control through `geometry_msgs/msg/Twist`
- Actor waypoint following through `geometry_msgs/msg/PoseArray`
- Launch file for Gazebo plus ROS/Gazebo bridges
- Optional terrain-aware actor height support using explicit terrain meshes
- Bundled actor skins and example environment assets

## Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/blackcoffeerobotics/gazebo-ros-actor-plugin.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select gazebo_ros_actor_plugin
source install/setup.bash
```

## Launch

Start the default sample world:

```bash
ros2 launch gazebo_ros_actor_plugin sim.launch.py
```

Start the Baylands example world:

```bash
ros2 launch gazebo_ros_actor_plugin sim.launch.py \
  world:=$(ros2 pkg prefix gazebo_ros_actor_plugin)/share/gazebo_ros_actor_plugin/config/worlds/baylands.world
```

Launch arguments:

- `world`: absolute path to the world file to load
- `verbose`: `True` or `False`
- `headless`: `True` or `False`

## Control

Velocity control:

```bash
sudo apt-get install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Path following:

```bash
ros2 run gazebo_ros_actor_plugin path_publisher.py
```

The bundled path publisher sends a straight 10-waypoint path in the Baylands
example area. The actor plugin publishes follower-facing state from the actual
Gazebo actor:

- `/human_path` (`nav_msgs/msg/Path`): full actor route received by the plugin.
- `/human_pose` (`geometry_msgs/msg/PoseStamped`): current actor pose from Gazebo.

When `bot_bringup` is launched with `world_name:=baylands`, this publisher is
started automatically after a short delay.

## Terrain-Aware Height

Terrain tracking is opt-in. Add one or more `<terrain>` entries inside the actor plugin block:

```xml
<plugin
  filename="libgazebo_ros_actor_plugin.so"
  name="gazebo_ros_actor_plugin::GazeboRosActorCommand">
  <follow_mode>path</follow_mode>
  <vel_topic>/cmd_vel</vel_topic>
  <path_topic>/cmd_path</path_topic>
  <terrain>
    <mesh>model://my_world/media/terrain_01.dae</mesh>
    <pose>0 0 0 0 0 0</pose>
  </terrain>
</plugin>
```

Notes:

- The plugin does not auto-discover terrain from the world file.
- Each terrain mesh must be declared explicitly.
- Meshes must load through Gazebo's mesh loader and expose triangle geometry.
- If no terrain triangle is found at the actor's current `x/y`, the actor falls back to its initial `z`.

## Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `follow_mode` | `velocity` | `velocity` or `path` |
| `vel_topic` | `/cmd_vel` | Velocity command topic |
| `path_topic` | `/cmd_path` | Waypoint topic |
| `animation_factor` | `4.0` | Animation speed multiplier |
| `linear_tolerance` | `0.1` | Waypoint arrival threshold |
| `linear_velocity` | `1.0` | Movement speed in m/s |
| `angular_tolerance` | `0.0872` | Angular tolerance in rad |
| `angular_velocity` | `0.1745` | Angular speed in rad/s |
| `default_rotation` | `1.57` | Skin alignment offset in rad |

## ROS Interface

Subscribed topics:

- `/cmd_vel` with type `geometry_msgs/msg/Twist`
- `/cmd_path` with type `geometry_msgs/msg/PoseArray`

Published topics from `path_publisher.py`:

- `/cmd_path` with type `geometry_msgs/msg/PoseArray`

Published topics from the actor plugin:

- `/human_path` with type `nav_msgs/msg/Path`
- `/human_pose` with type `geometry_msgs/msg/PoseStamped`

## Assets

- Actor skin assets are under `config/skins`
- Example environment assets are under `config/models`

## License

Apache-2.0
