#!/usr/bin/env python3
"""Publish the scripted actor route to the Gazebo actor plugin."""

import math

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from rclpy.node import Node


def quaternion_from_yaw(yaw):
    """Create a planar quaternion from yaw."""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw * 0.5),
        w=math.cos(yaw * 0.5),
    )


class ActorPathPublisher(Node):
    """Publish the Baylands staircase actor route once."""

    def __init__(self):
        super().__init__('actor_path_publisher')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('cmd_path_topic', '/cmd_path')
        self.declare_parameter('cmd_path_delay', 1.0)
        self.declare_parameter('num_waypoints', 10)
        self.declare_parameter('start_x', -73.090393)
        self.declare_parameter('start_y', -108.385361)
        self.declare_parameter('start_z', 0.0)
        self.declare_parameter('waypoint_spacing', 1.0)
        self.declare_parameter('heading', math.pi / 2.0)

        self.publisher = self.create_publisher(
            PoseArray,
            self.get_parameter('cmd_path_topic').value,
            10,
        )
        self.waypoints = self.build_waypoints()
        self.published = False

        cmd_path_delay = self.get_parameter('cmd_path_delay').value
        self.timer = self.create_timer(
            cmd_path_delay,
            self.publish_cmd_path_once,
        )
        self.get_logger().info(
            f'Actor path publisher ready with {len(self.waypoints)} waypoints.'
        )

    def build_waypoints(self):
        """Build the Baylands staircase demo route."""
        num_waypoints = self.get_parameter('num_waypoints').value
        start_x = self.get_parameter('start_x').value
        start_y = self.get_parameter('start_y').value
        start_z = self.get_parameter('start_z').value
        spacing = self.get_parameter('waypoint_spacing').value
        heading = self.get_parameter('heading').value
        quat = quaternion_from_yaw(heading)

        waypoints = []
        for index in range(num_waypoints):
            pose = Pose()
            pose.position = Point(
                x=start_x,
                y=start_y + index * spacing,
                z=start_z,
            )
            pose.orientation = quat
            waypoints.append(pose)
        return waypoints

    def publish_cmd_path_once(self):
        """Send the route to the Gazebo actor plugin once."""
        if self.published:
            return

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.get_parameter('frame_id').value
        pose_array.poses = list(self.waypoints)
        self.publisher.publish(pose_array)
        self.published = True
        self.get_logger().info(
            f'Published actor /cmd_path with {len(self.waypoints)} poses.'
        )
        self.timer.cancel()
        self.create_timer(0.1, self.shutdown)

    def shutdown(self):
        """Stop this one-shot publisher."""
        self.destroy_node()
        raise SystemExit


def main(args=None):
    """Run the actor path publisher."""
    rclpy.init(args=args)
    node = ActorPathPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
