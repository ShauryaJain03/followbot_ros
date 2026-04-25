"""Publish a simple human pose target for early follower integration tests."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class HumanPosePublisher(Node):
    """Publish a deterministic human pose for isolated tests."""

    def __init__(self):
        super().__init__('human_pose_publisher')
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('human_pose_topic', '/human_pose')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('x', 3.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)

        topic = self.get_parameter('human_pose_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        self.publisher = self.create_publisher(PoseStamped, topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_pose)

    def publish_pose(self):
        """Publish the configured human pose."""
        yaw = self.get_parameter('yaw').value
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.get_parameter('frame_id').value
        pose.pose.position.x = self.get_parameter('x').value
        pose.pose.position.y = self.get_parameter('y').value
        pose.pose.position.z = self.get_parameter('z').value
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.publisher.publish(pose)


def main(args=None):
    """Run the human pose publisher node."""
    rclpy.init(args=args)
    node = HumanPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
