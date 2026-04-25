"""Publish robot ground truth pose from Gazebo pose info."""

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.node import Node


class RobotGroundTruthPublisher(Node):

    def __init__(self):
        super().__init__('robot_ground_truth_publisher')
        self.declare_parameter('world_pose_topic', '/world/baylands/pose/info')
        self.declare_parameter('robot_entity_name', 'bot')
        self.declare_parameter('pose_topic', '/robot_pose_gt')
        self.declare_parameter('map_frame_id', 'map')

        self.robot_entity_name = self.get_parameter('robot_entity_name').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter('pose_topic').value,
            10,
        )
        self.create_subscription(
            TFMessage,
            self.get_parameter('world_pose_topic').value,
            self.world_pose_callback,
            10,
        )
        self.get_logger().info(
            'Listening to Gazebo pose info for entity '
            f'"{self.robot_entity_name}".'
        )

    def match_robot_transform(self, msg: TFMessage) -> Optional[TransformStamped]:
        """Return the first transform matching the robot entity name."""
        for transform in msg.transforms:
            child_frame = transform.child_frame_id
            if child_frame == self.robot_entity_name:
                return transform
            if child_frame.startswith(self.robot_entity_name + '/'):
                return transform
            if child_frame.startswith(self.robot_entity_name + '::'):
                return transform
            if self.robot_entity_name in child_frame:
                return transform
        return None

    def world_pose_callback(self, msg: TFMessage):
        """Republish the robot ground truth pose and TF from Gazebo."""
        robot_transform = self.match_robot_transform(msg)
        if robot_transform is None:
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = robot_transform.header.stamp
        pose_msg.header.frame_id = self.map_frame_id
        pose_msg.pose.position.x = robot_transform.transform.translation.x
        pose_msg.pose.position.y = robot_transform.transform.translation.y
        pose_msg.pose.position.z = robot_transform.transform.translation.z
        pose_msg.pose.orientation = robot_transform.transform.rotation
        self.pose_pub.publish(pose_msg)


def main(args=None):
    """Run the robot ground truth publisher node."""
    rclpy.init(args=args)
    node = RobotGroundTruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
