"""Convert the Gazebo actor pose into Follow Server's local pose interface."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class PersonGroundTruthPublisher(Node):
    """Publish the simulated actor's ground-truth pose in the odom frame."""

    def __init__(self):
        super().__init__('person_ground_truth_publisher')
        self.declare_parameter('source_pose_topic', '/person/ground_truth')
        self.declare_parameter('pose_topic', '/person_pose')
        self.declare_parameter('world_origin_x', -222.0)
        self.declare_parameter('world_origin_y', -68.0)
        self.declare_parameter('world_origin_yaw', 0.0)

        self.origin_x = self.get_parameter('world_origin_x').value
        self.origin_y = self.get_parameter('world_origin_y').value
        self.origin_yaw = self.get_parameter('world_origin_yaw').value
        self.pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter('pose_topic').value, 10)
        self.create_subscription(
            PoseStamped,
            self.get_parameter('source_pose_topic').value,
            self.pose_callback,
            10,
        )
        self.get_logger().info(
            'Publishing actor ground truth on '
            f'"{self.get_parameter("pose_topic").value}".')

    def pose_callback(self, msg: PoseStamped):
        """Publish the actor transform in the robot's local odometry frame."""
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        if pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0:
            pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x, pose.pose.position.y, yaw = self._to_odom(
            msg.pose.position.x,
            msg.pose.position.y,
            self._yaw_from_rotation(msg.pose.orientation),
            self.origin_x,
            self.origin_y,
            self.origin_yaw,
        )
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.pose_pub.publish(pose)

    @staticmethod
    def _to_odom(world_x, world_y, world_yaw, origin_x, origin_y, origin_yaw):
        """Express a world pose in the local frame created at robot spawn."""
        dx, dy = world_x - origin_x, world_y - origin_y
        cos_yaw, sin_yaw = math.cos(-origin_yaw), math.sin(-origin_yaw)
        return (
            dx * cos_yaw - dy * sin_yaw,
            dx * sin_yaw + dy * cos_yaw,
            world_yaw - origin_yaw,
        )

    @staticmethod
    def _yaw_from_rotation(rotation):
        return math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )


def main(args=None):
    """Run the actor ground-truth publisher."""
    rclpy.init(args=args)
    node = PersonGroundTruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
