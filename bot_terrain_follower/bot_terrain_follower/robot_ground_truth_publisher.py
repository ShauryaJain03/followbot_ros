"""Publish robot ground truth pose and local odometry from Gazebo pose info."""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
from rclpy.node import Node


class RobotGroundTruthPublisher(Node):

    def __init__(self):
        super().__init__('robot_ground_truth_publisher')
        self.declare_parameter('world_pose_topic', '/world/baylands/pose/info')
        self.declare_parameter('robot_entity_name', 'bot')
        self.declare_parameter('pose_topic', '/robot_pose_gt')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.robot_entity_name = self.get_parameter('robot_entity_name').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter('pose_topic').value,
            10,
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('odom_topic').value,
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(
            TFMessage,
            self.get_parameter('world_pose_topic').value,
            self.world_pose_callback,
            10,
        )
        self.origin_x = None
        self.origin_y = None
        self.origin_yaw = None
        self.last_local_x = None
        self.last_local_y = None
        self.last_local_yaw = None
        self.last_time = None
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
        """Republish the robot ground truth pose and local odometry from Gazebo."""
        robot_transform = self.match_robot_transform(msg)
        if robot_transform is None:
            return

        stamp = robot_transform.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame_id
        pose_msg.pose.position.x = robot_transform.transform.translation.x
        pose_msg.pose.position.y = robot_transform.transform.translation.y
        pose_msg.pose.position.z = robot_transform.transform.translation.z
        pose_msg.pose.orientation = robot_transform.transform.rotation
        self.pose_pub.publish(pose_msg)

        world_x = robot_transform.transform.translation.x
        world_y = robot_transform.transform.translation.y
        world_yaw = self._yaw_from_rotation(robot_transform.transform.rotation)

        if self.origin_x is None:
            self.origin_x = world_x
            self.origin_y = world_y
            self.origin_yaw = world_yaw

        dx = world_x - self.origin_x
        dy = world_y - self.origin_y
        cos_yaw = math.cos(-self.origin_yaw)
        sin_yaw = math.sin(-self.origin_yaw)
        local_x = dx * cos_yaw - dy * sin_yaw
        local_y = dx * sin_yaw + dy * cos_yaw
        local_yaw = self._normalize_angle(world_yaw - self.origin_yaw)
        qz = math.sin(local_yaw * 0.5)
        qw = math.cos(local_yaw * 0.5)

        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        current_time = self._stamp_to_seconds(stamp)
        if self.last_time is not None and current_time > self.last_time:
            dt = current_time - self.last_time
            if dt > 0.0:
                linear_x = (local_x - self.last_local_x) / dt
                linear_y = (local_y - self.last_local_y) / dt
                angular_z = self._normalize_angle(local_yaw - self.last_local_yaw) / dt

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        odom_msg.pose.pose.position.x = local_x
        odom_msg.pose.pose.position.y = local_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = linear_x
        odom_msg.twist.twist.linear.y = linear_y
        odom_msg.twist.twist.angular.z = angular_z
        self.odom_pub.publish(odom_msg)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = local_x
            tf_msg.transform.translation.y = local_y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

        self.last_local_x = local_x
        self.last_local_y = local_y
        self.last_local_yaw = local_yaw
        self.last_time = current_time

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _yaw_from_rotation(rotation) -> float:
        siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


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
