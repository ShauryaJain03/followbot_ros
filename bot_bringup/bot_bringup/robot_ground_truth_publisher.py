"""Publish robot ground-truth pose and local odometry from Gazebo pose info."""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


class RobotGroundTruthPublisher(Node):
    """Publish Gazebo ground truth as robot pose, odometry, and TF."""

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
            PoseStamped, self.get_parameter('pose_topic').value, 10)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(
            TFMessage,
            self.get_parameter('world_pose_topic').value,
            self.world_pose_callback,
            10,
        )
        self.origin_x = self.origin_y = self.origin_yaw = None
        self.last_local_x = self.last_local_y = self.last_local_yaw = None
        self.last_time = None
        self.get_logger().info(
            f'Listening to Gazebo pose info for entity "{self.robot_entity_name}".')

    def match_robot_transform(self, msg: TFMessage) -> Optional[TransformStamped]:
        """Return the first transform matching the robot entity name."""
        for transform in msg.transforms:
            child_frame = transform.child_frame_id
            if (child_frame == self.robot_entity_name or
                    child_frame.startswith(self.robot_entity_name + '/') or
                    child_frame.startswith(self.robot_entity_name + '::') or
                    self.robot_entity_name in child_frame):
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
            self.origin_x, self.origin_y, self.origin_yaw = world_x, world_y, world_yaw

        dx, dy = world_x - self.origin_x, world_y - self.origin_y
        cos_yaw, sin_yaw = math.cos(-self.origin_yaw), math.sin(-self.origin_yaw)
        local_x = dx * cos_yaw - dy * sin_yaw
        local_y = dx * sin_yaw + dy * cos_yaw
        local_yaw = self._normalize_angle(world_yaw - self.origin_yaw)
        linear_x = linear_y = angular_z = 0.0
        current_time = self._stamp_to_seconds(stamp)
        if self.last_time is not None and current_time > self.last_time:
            dt = current_time - self.last_time
            linear_x, linear_y = self._velocity_in_base_frame(
                (local_x - self.last_local_x) / dt,
                (local_y - self.last_local_y) / dt,
                local_yaw,
            )
            angular_z = self._normalize_angle(local_yaw - self.last_local_yaw) / dt

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y = local_x, local_y
        odom_msg.pose.pose.orientation.z = math.sin(local_yaw * 0.5)
        odom_msg.pose.pose.orientation.w = math.cos(local_yaw * 0.5)
        odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y = linear_x, linear_y
        odom_msg.twist.twist.angular.z = angular_z
        self.odom_pub.publish(odom_msg)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id, tf_msg.child_frame_id = (
                self.odom_frame_id, self.base_frame_id)
            tf_msg.transform.translation.x, tf_msg.transform.translation.y = local_x, local_y
            tf_msg.transform.rotation.z = math.sin(local_yaw * 0.5)
            tf_msg.transform.rotation.w = math.cos(local_yaw * 0.5)
            self.tf_broadcaster.sendTransform(tf_msg)

        self.last_local_x, self.last_local_y, self.last_local_yaw = (
            local_x, local_y, local_yaw)
        self.last_time = current_time

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _yaw_from_rotation(rotation) -> float:
        return math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _velocity_in_base_frame(
        linear_x: float, linear_y: float, yaw: float) -> tuple[float, float]:
        """Express an odom-frame velocity in the robot base frame."""
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        return (
            cos_yaw * linear_x + sin_yaw * linear_y,
            -sin_yaw * linear_x + cos_yaw * linear_y,
        )


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
