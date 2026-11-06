"""Publish robot ground-truth pose and local odometry from Gazebo pose info."""

import math
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class RobotGroundTruthPublisher(Node):
    """Publish Gazebo ground truth as robot pose, odometry, and TF."""

    def __init__(self):
        super().__init__('robot_ground_truth_publisher')
        self.declare_parameter('source_odom_topic', '/odom_gz')
        self.declare_parameter('pose_topic', '/robot_pose_gt')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

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
            Odometry,
            self.get_parameter('source_odom_topic').value,
            self.odom_callback,
            10,
        )
        self.origin_x = self.origin_y = self.origin_yaw = None
        self.get_logger().info(
            f'Listening to Gazebo odometry on "{self.get_parameter("source_odom_topic").value}".')

    def odom_callback(self, msg: Odometry):
        """Republish Gazebo odometry as local odometry, TF, and ground-truth pose."""
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame_id
        pose_msg.pose.position.x = msg.pose.pose.position.x
        pose_msg.pose.position.y = msg.pose.pose.position.y
        pose_msg.pose.position.z = msg.pose.pose.position.z
        pose_msg.pose.orientation = msg.pose.pose.orientation
        self.pose_pub.publish(pose_msg)

        world_x = msg.pose.pose.position.x
        world_y = msg.pose.pose.position.y
        world_yaw = self._yaw_from_rotation(msg.pose.pose.orientation)
        if self.origin_x is None:
            self.origin_x, self.origin_y, self.origin_yaw = world_x, world_y, world_yaw

        dx, dy = world_x - self.origin_x, world_y - self.origin_y
        cos_yaw, sin_yaw = math.cos(-self.origin_yaw), math.sin(-self.origin_yaw)
        local_x = dx * cos_yaw - dy * sin_yaw
        local_y = dx * sin_yaw + dy * cos_yaw
        local_yaw = self._normalize_angle(world_yaw - self.origin_yaw)
        linear_x = linear_y = angular_z = 0.0
        if self.origin_x is not None:
            linear_x = msg.twist.twist.linear.x
            linear_y = msg.twist.twist.linear.y
            angular_z = msg.twist.twist.angular.z

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
