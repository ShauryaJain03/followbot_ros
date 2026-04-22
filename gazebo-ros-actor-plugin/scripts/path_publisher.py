#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, PoseArray, Pose


class PoseArrayPublisher(Node):
    def __init__(self):
        super().__init__('pose_array_publisher_node')
        self.publisher = self.create_publisher(PoseArray, '/cmd_path', 10)
        self.timer = self.create_timer(1.0, self.publish_pose_array)
        self.published = False
        self.get_logger().info('PoseArray publisher node started')

    def publish_pose_array(self):
        if self.published:
            return

        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = 'map'

        num_waypoints = 10
        start_x = -73.090393
        start_y = -108.385361
        waypoint_spacing = 1.0
        heading = 0.0
        quat = self.euler_to_quaternion(0, 0, heading)

        for i in range(num_waypoints):
            pose = Pose()
            pose.position = Point(
                y=start_y + i * waypoint_spacing,
                x=start_x,
                z=0.0)

            pose.orientation = Quaternion(
                x=quat[0],
                y=quat[1],
                z=quat[2],
                w=quat[3])
            
            pose_array_msg.poses.append(pose)
        self.publisher.publish(pose_array_msg)
        self.get_logger().info(f'Published PoseArray with {num_waypoints} poses')

        self.published = True
        self.timer.cancel()
        self.get_logger().info('PoseArray published successfully. Shutting down...')
        self.create_timer(0.1, self.destroy_and_exit)

    def destroy_and_exit(self):
        self.destroy_node()
        raise SystemExit

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy

        return q


def main(args=None):
    rclpy.init(args=args)
    pose_array_publisher = PoseArrayPublisher()
    rclpy.spin(pose_array_publisher)
    pose_array_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
