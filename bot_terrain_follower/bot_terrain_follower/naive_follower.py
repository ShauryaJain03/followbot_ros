"""Direct human follower used as the failing baseline."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node


def clamp(value, low, high):
    """Limit value to the inclusive range [low, high]."""
    return max(low, min(high, value))


def yaw_from_quaternion(q):
    """Return planar yaw from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class NaiveFollower(Node):
    """Drive straight toward the human pose with no terrain awareness."""

    def __init__(self):
        super().__init__('naive_follower')
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('human_pose_topic', '/human_pose')
        self.declare_parameter('robot_pose_topic', '/robot_pose_gt')
        self.declare_parameter(
            'cmd_vel_topic',
            '/bot_controller/cmd_vel_unstamped',
        )
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('target_distance', 2.5)
        self.declare_parameter('kp_linear', 0.6)
        self.declare_parameter('kp_angular', 1.8)
        self.declare_parameter('max_linear_speed', 0.7)
        self.declare_parameter('max_angular_speed', 1.2)

        self.human_pose = None
        self.robot_pose = None
        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter('robot_pose_topic').value,
            self.robot_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter('human_pose_topic').value,
            self.human_callback,
            10,
        )
        rate = self.get_parameter('control_rate').value
        self.timer = self.create_timer(1.0 / rate, self.control_step)

    def human_callback(self, msg):
        """Store the latest human pose."""
        self.human_pose = msg

    def robot_callback(self, msg):
        """Store the latest robot ground truth pose."""
        self.robot_pose = msg

    def control_step(self):
        """Publish direct-follow velocity commands."""
        if self.human_pose is None or self.robot_pose is None:
            return

        robot_pose = self.robot_pose.pose
        dx = self.human_pose.pose.position.x - robot_pose.position.x
        dy = self.human_pose.pose.position.y - robot_pose.position.y
        distance = math.hypot(dx, dy)
        target_distance = self.get_parameter('target_distance').value

        desired_heading = math.atan2(dy, dx)
        robot_yaw = yaw_from_quaternion(robot_pose.orientation)
        heading_error = normalize_angle(desired_heading - robot_yaw)

        cmd = Twist()
        linear_error = self.get_parameter('kp_linear').value
        linear_error *= distance - target_distance
        cmd.linear.x = clamp(
            linear_error,
            0.0,
            self.get_parameter('max_linear_speed').value,
        )
        cmd.angular.z = clamp(
            self.get_parameter('kp_angular').value * heading_error,
            -self.get_parameter('max_angular_speed').value,
            self.get_parameter('max_angular_speed').value,
        )
        self.cmd_pub.publish(cmd)


def main(args=None):
    """Run the naive follower node."""
    rclpy.init(args=args)
    node = NaiveFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
