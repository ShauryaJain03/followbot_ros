"""Reactive baseline follower using the detected human TF."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value: float, limit: float) -> float:
    """Clamp a scalar symmetrically around zero."""
    return max(-limit, min(limit, value))


class NaiveFollower(Node):
    """Drive toward the detected human with simple proportional control."""

    def __init__(self) -> None:
        super().__init__('naive_follower')

        self.declare_parameter('cmd_vel_topic', '/bot_controller/cmd_vel_unstamped')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('target_frame', 'detected_human')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('follow_distance', 2.5)
        self.declare_parameter('distance_tolerance', 0.3)
        self.declare_parameter('target_timeout', 0.75)
        self.declare_parameter('kp_linear', 0.35)
        self.declare_parameter('kp_angular', 1.8)
        self.declare_parameter('max_linear_speed', 0.50)
        self.declare_parameter('max_angular_speed', 0.7)
        self.declare_parameter('max_heading_for_forward', 1.0)

        self.base_frame = self.get_parameter('base_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.distance_tolerance = float(self.get_parameter('distance_tolerance').value)
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.max_heading_for_forward = float(
            self.get_parameter('max_heading_for_forward').value
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        control_rate = float(self.get_parameter('control_rate').value)
        self.timer = self.create_timer(1.0 / control_rate, self.control_step)

        self.get_logger().info(
            f'Naive follower tracking {self.target_frame} from {self.base_frame} '
            f'at {control_rate:.1f} Hz.'
        )

    def publish_stop(self) -> None:
        """Publish a zero velocity command."""
        self.cmd_pub.publish(Twist())

    def target_is_stale(self, stamp) -> bool:
        """Return True if the transform timestamp is too old."""
        if stamp.sec == 0 and stamp.nanosec == 0:
            return False
        age = self.get_clock().now() - rclpy.time.Time.from_msg(stamp)
        return age.nanoseconds > int(self.target_timeout * 1e9)

    def control_step(self) -> None:
        """Look up the human transform and command a simple reactive follow."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'No transform {self.base_frame} -> {self.target_frame}: {ex}',
                throttle_duration_sec=2.0,
            )
            self.publish_stop()
            return

        if self.target_is_stale(transform.header.stamp):
            self.get_logger().warn(
                f'Stale transform for {self.target_frame}; stopping.',
                throttle_duration_sec=2.0,
            )
            self.publish_stop()
            return

        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)

        heading_error = math.atan2(ty, tx)
        forward_error = tx - self.follow_distance

        cmd = Twist()
        cmd.angular.z = clamp(self.kp_angular * heading_error, self.max_angular_speed)

        target_in_front = tx > 0.0
        close_enough = forward_error <= self.distance_tolerance
        heading_ok = abs(heading_error) < self.max_heading_for_forward

        if target_in_front and not close_enough and heading_ok:
            cmd.linear.x = clamp(self.kp_linear * forward_error, self.max_linear_speed)

        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    """Run the naive follower node."""
    rclpy.init(args=args)
    node = NaiveFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
