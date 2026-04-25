"""Log baseline and capability-aware following metrics."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class DemoMetricsLogger(Node):
    """Track human distance metrics during a demo run."""

    def __init__(self):
        super().__init__('demo_metrics_logger')
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('human_pose_topic', '/human_pose')
        self.declare_parameter('robot_pose_topic', '/robot_pose_gt')
        self.declare_parameter('report_period', 2.0)
        self.human_pose = None
        self.robot_pose = None
        self.max_distance = 0.0
        self.create_subscription(
            PoseStamped,
            self.get_parameter('human_pose_topic').value,
            self.human_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter('robot_pose_topic').value,
            self.robot_callback,
            10,
        )
        self.timer = self.create_timer(
            self.get_parameter('report_period').value,
            self.report,
        )

    def human_callback(self, msg):
        """Store latest human pose."""
        self.human_pose = msg

    def robot_callback(self, msg):
        """Store latest robot ground truth pose."""
        self.robot_pose = msg

    def report(self):
        """Report current and maximum human-following distance."""
        if self.human_pose is None or self.robot_pose is None:
            return
        robot_position = self.robot_pose.pose.position
        human_position = self.human_pose.pose.position
        distance = math.hypot(
            human_position.x - robot_position.x,
            human_position.y - robot_position.y,
        )
        self.max_distance = max(self.max_distance, distance)
        message = (
            f'human_distance={distance:.2f} m '
            f'max_distance={self.max_distance:.2f} m'
        )
        self.get_logger().info(message)


def main(args=None):
    """Run the demo metrics logger node."""
    rclpy.init(args=args)
    node = DemoMetricsLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
