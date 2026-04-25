"""Capability-aware follower node scaffold."""

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node


class CapabilityAwareFollower(Node):
    """Follow a human while avoiding infeasible terrain."""

    def __init__(self):
        super().__init__('capability_aware_follower')
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('human_pose_topic', '/human_pose')
        self.declare_parameter('robot_pose_topic', '/robot_pose_gt')
        self.declare_parameter(
            'traversability_grid_topic',
            '/traversability_grid',
        )
        self.declare_parameter('planned_path_topic', '/terrain_follower/path')
        self.declare_parameter(
            'cmd_vel_topic',
            '/bot_controller/cmd_vel_unstamped',
        )
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('target_distance', 3.0)
        self.declare_parameter('max_rejoin_distance', 6.0)

        self.robot_pose = None
        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self.path_pub = self.create_publisher(
            Path,
            self.get_parameter('planned_path_topic').value,
            1,
        )
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
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('traversability_grid_topic').value,
            self.grid_callback,
            1,
        )
        rate = self.get_parameter('control_rate').value
        self.timer = self.create_timer(1.0 / rate, self.control_step)
        self.get_logger().info('Capability-aware follower scaffold ready.')

    def human_callback(self, msg):
        """Receive latest human pose."""
        del msg

    def robot_callback(self, msg):
        """Receive latest robot ground truth pose."""
        self.robot_pose = msg

    def grid_callback(self, msg):
        """Receive latest traversability grid."""
        del msg

    def control_step(self):
        """Plan and track a traversable rejoin path; implementation follows."""


def main(args=None):
    """Run the capability-aware follower node."""
    rclpy.init(args=args)
    node = CapabilityAwareFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
