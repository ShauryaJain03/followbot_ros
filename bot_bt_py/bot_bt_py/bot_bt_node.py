#!/usr/bin/env python3
import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from tf_transformations import euler_from_quaternion
from .behaviours import FollowCondition, ReturnCondition, FollowAction, ReturnAction


class RobotBTNode(Node):
    def __init__(self):
        super().__init__('robot_behavior_tree')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/bot_controller/cmd_vel_unstamped', 10)

        self.robot_mode = "idle"
        self.create_subscription(String, "/bot/mode", self.mode_callback, 10)

        self.latitude = None
        self.longitude = None
        self.yaw = 0.0

        self.create_subscription(NavSatFix, '/navsat', self.navsat_callback, 10)
        self.create_subscription(Imu, '/imu_raw', self.imu_callback, 10)

        self.root = self.create_behavior_tree()
        
        self.timer = self.create_timer(0.1, self.tick_tree)
        self.get_logger().info("Robot Behavior Tree Node Started!")

    def mode_callback(self, msg: String):
        self.robot_mode = msg.data.lower()

    def navsat_callback(self, msg: NavSatFix):
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def imu_callback(self, msg: Imu):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)

    def create_behavior_tree(self):
        root = py_trees.composites.Selector(name="Main Behavior", memory=False)

        follow_human = py_trees.composites.Sequence(name="Follow Human", memory=False)
        return_home = py_trees.composites.Sequence(name="Return Home", memory=False)

        follow_condition = FollowCondition(self)
        follow_action = FollowAction(self)

        return_condition = ReturnCondition(self)
        return_action = ReturnAction(self)

        follow_human.add_children([follow_condition, follow_action])
        return_home.add_children([return_condition, return_action])

        root.add_children([follow_human, return_home])
        return root

    def tick_tree(self):
        try:
            self.root.tick_once()
        except Exception as e:
            self.get_logger().error(f"Error ticking tree: {str(e)}")

    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        robot_bt_node = RobotBTNode()
        rclpy.spin(robot_bt_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'robot_bt_node' in locals():
            robot_bt_node.publish_velocity(0.0, 0.0)
            robot_bt_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()