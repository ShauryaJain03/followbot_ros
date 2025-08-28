#!/usr/bin/env python3
import py_trees
import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from tf_transformations import euler_from_quaternion
from .behaviours import FollowCondition, ReturnCondition, FollowAction, ReturnAction, BatteryLowCondition


class RobotBTNode(Node):
    def __init__(self):
        super().__init__('robot_behavior_tree')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/bot_controller/cmd_vel_unstamped', 10)
        self.expl_pub = self.create_publisher(String, "/bot/metadata", 10)  

        self.robot_mode = "idle"
        self.create_subscription(String, "/bot/mode", self.mode_callback, 10)
        
        self.latitude = None    
        self.longitude = None
        self.yaw = 0.0

        self._last_summary_json = None

        self.create_subscription(NavSatFix, '/navsat', self.navsat_callback, 10)
        self.create_subscription(Imu, '/imu_raw', self.imu_callback, 10)

        self.root = self.create_behavior_tree()
        
        self.timer = self.create_timer(0.5, self.tick_tree)
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

        battery_low = py_trees.composites.Sequence(name="Battery Failsafe", memory=False)
        self.battery_condition = BatteryLowCondition(self, threshold=50.0)
        self.battery_action = ReturnAction(self) 
        battery_low.add_children([self.battery_condition, self.battery_action])

        follow_human = py_trees.composites.Sequence(name="Follow Human", memory=False)
        self.follow_condition = FollowCondition(self)
        self.follow_action = FollowAction(self)
        follow_human.add_children([self.follow_condition, self.follow_action])

        return_home = py_trees.composites.Sequence(name="Return Home", memory=False)
        self.return_condition = ReturnCondition(self)
        self.return_action = ReturnAction(self)
        return_home.add_children([self.return_condition, self.return_action])

        root.add_children([battery_low, follow_human, return_home])
        return root


    def status_name(self, node):
        try:
            return node.status.name 
        except Exception:
            return "UNKNOWN"

    def has_children(self, node):
        return hasattr(node, "children") and node.children

    def node_kind(self, node):
        if isinstance(node, (py_trees.composites.Selector, py_trees.composites.Sequence, py_trees.composites.Parallel)):
            return "composite"
        return "leaf"

    def find_active_path(self, node):
        path = [node]
        if not self.has_children(node):
            return path

        running_child = next((c for c in node.children if self.status_name(c) == "RUNNING"), None)
        if running_child:
            return path + self.find_active_path(running_child)

        success_child = next((c for c in node.children if self.status_name(c) == "SUCCESS"), None)
        if success_child:
            return path + self.find_active_path(success_child)

        return path

    def node_meta_dict(self, node):
        base = {
            "name": node.name,
            "kind": self.node_kind(node),
            "status": self.status_name(node)
        }
        if hasattr(node, "metadata") and node.metadata is not None:
            base["metadata"] = node.metadata.to_dict()
        return base

    def build_current_activity_summary(self):
        active_nodes = self.find_active_path(self.root)
        active_names = [n.name for n in active_nodes]
        current_node = active_nodes[-1] if active_nodes else self.root
        current_meta = self.node_meta_dict(current_node)

        battery_pct = None
        if hasattr(self, "battery_condition") and hasattr(self.battery_condition, "battery_level"):
            battery_pct = float(self.battery_condition.battery_level)

        pose = None
        if self.latitude is not None and self.longitude is not None:
            pose = {
                "lat": float(self.latitude),
                "lon": float(self.longitude),
                "yaw_rad": float(self.yaw)
            }

        stamp = self.get_clock().now().to_msg()
        ros_time = {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}

        summary = {
            "timestamp": ros_time,
            "mode": self.robot_mode,
            "battery_pct": battery_pct,
            "pose": pose,
            "active_path": active_names,
            "current": current_meta
        }
        return summary


    def publish_if_changed(self, summary_dict):
  
        summary_json = json.dumps(summary_dict, sort_keys=True)

        if summary_json != self._last_summary_json:
            self._last_summary_json = summary_json
            self.get_logger().info(f"[BT] Activity change -> {summary_json}")

            msg = String()
            msg.data = summary_json
            self.expl_pub.publish(msg)

    def tick_tree(self):
        try:
            self.root.tick_once()
            summary = self.build_current_activity_summary()
            self.publish_if_changed(summary)

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
