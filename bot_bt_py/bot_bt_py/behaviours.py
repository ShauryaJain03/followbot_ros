#!/usr/bin/env python3
import py_trees
from apriltag_msgs.msg import AprilTagDetectionArray
from .apriltag_follower import AprilTagFollower
from .gps_navigator import GPSNavigator
from std_msgs.msg import Float32


class FollowCondition(py_trees.behaviour.Behaviour):
    def __init__(self, robot_node):
        super().__init__("Check Follow Mode")
        self.robot_node = robot_node

    def update(self):
        if self.robot_node.robot_mode == "follow":
            self.robot_node.get_logger().info("[FollowCondition] SUCCESS - Following human mode enabled")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class ReturnCondition(py_trees.behaviour.Behaviour):
    def __init__(self, robot_node):
        super().__init__("Check Return Mode")
        self.robot_node = robot_node

    def update(self):
        if self.robot_node.robot_mode == "return":
            self.robot_node.get_logger().info("[ReturnCondition] SUCCESS - return mode enabled")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class FollowAction(py_trees.behaviour.Behaviour):
    def __init__(self, robot_node):
        super().__init__("Follow Human")
        self.robot_node = robot_node
        self.follower = AprilTagFollower(robot_node.cmd_vel_pub)
        self.latest_msg = None

        self.robot_node.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        self.latest_msg = msg

    def update(self):
        if self.latest_msg is None:
            return py_trees.common.Status.RUNNING
        return self.follower.process_detections(self.latest_msg)
    
    def terminate(self, new_status):
        self.follower.stop()


class ReturnAction(py_trees.behaviour.Behaviour):
    def __init__(self, robot_node):
        super().__init__("Return Home")
        self.robot_node = robot_node
        self.navigator = GPSNavigator(robot_node.cmd_vel_pub)

    def update(self):
        if self.robot_node.latitude is None or self.robot_node.longitude is None:
            return py_trees.common.Status.RUNNING

        self.robot_node.get_logger().info("[ReturnAction] returning back to base")
        self.navigator.update_pose(self.robot_node.latitude, self.robot_node.longitude, self.robot_node.yaw)
        return self.navigator.navigate()

    def terminate(self, new_status):
        self.navigator.stop()


class BatteryLowCondition(py_trees.behaviour.Behaviour):
    def __init__(self, robot_node, threshold=80.0):
        super().__init__("Battery Low?")
        self.robot_node = robot_node
        self.threshold = threshold
        self.battery_level = 100.0

        self.sub = self.robot_node.create_subscription(
            Float32,
            "/bot/battery_status",
            self.battery_callback,
            10
        )

    def battery_callback(self, msg):
        self.battery_level = msg.data

    def update(self):
        if self.battery_level <= self.threshold:
            self.robot_node.get_logger().warn(f"[BatteryLowCondition] Battery low! ({self.battery_level}%)")
            return py_trees.common.Status.SUCCESS
        self.robot_node.get_logger().info(f"[BatteryLowCondition] Battery OK ({self.battery_level}%)")
        return py_trees.common.Status.FAILURE