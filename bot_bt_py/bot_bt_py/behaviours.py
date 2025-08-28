#!/usr/bin/env python3
import py_trees
from apriltag_msgs.msg import AprilTagDetectionArray
from .apriltag_follower import AprilTagFollower
from .gps_navigator import GPSNavigator
from std_msgs.msg import Float32


class NodeMetadata:
    def __init__(self, action, context="", preconditions=None):
        self.action = action
        self.context = context
        self.preconditions = preconditions or []
        self.confidence = None
        self.status = None

    def to_dict(self):
        return {
            "action": self.action,
            "context": self.context,
            "preconditions": self.preconditions,
            "confidence": self.confidence,
            "status": self.status,
        }


class FollowCondition(py_trees.behaviour.Behaviour):
    def __init__(self, robot_node):
        super().__init__("Check Follow Mode")
        self.robot_node = robot_node
        self.metadata = NodeMetadata(
            action="Check if follow mode is enabled",
            context="Determines whether the robot should follow a human",
            preconditions=["robot_mode must be set to 'follow'"]
        )

    def update(self):
        if self.robot_node.robot_mode == "follow":
            self.metadata.status = "SUCCESS"
            self.metadata.confidence = 0.9
            self.robot_node.get_logger().info("[FollowCondition] SUCCESS - Following human mode enabled")
            return py_trees.common.Status.SUCCESS

        self.metadata.status = "FAILURE"
        self.metadata.confidence = 0.9
        return py_trees.common.Status.FAILURE


class ReturnCondition(py_trees.behaviour.Behaviour):
    def __init__(self, robot_node):
        super().__init__("Check Return Mode")
        self.robot_node = robot_node
        self.metadata = NodeMetadata(
            action="Check if return mode is enabled",
            context="Determines whether the robot should return home",
            preconditions=["robot_mode must be set to 'return'"]
        )

    def update(self):
        if self.robot_node.robot_mode == "return":
            self.metadata.status = "SUCCESS"
            self.metadata.confidence = 0.9
            self.robot_node.get_logger().info("[ReturnCondition] SUCCESS - return mode enabled")
            return py_trees.common.Status.SUCCESS

        self.metadata.status = "FAILURE"
        self.metadata.confidence = 0.9
        return py_trees.common.Status.FAILURE


class FollowAction(py_trees.behaviour.Behaviour):
    def __init__(self, robot_node):
        super().__init__("Follow Human")
        self.robot_node = robot_node
        self.follower = AprilTagFollower(robot_node.cmd_vel_pub)
        self.latest_msg = None
        self.metadata = NodeMetadata(
            action="Follow human using AprilTag detections",
            context="Activated when follow condition succeeds",
            preconditions=["AprilTag detections must be available"]
        )

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
            self.metadata.status = "RUNNING"
            self.metadata.confidence = 0.5
            return py_trees.common.Status.RUNNING

        status = self.follower.process_detections(self.latest_msg)
        self.metadata.status = str(status)
        self.metadata.confidence = 0.8
        return status
    
    def terminate(self, new_status):
        self.follower.stop()


class ReturnAction(py_trees.behaviour.Behaviour):
    def __init__(self, robot_node):
        super().__init__("Return Home")
        self.robot_node = robot_node
        self.navigator = GPSNavigator(robot_node.cmd_vel_pub)
        self.metadata = NodeMetadata(
            action="Return to base",
            context="Triggered when battery is low or return home command is received",
            preconditions=["GPS fix available", "Start location known"]
        )

    def update(self):
        if self.robot_node.latitude is None or self.robot_node.longitude is None:
            self.metadata.status = "RUNNING"
            self.metadata.confidence = 0.5
            return py_trees.common.Status.RUNNING

        self.robot_node.get_logger().info("[ReturnAction] returning back to base")
        self.navigator.update_pose(self.robot_node.latitude, self.robot_node.longitude, self.robot_node.yaw)
        status = self.navigator.navigate()
        self.metadata.status = str(status)
        self.metadata.confidence = 0.85
        return status

    def terminate(self, new_status):
        self.navigator.stop()


class BatteryLowCondition(py_trees.behaviour.Behaviour):
    def __init__(self, robot_node, threshold=50.0):
        super().__init__("Battery Low?")
        self.robot_node = robot_node
        self.threshold = threshold
        self.battery_level = 100.0
        self.metadata = NodeMetadata(
            action="Check battery level",
            context=f"Triggers failsafe if battery < {threshold}%",
            preconditions=["Battery topic must publish valid Float32 percentage"]
        )

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
            self.metadata.status = "SUCCESS"
            self.metadata.confidence = 0.95
            self.robot_node.get_logger().warn(f"[BatteryLowCondition] Battery low! ({self.battery_level}%)")
            return py_trees.common.Status.SUCCESS

        self.metadata.status = "FAILURE"
        self.metadata.confidence = 0.95
        self.robot_node.get_logger().info(f"[BatteryLowCondition] Battery OK ({self.battery_level}%)")
        return py_trees.common.Status.FAILURE
