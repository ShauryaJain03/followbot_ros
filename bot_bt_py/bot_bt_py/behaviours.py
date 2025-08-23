#!/usr/bin/env python3
import py_trees
from apriltag_msgs.msg import AprilTagDetectionArray
from .apriltag_follower import AprilTagFollower
from .gps_navigator import GPSNavigator


class FollowCondition(py_trees.behaviour.Behaviour):
    def __init__(self, robot_node):
        super().__init__("Check Follow Mode")
        self.robot_node = robot_node

    def update(self):
        if self.robot_node.robot_mode == "follow":
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class ReturnCondition(py_trees.behaviour.Behaviour):
    def __init__(self, robot_node):
        super().__init__("Check Return Mode")
        self.robot_node = robot_node

    def update(self):
        if self.robot_node.robot_mode == "return":
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

        self.navigator.update_pose(self.robot_node.latitude, self.robot_node.longitude, self.robot_node.yaw)
        return self.navigator.navigate()

    def terminate(self, new_status):
        self.navigator.stop()