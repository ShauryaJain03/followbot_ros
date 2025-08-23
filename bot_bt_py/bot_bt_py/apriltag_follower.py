#!/usr/bin/env python3
import time
import py_trees
from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray
from .pid_controller import PIDController


class AprilTagFollower:
    def __init__(self, cmd_vel_pub, target_tag_id=0, target_area=22000):
        self.cmd_vel_pub = cmd_vel_pub
        self.target_tag_id = target_tag_id
        self.target_area = target_area

        self.image_center_x = 320  

        self.angular_pid = PIDController(0.005, 0.0001, 0.01, output_limits=(-0.8, 0.8))
        self.linear_pid = PIDController(0.00008, 0.000001, 0.0001, output_limits=(-0.8, 0.8))

        self.angular_deadzone = 30
        self.area_deadzone = 1000
        self.min_linear_speed = 0.1
        self.max_no_detection_time = 2.0
        self.last_detection_time = time.time()

    def process_detections(self, msg: AprilTagDetectionArray):
        current_time = time.time()

        if not msg.detections:
            if current_time - self.last_detection_time > self.max_no_detection_time:
                self.stop()
            return py_trees.common.Status.FAILURE  

        target_tag = None
        for detection in msg.detections:
            if detection.id == self.target_tag_id:
                target_tag = detection
                break

        if target_tag is None:
            if current_time - self.last_detection_time > self.max_no_detection_time:
                self.stop()
            return py_trees.common.Status.FAILURE

        self.last_detection_time = current_time

        current_area = self.calculate_tag_area(target_tag.corners)
        angular_error = target_tag.centre.x - self.image_center_x
        area_error = self.target_area - current_area

        twist = Twist()

        if abs(angular_error) > self.angular_deadzone:
            twist.angular.z = self.angular_pid.update(-angular_error, current_time)
        else:
            twist.angular.z = 0.0
            self.angular_pid.reset()

        if abs(area_error) > self.area_deadzone:
            twist.linear.x = self.linear_pid.update(area_error, current_time)
            if 0 < twist.linear.x < self.min_linear_speed:
                twist.linear.x = self.min_linear_speed
            elif -self.min_linear_speed < twist.linear.x < 0:
                twist.linear.x = -self.min_linear_speed
        else:
            twist.linear.x = 0.0
            self.linear_pid.reset()

        self.cmd_vel_pub.publish(twist)
        return py_trees.common.Status.RUNNING

    def stop(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.angular_pid.reset()
        self.linear_pid.reset()
        return py_trees.common.Status.FAILURE

    def calculate_tag_area(self, corners):
        if len(corners) != 4:
            return 0
        x_coords = [c.x for c in corners]
        y_coords = [c.y for c in corners]
        return (max(x_coords) - min(x_coords)) * (max(y_coords) - min(y_coords))