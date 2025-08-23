#!/usr/bin/env python3
import py_trees
from geometry_msgs.msg import Twist
from .utils import haversine
import math


class GPSNavigator:
    def __init__(self, cmd_vel_pub, waypoints=None):
        self.cmd_vel_pub = cmd_vel_pub

        self.latitude = None
        self.longitude = None
        self.yaw = 0.0

        if waypoints is None:
            self.waypoints = [[47.47894999999999,19.057785]]
        else:
            self.waypoints = waypoints

        self.waypoint_index = 0

    def update_pose(self, latitude, longitude, yaw):
        self.latitude = latitude
        self.longitude = longitude
        self.yaw = yaw

    def navigate(self):
        if self.latitude is None or self.longitude is None:
            return py_trees.common.Status.RUNNING  

        if self.waypoint_index >= len(self.waypoints):
            self.stop()
            return py_trees.common.Status.SUCCESS

        msg = Twist()
        target_lat, target_lon = self.waypoints[self.waypoint_index]

        distance, bearing = haversine(self.latitude, self.longitude, target_lat, target_lon)

        heading_error = -1 * bearing - (self.yaw - math.pi/2)

        if heading_error > math.pi:
            heading_error -= (2 * math.pi)
        if heading_error < -math.pi:
            heading_error += (2 * math.pi)

        if abs(heading_error) > 0.03:
            msg.linear.x = 0.0
            msg.angular.z = -0.3 if heading_error < 0 else 0.3
        else:
            msg.angular.z = 0.0
            if distance > 1.0:
                msg.linear.x = 0.8
            else:
                msg.linear.x = 0.0
                self.waypoint_index += 1

        self.cmd_vel_pub.publish(msg)

        if self.waypoint_index == len(self.waypoints):
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def stop(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)