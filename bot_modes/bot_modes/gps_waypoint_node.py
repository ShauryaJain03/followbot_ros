#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist
from bot_msgs.srv import GPSHome
from tf_transformations import euler_from_quaternion
import math
import threading
import time


def haversine(lat1, lon1, lat2, lon2):
    R = 6378.137
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.asin(math.sqrt(a))
    dist = R * c * 1000
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(y, x)
    return dist, bearing


class GPSHomeLifecycle(Node):
    def __init__(self, node_name="gps_waypoint_node"):
        super().__init__(node_name)

        self.get_logger().info("GPS Waypoint node created")
        self.latitude = None
        self.longitude = None
        self.yaw = None

        self.goal_lat = None
        self.goal_lon = None
        self.active_goal = False

        self.thread = None
        self.running = False

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("GPS Waypoint Node configured...")

        self.declare_parameter('latitude', 0.0)
        self.declare_parameter('longitude', 0.0)

        self.gps_sub = self.create_subscription(NavSatFix, '/navsat', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu_raw', self.imu_callback, 10)
        self.cmd_vel_pub = self.create_lifecycle_publisher(Twist, '/bot_controller/cmd_vel_unstamped', 10)

        self.srv = self.create_service(GPSHome, 'send_gps_waypoints', self.goal_callback)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("GPS Waypoint node Activated")

        self.running = True
        self.thread = threading.Thread(target=self.control_loop)
        self.thread.start()

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("GPS Waypoint node Deactivated")
        self.running = False
        if self.thread:
            self.thread.join()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up GPS Waypoint node")
        if hasattr(self, 'gps_sub'):
            self.destroy_subscription(self.gps_sub)
        if hasattr(self, 'imu_sub'):
            self.destroy_subscription(self.imu_sub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting Down GPS Waypoint node")
        return TransitionCallbackReturn.SUCCESS

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        _, _, self.yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

    def goal_callback(self, request, response):
        self.goal_lat = request.latitude
        self.goal_lon = request.longitude
        self.active_goal = True
        response.success = True
        response.message = "GPS goal received."
        self.get_logger().info(f"New goal received: lat={self.goal_lat}, lon={self.goal_lon}")
        return response

    def control_loop(self):
        rate = 10  # Hz
        interval = 1.0 / rate

        while self.running and rclpy.ok():
            if not self.active_goal or None in [self.latitude, self.longitude, self.yaw]:
                time.sleep(interval)
                continue

            distance, bearing = haversine(self.latitude, self.longitude, self.goal_lat, self.goal_lon)
            heading_error = -1 * bearing - (self.yaw - math.pi / 2)

            if heading_error > math.pi:
                heading_error -= 2 * math.pi
            if heading_error < -math.pi:
                heading_error += 2 * math.pi

            self.get_logger().info(f"Distance to goal: {distance:.2f} m, Heading error: {heading_error:.3f} rad")

            msg = Twist()

            if abs(heading_error) > 0.03:
                msg.linear.x = 0.0
                msg.angular.z = 0.3 * (1 if heading_error > 0 else -1)
            else:
                msg.angular.z = 0.0
                msg.linear.x = 0.8 if distance > 1.0 else 0.0

            if distance < 1.0:
                self.get_logger().info("Reached Waypoint Position.")
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.active_goal = False

            self.cmd_vel_pub.publish(msg)
            time.sleep(interval)


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = GPSHomeLifecycle()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("Keyboard Interrupt. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#ros2 service call /send_gps_waypoints bot_msgs/srv/GPSHome "{latitude: 47.478830, longitude: 19.058087}"