#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterType
from bot_msgs.srv import GPSHome
from tf_transformations import euler_from_quaternion
import math

def haversine(lat1, lon1, lat2, lon2):
    R = 6378.137
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.asin(math.sqrt(a))
    dist = R * c * 1000
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(y, x)
    return dist, bearing

class DynamicGPSFollower(Node):
    def __init__(self):
        super().__init__("GPS_Home")
        self.get_logger().info("Send Home using GPS Node started...")
        self.declare_parameter('latitude')
        self.declare_parameter('longitude')
        self.latitude = self.get_parameter('latitude').value
        self.longitude = self.get_parameter('longitude').value
        self.publisher = self.create_publisher(Twist, '/bot_controller/cmd_vel_unstamped', 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/navsat', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu_raw', self.imu_callback, 10)
        self.srv = self.create_service(GPSHome, 'send_gps_home', self.goal_callback)

        self.yaw = None
        self.goal_lat = None
        self.goal_lon = None
        self.active = False

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
        self.active = True
        response.success = True
        response.message = "GPS goal received."
        self.get_logger().info(f"New goal received: lat={self.goal_lat}, lon={self.goal_lon}")
        return response

    def control_loop(self):
        msg = Twist()
        rate = self.create_rate(10)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if not self.active or None in [self.latitude, self.longitude, self.yaw]:
                continue

            distance, bearing = haversine(self.latitude, self.longitude, self.goal_lat, self.goal_lon)
            heading_error = -1 * bearing - (self.yaw - math.pi / 2)

            if heading_error > math.pi:
                heading_error -= 2 * math.pi
            if heading_error < -math.pi:
                heading_error += 2 * math.pi

            self.get_logger().info(f"Distance to goal: {distance:.2f} m, Heading error: {heading_error:.3f} rad")

            if abs(heading_error) > 0.03:
                msg.linear.x = 0.0
                msg.angular.z = 0.3 * (1 if heading_error > 0 else -1)
            else:
                msg.angular.z = 0.0
                msg.linear.x = 0.8 if distance > 1.0 else 0.0

            if distance < 1.0:
                self.get_logger().info("Reached Home Position.")
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.active = False

            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicGPSFollower()
    try:
        node.control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

#ros2 service call /send_gps_home bot_msgs/srv/GPSHome "{latitude: 47.47894999999999, longitude: 19.057785}"