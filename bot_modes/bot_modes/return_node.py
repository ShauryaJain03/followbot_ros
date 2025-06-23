#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
import math
import threading


class ReturnHomeLifecycle(LifecycleNode):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

        self.latitude = 0
        self.longitude = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.waypoints = [[47.47894999999999, 19.057785]]
        self.waypoint_index = 0
        
        # Initialize publishers and subscriptions as None
        self.publisher = None
        self.gps_subscription = None
        self.imu_subscription = None
        
        # Control flag for waypoint following
        self.following_waypoints = False
        self.waypoint_thread = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Return Home Node configured...")
        
        # Create publishers and subscriptions in configure state
        self.publisher = self.create_publisher(Twist, '/bot_controller/cmd_vel_unstamped', 10)
        self.gps_subscription = self.create_subscription(NavSatFix, '/navsat', self.navsat_callback, 10)
        self.imu_subscription = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Return Home Node activated...")
        
        # Start waypoint following
        self.following_waypoints = True
        self.waypoint_thread = threading.Thread(target=self.waypoint_follower)
        self.waypoint_thread.daemon = True
        self.waypoint_thread.start()
        
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Return Home Node deactivated...")
        
        # Stop waypoint following
        self.following_waypoints = False
        if self.waypoint_thread and self.waypoint_thread.is_alive():
            self.waypoint_thread.join(timeout=1.0)
        
        # Send zero velocity command to stop the robot
        if self.publisher:
            stop_msg = Twist()
            self.publisher.publish(stop_msg)
        
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Return Home Node cleaned up...")
        
        # Destroy subscriptions and publishers
        if self.gps_subscription:
            self.destroy_subscription(self.gps_subscription)
            self.gps_subscription = None
        if self.imu_subscription:
            self.destroy_subscription(self.imu_subscription)
            self.imu_subscription = None
        if self.publisher:
            self.destroy_publisher(self.publisher)
            self.publisher = None
            
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Return Home Node Shutting Down")
        
        # Stop waypoint following
        self.following_waypoints = False
        
        # Clean up resources
        if self.gps_subscription:
            self.destroy_subscription(self.gps_subscription)
        if self.imu_subscription:
            self.destroy_subscription(self.imu_subscription)
        if self.publisher:
            self.destroy_publisher(self.publisher)
            
        return TransitionCallbackReturn.SUCCESS

    def navsat_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def waypoint_follower(self):
        """Main waypoint following logic running in separate thread"""
        rate = self.create_rate(20)  # 20 Hz
        
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        while self.following_waypoints and rclpy.ok():
            try:
                # Check if we have valid GPS data
                if self.latitude == 0 and self.longitude == 0:
                    self.get_logger().warn("Waiting for GPS data...")
                    rate.sleep()
                    continue

                # Check if we've reached all waypoints
                if self.waypoint_index >= len(self.waypoints):
                    self.get_logger().info("All waypoints reached!")
                    break

                distance, bearing = haversine(
                    self.latitude, self.longitude, 
                    self.waypoints[self.waypoint_index][0], 
                    self.waypoints[self.waypoint_index][1]
                )

                heading_error = -1 * bearing - (self.yaw - math.pi/2)
                        
                if heading_error > math.pi:
                    heading_error = heading_error - (2 * math.pi) 
                if heading_error < -math.pi:
                    heading_error = heading_error + (2 * math.pi)
            
                self.get_logger().info(f'Distance: {distance:.2f} m, heading error: {heading_error:.3f}')

                if abs(heading_error) > 0.03:
                    msg.linear.x = 0.0
                    if heading_error < 0:
                        msg.angular.z = -0.3
                    else:
                        msg.angular.z = 0.3

                else:
                    msg.angular.z = 0.0
                    if distance > 1.0:
                        msg.linear.x = 0.5
                    else:
                        msg.linear.x = 0.0
                        self.get_logger().info("Home Reached!")
                        self.waypoint_index += 1

                # Only publish if we're still active and have a publisher
                if self.publisher and self.following_waypoints:
                    self.publisher.publish(msg)

                rate.sleep()
                
            except Exception as e:
                self.get_logger().error(f"Error in waypoint follower: {str(e)}")
                break

        # Send stop command when done
        if self.publisher and self.following_waypoints:
            stop_msg = Twist()
            self.publisher.publish(stop_msg)
            self.get_logger().info("Waypoint following completed")


def haversine(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    # 0. Radius of earth in km
    R = 6378.137
    # 1. Convert from degrees to radians
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)

    # 2. Haversine formula for distance
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))
    distance_m = R * c * 1000

    # 3. Initial bearing calculation (forward azimuth)
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing_rad = math.atan2(y, x)  # range -π to +π

    return distance_m, bearing_rad


def main(): 
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    return_node = ReturnHomeLifecycle('return_node')
    executor.add_node(return_node)
    
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        return_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()