import rclpy
from rclpy.node import Node
import subprocess
from bot_msgs.srv import SetMode
from bot_msgs.srv import GPSHome

class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')
        self.srv = self.create_service(SetMode, 'set_mode', self.set_mode_callback)
        self.get_logger().info("manager node started")
        self.gps_lat = None
        self.gps_lon = None
        self.create_service(GPSHome, "send_gps_home", self.set_home_callback)

    def set_home_callback(self, request, response):
        self.gps_lat = request.latitude
        self.gps_lon = request.longitude
        response.success = True
        response.message = "Home coordinates set"
        self.get_logger().info(f"Home position received: {self.gps_lat}, {self.gps_lon}")
        return response

    def set_mode_callback(self, request, response):
        mode = request.mode_name
        if mode == "follow":
            subprocess.Popen(["ros2", "launch", "apriltag_navigation", "apriltag_navigation.launch.py"])
        elif mode == "gps_waypoint":
            subprocess.Popen(["ros2", "launch", "bot_gps", "gps.launch.py"])
        elif mode == "return_home":
            subprocess.Popen(["ros2", "run", "bot_gps", "gps_home",f"--ros-args", "-p", f"latitude:={self.gps_lat}", "-p", f"longitude:={self.gps_lon}"])
        else:
            response.success = False
            response.message = f"Unknown mode: {mode}"
            return response 

        response.success = True
        response.message = f"Mode '{mode}' launched"
        return response

def main(args=None):    
    rclpy.init(args=args)
    node = ModeManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()


#/* ros2 service call /set_mode bot_msgs/srv/SetMode "{mode_name: 'follow'}"*/