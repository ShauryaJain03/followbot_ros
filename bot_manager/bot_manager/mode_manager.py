import rclpy
from rclpy.node import Node
import subprocess
from bot_msgs.srv import SetMode

class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')
        self.srv = self.create_service(SetMode, 'set_mode', self.set_mode_callback)
        self.get_logger().info("manager node started")

    def set_mode_callback(self, request, response):
        mode = request.mode_name
        if mode == "follow":
            subprocess.Popen(["ros2", "launch", "apriltag_navigation", "apriltag_navigation.launch.py"])
        elif mode == "gps_waypoint":
            subprocess.Popen(["ros2", "launch", "bot_gps", "gps.launch.py"])
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