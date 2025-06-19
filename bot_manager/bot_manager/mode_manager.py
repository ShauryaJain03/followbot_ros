import rclpy
from rclpy.node import Node
import subprocess
import signal
from bot_msgs.srv import SetMode
from bot_msgs.srv import GPSHome

class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')

        self.srv = self.create_service(SetMode, 'set_mode', self.set_mode_callback)
        self.create_service(GPSHome, "send_gps_home", self.set_home_callback)

        self.get_logger().info("ModeManager node started")

        self.gps_lat = None
        self.gps_lon = None
        self.current_process = None
        self.current_mode = None

    def set_home_callback(self, request, response):
        self.gps_lat = request.latitude
        self.gps_lon = request.longitude
        self.get_logger().info(f"Home position received: {self.gps_lat}, {self.gps_lon}")
        response.success = True
        response.message = "Home coordinates set"
        return response

    def stop_current_process(self):
        if self.current_process and self.current_process.poll() is None:
            self.get_logger().info(f"Terminating previous mode: {self.current_mode}")
            self.current_process.send_signal(signal.SIGINT)
            try:
                self.current_process.wait(timeout=5)
                self.get_logger().info("Previous process shut down gracefully.")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Force killing unresponsive process.")
                self.current_process.kill()
            self.current_process = None

    def set_mode_callback(self, request, response):
        mode = request.mode_name

        # Stop any existing running process
        self.stop_current_process()

        launch_cmd = None

        if mode == "follow":
            launch_cmd = ["ros2", "launch", "apriltag_navigation", "apriltag_navigation.launch.py"]
        elif mode == "gps_waypoint":
            launch_cmd = ["ros2", "launch", "bot_gps", "gps.launch.py"]
        elif mode == "return_home":
            if self.gps_lat is None or self.gps_lon is None:
                response.success = False
                response.message = "Home GPS coordinates not set"
                return response
            launch_cmd = [
                "ros2", "run", "bot_gps", "gps_home",
                "--ros-args", "-p", f"latitude:={self.gps_lat}", "-p", f"longitude:={self.gps_lon}"
            ]
        elif mode=="manual":
            pass
        else:
            response.success = False
            response.message = f"Unknown mode: {mode}"
            return response

        try:
            self.get_logger().info(f"Launching new mode: {mode}")
            self.current_process = subprocess.Popen(launch_cmd)
            self.current_mode = mode
            response.success = True
            response.message = f"Mode '{mode}' launched"
        except Exception as e:
            self.get_logger().error(f"Failed to launch {mode}: {str(e)}")
            response.success = False
            response.message = f"Failed to launch {mode}: {str(e)}"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.stop_current_process()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
