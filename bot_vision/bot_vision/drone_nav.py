import rclpy 
from rclpy.node import Node

class DroneNav(Node):
    def __init__(self):
        super().__init__("drone_nav")
        self.get_logger().info("drone_nav node activated")
    
def main(args=None):
    rclpy.init(args=args)
    node = DroneNav()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()