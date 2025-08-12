import rclpy
from rclpy.node import Node

class NavigationConf(Node):
    def __init__(self):
        super().__init__("navigation_confidence")



def main(args=None):
    rclpy.init(args=args)
    node = NavigationConf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()