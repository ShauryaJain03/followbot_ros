import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

class BatteryPub(Node):
    def __init__(self):
        super().__init__("battery_publisher")
        self.pub = self.create_publisher(Float32, "/bot/battery", 10)
        self.start_time = time.time()
        self.battery = 100.0 
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        if elapsed < 300:  
            if (int(elapsed) % 10 == 0):  
                self.battery = self.battery - 2.0

        msg = Float32()
        msg.data = self.battery
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
