import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
import time 


class ModePublisher(Node):
    def __init__(self):
        super().__init__("mode_publisher")
        self.pub = self.create_publisher(String,"/bot/mode",10)
        self.start_time = time.time()
        timer_period = 0.5 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        msg = String()
        if(elapsed<120.0):           
            msg.data = "follow"
            self.pub.publish(msg)
        else:
            msg.data = "return"
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
