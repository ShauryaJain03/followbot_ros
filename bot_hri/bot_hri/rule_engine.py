#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json
import numpy as np

class RuleEngine(Node):
    def __init__(self):
        super().__init__('rule_engine')

        self.threshold_distance = 1.5  
        self.min_confidence = 50     
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.publisher_ = self.create_publisher(String, '/bot/log', 10)

    def laser_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        min_dist = np.nanmin(ranges)
        event_msg = None
        if min_dist < self.threshold_distance:
            confidence = max(60, int(100 - (min_dist / self.threshold_distance) * 100))
            event_msg = {
                "event": "Slowdown_Stop",
                "cause": f"Obstacle at {min_dist:.2f} m",
                "conf": confidence
            }
        else:
            event_msg = {
                "event": "Normal_Drive",
                "cause": "No obstacle within threshold",
                "conf": 90
            }

        msg_out = String()
        msg_out.data = json.dumps(event_msg)
        self.publisher_.publish(msg_out)

        self.get_logger().info(f"Rule Engine published: {msg_out.data}")


def main(args=None):
    rclpy.init(args=args)
    node = RuleEngine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
