#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String


class LogPublisher(Node):
    def __init__(self):
        super().__init__("log_publisher")
        self.clean_pub = self.create_publisher(String, "/bot/log", 10)
        self.create_subscription(String, "/bot/metadata", self.metadata_callback, 10)

    def metadata_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            cleaned = {
                "mode": data.get("mode"),
                "battery_pct": data.get("battery_pct"),
                "pose": data.get("pose"),
                "active_path": data.get("active_path"),
                "current_task": data.get("current", {}).get("name"),
                "status": data.get("current", {}).get("status"),
                "timestamp": data.get("timestamp", {}).get("sec")
            }
            clean_json = json.dumps(cleaned, ensure_ascii=False)
            out_msg = String()
            out_msg.data = clean_json
            self.clean_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error cleaning metadata: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LogPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
