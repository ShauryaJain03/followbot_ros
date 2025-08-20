import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray


class ApriltagConf(Node):
    def __init__(self):
        super().__init__("apriltag_confidence")
        self.subscriber = self.create_subscription(AprilTagDetectionArray,"/apriltag_detections",self.detection_callback,10)
        
    def apriltag_confidence(self,hamming, decision_margin):
        hamming_conf = max(0.0, 1.0 - hamming * 0.3)
        dm_conf = min(decision_margin / 100.0, 1.0)
        return 0.6 * dm_conf + 0.4 * hamming_conf
    
    def detection_callback(self,msg):
        decision_margin = msg.detections[0].decision_margin
        hamming = msg.detections[0].hamming
        apriltag_conf = self.apriltag_confidence(hamming,decision_margin)
        self.get_logger().info(f"apriltag detection confidence : {apriltag_conf}")

def main(args=None):
    rclpy.init(args=args)
    node = ApriltagConf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()