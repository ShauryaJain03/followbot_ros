import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.detection_subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.detection_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/bot_controller/cmd_vel_unstamped', 10)

    def detection_callback(self, msg: AprilTagDetectionArray):
        if not msg.detections:
            self.get_logger().info('No tag detections.')
            return

        for detection in msg.detections:
            self.get_logger().info(f"Tag ID: {detection.id}")
            self.get_logger().info(f"Family: {detection.family}")
            self.get_logger().info(f"Hamming: {detection.hamming}")
            self.get_logger().info(f"Decision Margin: {detection.decision_margin}")
            self.get_logger().info(f"Centre: ({detection.centre.x}, {detection.centre.y})")
            '''self.get_logger().info("  Corners:")
            for i, corner in enumerate(detection.corners):
                self.get_logger().info(f"    Corner {i+1}: ({corner.x}, {corner.y})")'''
            cmd_vel = Twist()
            cmd_vel.linear.x = 1.0

            if detection.id == 0:
                self.publisher.publish(cmd_vel)
                



def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
