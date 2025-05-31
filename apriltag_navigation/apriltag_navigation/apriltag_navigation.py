import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.detection_subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/bot_controller/cmd_vel_unstamped', 10)
        self.bridge = CvBridge()
        self.target_detected = False
        self.target_centre = None

    def listener_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        if self.target_detected and self.target_centre and self.detection:
            # Draw a box around the AprilTag
            corners = [(int(corner.x), int(corner.y)) for corner in self.detection.corners]
            cv2.drawContours(current_frame, [np.array(corners)], 0, (0, 255, 0), 2)

            # Draw the tag ID
            tag_id_text = f"Tag ID: {self.target_id}"
            print(self.target_id)
            cv2.putText(current_frame, tag_id_text, (int(self.target_centre.x), int(self.target_centre.y) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Camera Feed", current_frame)
        cv2.waitKey(1)

    def detection_callback(self, msg):
        if len(msg.detections) > 0:
            print(self.detections)
            self.target_detected = True
            self.target_centre = msg.detections[0].centre
            self.move_towards_tag()
        else:
            self.target_detected = False

    def move_towards_tag(self):
        twist = Twist()
        image_width = 640  # Assumes a known image width
        image_height = 480  # Assumes a known image height
        centre_x = self.target_centre.x
        centre_y = self.target_centre.y

        # Proportional control parameters
        linear_speed = 0.1
        angular_speed = 0.1

        # Calculate error from the centre of the image
        error_x = centre_x - (image_width / 2)
        error_y = centre_y - (image_height / 2)

        # Move towards the tag
        twist.linear.x = linear_speed
        twist.angular.z = -error_x * 0.002  # Adjust gain as necessary
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
