import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('human_follower')

        self.detection_subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.apriltag_callback,    
            10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/bot_controller/cmd_vel_unstamped', 10)

    
        self.target_tag_id = 0
        self.target_area = 22000  
        self.max_speed = 1.0      
        self.max_turn = 0.5       
        
        self.get_logger().info(f'Following tag ID: {self.target_tag_id}')
        self.get_logger().info(f'Will stop when tag area > {self.target_area} pixels')
        
    def calculate_tag_area(self, corners):
        if len(corners) != 4:
            return 0
            
        x_coords = [corner.x for corner in corners]
        y_coords = [corner.y for corner in corners]
        
        width = max(x_coords) - min(x_coords)
        height = max(y_coords) - min(y_coords)
        
        return width * height
    
    def apriltag_callback(self, msg:AprilTagDetectionArray):
        if not msg.detections:
            self.get_logger().info("No Detection!")
            self.stop_robot()
            return
        
        target_tag = None
        for detection in msg.detections:
            if detection.id == self.target_tag_id:
                target_tag = detection
                break
        
        if target_tag is None:
            self.stop_robot()
            return
        
        area = self.calculate_tag_area(target_tag.corners)
        self.get_logger().info(f'Tag area: {area:.0f} pixels')
        
        if area > self.target_area:
            self.get_logger().info('Target reached! Stopping.')
            self.stop_robot()
            return
        
        twist = Twist()
        
        image_center = 320  
        tag_x = target_tag.centre.x
        
        if abs(tag_x - image_center) > 50:  
            if tag_x < image_center:
                twist.angular.z = self.max_turn 
            else:
                twist.angular.z = -self.max_turn 
        
       
        if abs(tag_x - image_center) < 100:
            twist.linear.x = self.max_speed
        
        self.cmd_vel_pub.publish(twist)
        
    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
