import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        
    def update(self, error, current_time=None):
        if current_time is None:
            current_time = time.time()
            
        if self.prev_time is None:
            self.prev_time = current_time
            dt = 0.0
        else:
            dt = current_time - self.prev_time
            
        if dt <= 0.0:
            dt = 0.01  
            
        proportional = self.kp * error
        
        self.integral += error * dt
        integral_term = self.ki * self.integral
        
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        derivative_term = self.kd * derivative
        
        output = proportional + integral_term + derivative_term
        
        output = max(min(output, self.output_limits[1]), self.output_limits[0])
        
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

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
        
        self.image_center_x = 320  
        

        self.angular_pid = PIDController(
            kp=0.005,   
            ki=0.0001,  
            kd=0.01,     
            output_limits=(-0.8, 0.8) 
        )
        
        self.linear_pid = PIDController(
            kp=0.00008, 
            ki=0.000001, 
            kd=0.0001,  
            output_limits=(-0.8, 0.8)  
        )
        
        self.angular_deadzone = 30   
        self.area_deadzone = 1000     
        self.min_linear_speed = 0.1   
        self.max_no_detection_time = 2.0  
        
        self.last_detection_time = time.time()
        
        self.get_logger().info(f'Following tag ID: {self.target_tag_id}')
        self.get_logger().info(f'Target area: {self.target_area} pixels')
        self.get_logger().info('controllers initialized')
        
    def calculate_tag_area(self, corners):
        if len(corners) != 4:
            return 0
            
        x_coords = [corner.x for corner in corners]
        y_coords = [corner.y for corner in corners]
        
        width = max(x_coords) - min(x_coords)
        height = max(y_coords) - min(y_coords)
        
        return width * height
    
    def apriltag_callback(self, msg: AprilTagDetectionArray):
        current_time = time.time()
        
        if not msg.detections:
            if (current_time - self.last_detection_time > self.max_no_detection_time):
                self.get_logger().info("No detection for too long! Stopping.")
                self.stop_robot()
            return
        
        target_tag = None
        for detection in msg.detections:
            if detection.id == self.target_tag_id:
                target_tag = detection
                break
        
        if target_tag is None:
            if current_time - self.last_detection_time > self.max_no_detection_time:
                self.get_logger().info(f"Target tag {self.target_tag_id} not found! Stopping.")
                self.stop_robot()
            return
        
        self.last_detection_time = current_time
        
        current_area = self.calculate_tag_area(target_tag.corners)
        
        tag_x = target_tag.centre.x
        
        angular_error = tag_x - self.image_center_x 
        area_error = self.target_area - current_area 
        twist = Twist()
        
        if abs(angular_error) > self.angular_deadzone:
            angular_output = self.angular_pid.update(-angular_error, current_time)  
            twist.angular.z = angular_output
        else:
            twist.angular.z = 0.0
            self.angular_pid.reset()  
        
        if abs(area_error) > self.area_deadzone:
            linear_output = self.linear_pid.update(area_error, current_time)
            
            if linear_output > 0 and linear_output < self.min_linear_speed:
                linear_output = self.min_linear_speed
            elif linear_output < 0 and linear_output > -self.min_linear_speed:
                linear_output = -self.min_linear_speed
                
            twist.linear.x = linear_output
        else:
            twist.linear.x = 0.0
            self.linear_pid.reset()  
            self.get_logger().info('Target area reached! Holding position.')
        
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(
            f'Area: {current_area:.0f}/{self.target_area} | '
            f'X-error: {angular_error:.0f} | '
            f'Lin: {twist.linear.x:.3f} | '
            f'Ang: {twist.angular.z:.3f}'
        )
        
    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.angular_pid.reset()
        self.linear_pid.reset()


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()