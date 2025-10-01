#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from threading import Lock

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

        self.image_topic = '/camera/image_raw'      
        self.viz_topic = '/apriltag_detections/detection'
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.viz_pub = self.create_publisher(Image, self.viz_topic, 10)

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
        self.bridge = CvBridge()
        self.latest_image_cv = None
        self.image_lock = Lock()

        self.get_logger().info(f'Following tag ID: {self.target_tag_id}')
        self.get_logger().info(f'Target area: {self.target_area} pixels')
        self.get_logger().info('Controllers initialized')
        self.get_logger().info(f'Subscribed image topic: {self.image_topic}')
        self.get_logger().info(f'Publishing annotated image on: {self.viz_topic}')
        
    def calculate_tag_area(self, corners):
        if len(corners) != 4:
            return 0
            
        x_coords = [corner.x for corner in corners]
        y_coords = [corner.y for corner in corners]
        
        width = max(x_coords) - min(x_coords)
        height = max(y_coords) - min(y_coords)
        
        return width * height

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        with self.image_lock:
            self.latest_image_cv = cv_image.copy()

    def apriltag_callback(self, msg: AprilTagDetectionArray):
        current_time = time.time()
        
        if not msg.detections:
            if (current_time - self.last_detection_time > self.max_no_detection_time):
                self.get_logger().info("No detection for too long! Stopping.")
                self.stop_robot()
            self.publish_latest_image()
            return
        
        target_tag = None
        for detection in msg.detections:
            det_id = None
            try:
                det_id = detection.id[0] if isinstance(detection.id, (list, tuple)) and detection.id else detection.id
            except Exception:
                det_id = detection.id
            if det_id == self.target_tag_id:
                target_tag = detection
                break
        
        if target_tag is None:
            if current_time - self.last_detection_time > self.max_no_detection_time:
                self.get_logger().info(f"Target tag {self.target_tag_id} not found! Stopping.")
                self.stop_robot()
            self.publish_latest_image()
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
            self.get_logger().info('Target area reached! Stopping.')
        
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(
            f'Area: {current_area:.0f}/{self.target_area} | '
            f'X-error: {angular_error:.0f} | '
            f'Lin: {twist.linear.x:.3f} | '
            f'Ang: {twist.angular.z:.3f}'
        )

        self.publish_annotated_image(target_tag, msg.header)
        

    def publish_annotated_image(self, detection, header):
        with self.image_lock:
            if self.latest_image_cv is None:
                canvas = np.zeros((480, 640, 3), dtype=np.uint8)
            else:
                canvas = self.latest_image_cv.copy()

        try:
            corners = detection.corners
            if corners and len(corners) >= 4:
                pts = np.array([[int(round(c.x)), int(round(c.y))] for c in corners], dtype=np.int32)
                cv2.polylines(canvas, [pts], isClosed=True, color=(255, 0, 0), thickness=2)
                for (x, y) in pts:
                    cv2.circle(canvas, (x, y), 3, (0, 0, 255), -1)
            cx = int(round(detection.centre.x))
            cy = int(round(detection.centre.y))
            cv2.circle(canvas, (cx, cy), 6, (0, 255, 0), -1)
            try:
                tag_id = detection.id[0] if isinstance(detection.id, (list, tuple)) and detection.id else detection.id
                if tag_id is not None:
                    cv2.putText(canvas, f'ID:{tag_id}', (max(5, cx - 20), max(15, cy - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            except Exception:
                pass
        except Exception as e:
            self.get_logger().warn(f'Error annotating')

        try:
            out_msg = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
            if header is not None:
                out_msg.header = header
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge cv2_to_imgmsg error: {e}')
            return

        self.viz_pub.publish(out_msg)


    def publish_latest_image(self):
        with self.image_lock:
            if self.latest_image_cv is None:
                return
            canvas = self.latest_image_cv.copy()
        try:
            out_msg = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge cv2_to_imgmsg error: {e}')
            return
        self.viz_pub.publish(out_msg)

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
