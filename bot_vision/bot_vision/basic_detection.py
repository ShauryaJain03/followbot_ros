import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class Detection(Node):
    def __init__(self):
        super().__init__("detection")
        self.get_logger().info("detection node started")
        self.cam_subscriber = self.create_subscription(Image, "/camera/image_raw", self.detection_callback, 10)
        self.bridge = CvBridge()

    def detection_callback(self, msg):
        try:

            frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

            #grayscale image
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)  #Converts the color image (BGR) to a grayscale image.Each pixel stores intensity only (0–255), reducing computational load.

            #gaussian blur
            blurred = cv2.GaussianBlur(gray,(5,5),0)  #Applies a Gaussian filter (weighted average) to reduce image noise and detail.The kernel size (5,5) controls the amount of blurring.

            # Canny Edge Detection
            edges = cv2.Canny(blurred, 50, 150) #Detects edges (intensity changes) in the image using gradient methods.Uses two thresholds: 50 (low) and 150 (high) to filter weak/strong edges.

            #thresholding
            _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)  #pixels above 127 become 255(white) and below 127 become 0(black), useful for seperating objects from background

            #contour detection - 
            '''Finds continuous boundaries of objects (like outlines).
            drawContours draws them on the image.'''
            contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contour_img = frame.copy()
            cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 2)

            # HSV Masking for Red
            '''Converts the image to HSV color space (Hue, Saturation, Value).
            Creates a mask that keeps only red pixels.  
            bitwise_and keeps only the red parts of the image.'''
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            red_mask = cv2.inRange(hsv, lower_red, upper_red)
            red_result = cv2.bitwise_and(frame, frame, mask=red_mask)



            cv2.imshow("Original", frame)
            cv2.imshow("Grayscale", gray)
            cv2.imshow("Canny Edges",edges)
            cv2.imshow("Blurred",blurred)
            cv2.imshow("Thresholding",thresh)
            cv2.imshow("Contours", contour_img)
            cv2.imshow("Red Filtered (HSV)", red_result)
            cv2.waitKey(1) 

            
        except Exception as e:
            self.get_logger().error(f"Error in detection callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    detection_node = Detection() 
    
    try:
        rclpy.spin(detection_node) 
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()  
        detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()