#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque

class IntermediateCVNode(Node):
    """
    Intermediate Computer Vision Node demonstrating advanced CV techniques:
    
    1. Optical Flow (Lucas-Kanade + Farneback)
    2. Background Subtraction with post-processing
    3. Feature Detection and Matching (ORB, SIFT-like)
    4. Hough Transforms (Lines + Circles)
    5. Template Matching with multi-scale
    6. Motion Analysis and Tracking
    7. Image Segmentation (Watershed, K-means)
    8. Contour Analysis with shape descriptors
    """
    
    def __init__(self):
        super().__init__('intermediate_cv_node')
        
        # ROS Setup
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.listener_callback, 10)
        self.publisher = self.create_publisher(Image, '/processed_image', 10)
        self.bridge = CvBridge()
        
        # Computer Vision State Variables
        self.prev_gray = None
        self.prev_frame = None
        self.frame_count = 0
        
        # Background Subtraction
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            detectShadows=True, varThreshold=50)
        
        # Feature Detection
        self.orb = cv2.ORB_create(nfeatures=500)
        self.prev_keypoints = None
        self.prev_descriptors = None
        
        # Lucas-Kanade Optical Flow
        self.lk_params = dict(winSize=(15, 15),
                             maxLevel=2,
                             criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.good_features_params = dict(maxCorners=100,
                                       qualityLevel=0.3,
                                       minDistance=7,
                                       blockSize=7)
        self.tracks = []
        self.track_len = 10
        
        # Template for multi-scale matching
        self.template = None
        self.template_initialized = False
        
        # Motion history
        self.motion_history = deque(maxlen=10)
        
        self.get_logger().info("Intermediate CV Node initialized")
    
    def lucas_kanade_tracking(self, frame, gray):
        """Lucas-Kanade optical flow for feature tracking"""
        vis = frame.copy()
        
        if len(self.tracks) > 0:
            img0, img1 = self.prev_gray, gray
            p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
            p1, _st, _err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **self.lk_params)
            p0r, _st, _err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **self.lk_params)
            d = abs(p0 - p0r).reshape(-1, 2).max(-1)
            good = d < 1
            new_tracks = []
            
            for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                tr.append((x, y))
                if len(tr) > self.track_len:
                    del tr[0]
                new_tracks.append(tr)
                cv2.circle(vis, (int(x), int(y)), 2, (0, 255, 0), -1)
            
            self.tracks = new_tracks
            
            # Draw tracks
            for tr in self.tracks:
                cv2.polylines(vis, [np.int32(tr)], False, (0, 255, 0))
        
        # Detect new features if needed
        if self.frame_count % 20 == 0:
            mask = np.zeros_like(gray)
            mask[:] = 255
            for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                cv2.circle(mask, (x, y), 5, 0, -1)
            
            p = cv2.goodFeaturesToTrack(gray, mask=mask, **self.good_features_params)
            if p is not None:
                for x, y in np.float32(p).reshape(-1, 2):
                    self.tracks.append([(x, y)])
        
        return vis
    
    def advanced_background_subtraction(self, frame):
        """Background subtraction with morphological post-processing"""
        # Apply background subtraction
        fg_mask = self.bg_subtractor.apply(frame)
        
        # Morphological operations to clean up the mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        
        # Remove noise
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
        
        # Fill holes
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)
        
        # Remove shadows (MOG2 marks shadows as 127)
        fg_mask[fg_mask == 127] = 0
        
        # Find contours for object detection
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by area and draw bounding boxes
        result = frame.copy()
        min_area = 500
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Calculate contour properties
                aspect_ratio = float(w) / h
                extent = float(area) / (w * h)
                
                # Add text with object properties
                cv2.putText(result, f'A:{int(area)} AR:{aspect_ratio:.1f}', 
                           (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return result, cv2.cvtColor(fg_mask, cv2.COLOR_GRAY2BGR)
    
    def feature_detection_matching(self, frame, gray):
        """ORB feature detection and matching between frames"""
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        
        vis = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0), flags=0)
        
        # Match with previous frame
        if self.prev_descriptors is not None and descriptors is not None:
            # Brute force matcher
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(self.prev_descriptors, descriptors)
            matches = sorted(matches, key=lambda x: x.distance)
            
            # Draw top matches
            if len(matches) > 10:
                good_matches = matches[:20]
                for match in good_matches:
                    pt1 = tuple(map(int, self.prev_keypoints[match.queryIdx].pt))
                    pt2 = tuple(map(int, keypoints[match.trainIdx].pt))
                    cv2.line(vis, pt1, pt2, (255, 0, 0), 1)
                    cv2.circle(vis, pt2, 2, (0, 0, 255), -1)
        
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors
        
        return vis
    
    def advanced_hough_transforms(self, frame, gray):
        """Hough transforms for line and circle detection"""
        result = frame.copy()
        
        # Edge detection for Hough transforms
        edges = cv2.Canny(gray, 50, 150, apertureSize=5)
        
        # Hough Line Transform (Probabilistic)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, 
                               minLineLength=50, maxLineGap=10)
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(result, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Calculate line angle
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                mid_x, mid_y = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.putText(result, f'{int(angle)}°', (mid_x, mid_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # Hough Circle Transform
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
                                  param1=50, param2=30, minRadius=10, maxRadius=100)
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(result, (x, y), r, (0, 0, 255), 2)
                cv2.circle(result, (x, y), 2, (0, 0, 255), 3)
                cv2.putText(result, f'r:{r}', (x - 20, y - r - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        return result
    
    def multi_scale_template_matching(self, frame, gray):
        """Multi-scale template matching"""
        if not self.template_initialized and self.frame_count == 30:
            # Initialize template from center region
            h, w = gray.shape
            self.template = gray[h//3:2*h//3, w//3:2*w//3].copy()
            self.template_initialized = True
            self.get_logger().info("Template initialized")
        
        result = frame.copy()
        
        if self.template_initialized and self.template.size > 0:
            best_match = None
            best_val = -1
            best_scale = 1.0
            
            # Multi-scale matching
            for scale in [0.5, 0.75, 1.0, 1.25, 1.5]:
                scaled_template = cv2.resize(self.template, None, fx=scale, fy=scale)
                
                if scaled_template.shape[0] > gray.shape[0] or scaled_template.shape[1] > gray.shape[1]:
                    continue
                
                match_result = cv2.matchTemplate(gray, scaled_template, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, max_loc = cv2.minMaxLoc(match_result)
                
                if max_val > best_val:
                    best_val = max_val
                    best_match = max_loc
                    best_scale = scale
            
            # Draw best match
            if best_match and best_val > 0.6:
                th, tw = int(self.template.shape[0] * best_scale), int(self.template.shape[1] * best_scale)
                cv2.rectangle(result, best_match, 
                             (best_match[0] + tw, best_match[1] + th), (255, 0, 0), 2)
                cv2.putText(result, f'Match: {best_val:.2f} Scale: {best_scale:.1f}', 
                           (best_match[0], best_match[1] - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        return result
    
    def motion_analysis(self, gray):
        """Analyze motion patterns"""
        if self.prev_gray is not None:
            # Dense optical flow
            flow = cv2.calcOpticalFlowFarneback(self.prev_gray, gray, None, 
                                               0.5, 3, 15, 3, 5, 1.2, 0)
            
            # Calculate motion magnitude
            magnitude = np.sqrt(flow[..., 0]**2 + flow[..., 1]**2)
            direction = np.arctan2(flow[..., 1], flow[..., 0])
            
            # Motion statistics
            avg_motion = np.mean(magnitude)
            max_motion = np.max(magnitude)
            
            self.motion_history.append(avg_motion)
            
            # Create motion visualization
            hsv = np.zeros((gray.shape[0], gray.shape[1], 3), dtype=np.uint8)
            hsv[..., 1] = 255
            hsv[..., 0] = direction * 180 / np.pi / 2
            hsv[..., 2] = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX)
            motion_vis = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            
            # Add motion statistics text
            cv2.putText(motion_vis, f'Avg Motion: {avg_motion:.1f}', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(motion_vis, f'Max Motion: {max_motion:.1f}', 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            return motion_vis
        
        return np.zeros((gray.shape[0], gray.shape[1], 3), dtype=np.uint8)
    
    def image_segmentation(self, frame):
        """K-means clustering for color-based segmentation"""
        # Reshape image for k-means
        data = frame.reshape((-1, 3))
        data = np.float32(data)
        
        # Apply K-means
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        k = 4  # Number of clusters
        _, labels, centers = cv2.kmeans(data, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        
        # Convert back to uint8 and reshape
        centers = np.uint8(centers)
        segmented = centers[labels.flatten()]
        segmented = segmented.reshape(frame.shape)
        
        return segmented
    
    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Apply different CV techniques
            lk_result = self.lucas_kanade_tracking(frame, gray)
            bg_result, fg_mask = self.advanced_background_subtraction(frame)
            feature_result = self.feature_detection_matching(frame, gray)
            hough_result = self.advanced_hough_transforms(frame, gray)
            template_result = self.multi_scale_template_matching(frame, gray)
            motion_result = self.motion_analysis(gray)
            segmented_result = self.image_segmentation(frame)
            
            # Create a comprehensive output display
            # Resize all images to same size for concatenation
            h, w = 240, 320
            
            # Top row
            top_row = cv2.hconcat([
                cv2.resize(lk_result, (w, h)),
                cv2.resize(bg_result, (w, h)),
                cv2.resize(feature_result, (w, h))
            ])
            
            # Middle row  
            middle_row = cv2.hconcat([
                cv2.resize(hough_result, (w, h)),
                cv2.resize(template_result, (w, h)),
                cv2.resize(motion_result, (w, h))
            ])
            
            # Bottom row
            bottom_row = cv2.hconcat([
                cv2.resize(segmented_result, (w, h)),
                cv2.resize(fg_mask, (w, h)),
                cv2.resize(frame, (w, h))  # Original for reference
            ])
            
            # Combine all rows
            output = cv2.vconcat([top_row, middle_row, bottom_row])
            
            # Add labels
            labels = [
                "Lucas-Kanade Tracking", "Background Subtraction", "Feature Matching",
                "Hough Transforms", "Template Matching", "Motion Analysis", 
                "K-means Segmentation", "Foreground Mask", "Original"
            ]
            
            for i, label in enumerate(labels):
                x = (i % 3) * w + 10
                y = (i // 3) * h + 30
                cv2.putText(output, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Publish result
            self.publisher.publish(self.bridge.cv2_to_imgmsg(output, encoding='bgr8'))
            
            # Update state
            self.prev_gray = gray
            self.prev_frame = frame
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = IntermediateCVNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


'''1. Lucas-Kanade Optical Flow Tracking
What it is: Like following a person's movement in a crowd by tracking their hat.

Finds interesting points (corners, edges) in the first frame
Follows where those same points move in the next frame
Draws lines showing their path
Real-world use: Security cameras tracking people, sports analysis following players

2. Background Subtraction
What it is: Like having a "before" photo of an empty room, then seeing what's new when people enter.

Learns what the background looks like when nothing is moving
Anything different from the background = moving object
Removes shadows automatically
Real-world use: Automatic doors, traffic monitoring, intrusion detection

3. Feature Detection & Matching
What it is: Like recognizing your friend by their unique facial features, even in different photos.

Finds unique "landmarks" in images (corners, textures, patterns)
Matches these landmarks between different frames
Even works when the object rotates or moves
Real-world use: Google Photos face recognition, panorama stitching, robot navigation

4. Hough Transforms
What it is: Like having a special tool that can only see straight lines and circles.

Lines: Finds roads, building edges, lane markings
Circles: Finds wheels, coins, round objects
Ignores everything else
Real-world use: Self-driving cars detecting lanes, quality control in manufacturing

5. Multi-Scale Template Matching
What it is: Like playing "Where's Waldo" but Waldo can be different sizes.

Takes a small picture of what you're looking for
Searches the whole image at different sizes
Tells you where it found the best match
Real-world use: Finding logos in videos, medical imaging, satellite image analysis

6. Dense Motion Analysis (Farneback Flow)
What it is: Like seeing wind patterns made visible - every pixel shows which way it's moving.

Creates a "motion map" where colors show movement direction
Brightness shows how fast things are moving
Covers the entire image, not just specific points
Real-world use: Weather radar, crowd flow analysis, video compression

7. K-means Segmentation
What it is: Like sorting M&Ms by color, but for image regions.

Groups pixels with similar colors together
Creates distinct regions/segments
Reduces complex images to simpler color zones
Real-world use: Medical imaging (tumor detection), satellite imagery, photo editing

8. Advanced Background Subtraction
What it is: The smart version of #2 above.

Handles shadows (doesn't confuse them with objects)
Cleans up noise (removes tiny false detections)
Analyzes object shapes (tall person vs wide car)
Real-world use: Smart security systems, people counting, traffic analysis

🔗 How They Work Together
Think of it like a security system:

Background subtraction spots "something moved"
Feature matching recognizes "this is the same person from yesterday"
Lucas-Kanade tracking follows "where they're going"
Hough transforms detects "they're walking along the sidewalk"
Template matching confirms "this matches the VIP photo"
Motion analysis determines "they're walking normally, not running"
Segmentation separates "the person from their shadow"

Each technique solves a specific problem, but combining them creates powerful computer vision systems!
🎮 Easy Analogy
Imagine your eyes and brain:

Background subtraction = noticing something changed in your peripheral vision
Feature matching = recognizing a familiar face
Optical flow = tracking a ball flying through the air
Hough transforms = instantly seeing straight roads and round wheels
Template matching = spotting your favorite brand logo
Motion analysis = sensing the general "flow" of a crowd
Segmentation = separating foreground objects from background'''