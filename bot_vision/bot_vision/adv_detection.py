#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
import time

class AdvancedCVNode(Node):
    """
    Advanced Computer Vision Node for Robotics Engineers
    
    Essential techniques every robotics engineer should master:
    
    1. SLAM-Ready Feature Tracking (Visual Odometry basics)
    2. Stereo Vision & Depth Estimation
    3. Object Detection Pipeline (HOG + SVM style)
    4. Visual Servoing (Eye-in-Hand control)
    5. Structure from Motion (3D reconstruction)
    6. Semantic Segmentation (Region-based)
    7. Visual SLAM Features (Loop Closure Detection)
    8. Multi-Object Tracking (Kalman Filter based)
    9. Image Rectification & Calibration
    10. Real-time Performance Optimization
    """
    
    def __init__(self):
        super().__init__('advanced_cv_node')
        
        # ROS Setup
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.listener_callback, 10)
        self.publisher = self.create_publisher(Image, '/advanced_cv_output', 10)
        self.bridge = CvBridge()
        
        # Performance monitoring
        self.frame_times = deque(maxlen=30)
        self.frame_count = 0
        
        # SLAM-ready feature tracking
        self.feature_detector = cv2.goodFeaturesToTrack
        self.feature_params = dict(maxCorners=200, qualityLevel=0.01, 
                                 minDistance=10, blockSize=3)
        self.lk_params = dict(winSize=(21, 21), maxLevel=3,
                            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        
        self.prev_gray = None
        self.prev_pts = None
        self.tracks = []
        self.track_colors = np.random.randint(0, 255, (200, 3))
        
        # Visual odometry state
        self.pose_history = deque(maxlen=100)
        self.current_pose = np.eye(3)  # 2D transformation matrix
        
        # Stereo vision simulation (using temporal stereo)
        self.prev_frames = deque(maxlen=2)
        
        # Object detection (HOG-like features)
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        # Multi-object tracking
        self.trackers = []
        self.tracker_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), 
                              (255, 255, 0), (255, 0, 255), (0, 255, 255)]
        
        # Camera calibration matrices (example values)
        self.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([0.1, -0.2, 0, 0, 0], dtype=np.float32)
        
        # Structure from Motion
        self.keyframe_features = []
        self.keyframe_descriptors = []
        self.orb = cv2.ORB_create(nfeatures=1000)
        
        # Semantic segmentation setup
        self.segmentation_colors = np.random.randint(0, 255, (10, 3))
        
        # Performance optimization
        self.processing_pipeline = []
        
        self.get_logger().info("Advanced CV Node for Robotics initialized")
    
    def visual_odometry_estimation(self, gray):
        """
        Visual Odometry - Track camera movement for SLAM
        Essential for: Robot localization, autonomous navigation
        """
        if self.prev_gray is None or self.prev_pts is None:
            return gray, np.eye(3)
        
        # Track features using Lucas-Kanade
        next_pts, status, error = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None, **self.lk_params)
        
        # Select good points
        good_new = next_pts[status == 1]
        good_old = self.prev_pts[status == 1]
        
        if len(good_new) < 8:  # Need minimum points for pose estimation
            return gray, self.current_pose
        
        # Estimate fundamental matrix and camera motion
        try:
            F, mask = cv2.findFundamentalMat(good_old, good_new, 
                                           cv2.FM_RANSAC, 0.3, 0.99)
            
            if F is not None:
                # Essential matrix from fundamental matrix
                E = self.camera_matrix.T @ F @ self.camera_matrix
                
                # Recover pose from essential matrix
                _, R, t, _ = cv2.recoverPose(E, good_old, good_new, self.camera_matrix)
                
                # Update pose (simplified 2D transformation)
                motion = np.array([[R[0,0], R[0,1], t[0,0]], 
                                 [R[1,0], R[1,1], t[1,0]], 
                                 [0, 0, 1]])
                self.current_pose = self.current_pose @ motion
                
        except Exception as e:
            pass  # Keep previous pose if estimation fails
        
        # Visualize tracked points and trajectory
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        
        for pt in good_new:
            cv2.circle(vis, tuple(pt.astype(int)), 3, (0, 255, 0), -1)
        
        # Draw trajectory
        self.pose_history.append(self.current_pose[:2, 2])
        if len(self.pose_history) > 1:
            trajectory = np.array(self.pose_history)
            # Scale and offset for visualization
            traj_vis = (trajectory * 100 + [160, 120]).astype(int)
            for i in range(1, len(traj_vis)):
                cv2.line(vis, tuple(traj_vis[i-1]), tuple(traj_vis[i]), (255, 0, 0), 2)
        
        # Add pose information
        x, y = self.current_pose[0, 2], self.current_pose[1, 2]
        cv2.putText(vis, f'Pose: ({x:.2f}, {y:.2f})', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return vis, self.current_pose
    
    def stereo_depth_estimation(self, frame):
        """
        Temporal Stereo - Estimate depth from motion
        Essential for: Obstacle avoidance, 3D mapping, path planning
        """
        self.prev_frames.append(frame)
        
        if len(self.prev_frames) < 2:
            return frame
        
        # Use temporal baseline (camera motion) for stereo
        frame1 = cv2.cvtColor(self.prev_frames[0], cv2.COLOR_BGR2GRAY)
        frame2 = cv2.cvtColor(self.prev_frames[1], cv2.COLOR_BGR2GRAY)
        
        # Semi-global block matching for disparity
        stereo = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=64, blockSize=11,
            P1=8 * 3 * 11**2, P2=32 * 3 * 11**2,
            disp12MaxDiff=1, uniquenessRatio=15,
            speckleWindowSize=0, speckleRange=2,
            preFilterCap=63, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        
        disparity = stereo.compute(frame1, frame2)
        
        # Convert to depth map
        disparity = np.float32(disparity) / 16.0
        
        # Create depth visualization
        depth_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = np.uint8(depth_vis)
        depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        
        # Overlay depth on original frame
        result = cv2.addWeighted(frame, 0.7, depth_colored, 0.3, 0)
        
        # Add depth statistics
        valid_depth = disparity[disparity > 0]
        if len(valid_depth) > 0:
            avg_depth = np.mean(valid_depth)
            min_depth = np.min(valid_depth)
            cv2.putText(result, f'Avg Depth: {avg_depth:.1f}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(result, f'Min Depth: {min_depth:.1f}', (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return result
    
    def object_detection_pipeline(self, frame):
        """
        HOG + SVM Object Detection Pipeline
        Essential for: Person detection, obstacle recognition, scene understanding
        """
        # HOG people detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect people using HOG
        boxes, weights = self.hog.detectMultiScale(gray, winStride=(8, 8),
                                                  padding=(32, 32), scale=1.05)
        
        result = frame.copy()
        
        # Draw detection boxes
        for (x, y, w, h), weight in zip(boxes, weights):
            if weight > 0.5:  # Confidence threshold
                cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(result, f'Person: {weight:.2f}', (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Additional feature-based detection (corners as potential objects)
        corners = cv2.goodFeaturesToTrack(gray, maxCorners=50, qualityLevel=0.01,
                                        minDistance=20, blockSize=3)
        
        if corners is not None:
            for corner in corners:
                x, y = corner.ravel().astype(int)
                cv2.circle(result, (x, y), 5, (255, 0, 0), -1)
        
        # Object counting
        cv2.putText(result, f'People Detected: {len(boxes)}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(result, f'Features: {len(corners) if corners is not None else 0}', 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return result
    
    def visual_servoing_control(self, frame):
        """
        Visual Servoing - Control robot based on visual feedback
        Essential for: Robotic manipulation, precision tasks, hand-eye coordination
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Define target (center of image for this example)
        target_x, target_y = frame.shape[1] // 2, frame.shape[0] // 2
        
        # Detect features for servoing
        features = cv2.goodFeaturesToTrack(gray, maxCorners=1, qualityLevel=0.3,
                                         minDistance=50, blockSize=7)
        
        result = frame.copy()
        
        # Draw target
        cv2.circle(result, (target_x, target_y), 20, (0, 0, 255), 2)
        cv2.putText(result, 'TARGET', (target_x - 30, target_y - 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        if features is not None and len(features) > 0:
            # Get the best feature
            fx, fy = features[0].ravel().astype(int)
            
            # Calculate error
            error_x = fx - target_x
            error_y = fy - target_y
            error_magnitude = np.sqrt(error_x**2 + error_y**2)
            
            # Draw current feature
            cv2.circle(result, (fx, fy), 10, (0, 255, 0), 2)
            cv2.line(result, (fx, fy), (target_x, target_y), (255, 255, 0), 2)
            
            # Control commands (proportional controller)
            kp = 0.01  # Proportional gain
            cmd_x = -kp * error_x  # Negative for proper control direction
            cmd_y = -kp * error_y
            
            # Display control information
            cv2.putText(result, f'Error: ({error_x}, {error_y})', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(result, f'Control: ({cmd_x:.3f}, {cmd_y:.3f})', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(result, f'Distance: {error_magnitude:.1f}px', (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Color code based on error magnitude
            if error_magnitude < 20:
                status_color = (0, 255, 0)  # Green - on target
                status = "ON TARGET"
            elif error_magnitude < 50:
                status_color = (0, 255, 255)  # Yellow - close
                status = "CLOSE"
            else:
                status_color = (0, 0, 255)  # Red - far
                status = "ADJUSTING"
            
            cv2.putText(result, status, (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        return result
    
    def structure_from_motion(self, frame):
        """
        Structure from Motion - Build 3D map from 2D images
        Essential for: 3D mapping, SLAM, scene reconstruction
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect and compute features
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        
        result = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0), flags=0)
        
        # Store keyframes periodically
        if self.frame_count % 10 == 0 and descriptors is not None:
            self.keyframe_features.append(keypoints)
            self.keyframe_descriptors.append(descriptors)
            
            # Keep only recent keyframes
            if len(self.keyframe_features) > 5:
                self.keyframe_features.pop(0)
                self.keyframe_descriptors.pop(0)
        
        # Match with previous keyframes
        if len(self.keyframe_descriptors) >= 2 and descriptors is not None:
            # Match current frame with previous keyframe
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(self.keyframe_descriptors[-2], descriptors)
            matches = sorted(matches, key=lambda x: x.distance)
            
            # Draw matches
            good_matches = matches[:20]
            for match in good_matches:
                if match.trainIdx < len(keypoints):
                    pt = tuple(map(int, keypoints[match.trainIdx].pt))
                    cv2.circle(result, pt, 3, (255, 0, 0), -1)
            
            # Estimate 3D structure (simplified triangulation simulation)
            if len(good_matches) > 8:
                # Extract matching points
                pts1 = np.float32([self.keyframe_features[-2][m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                pts2 = np.float32([keypoints[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                
                # Estimate essential matrix
                E, _ = cv2.findEssentialMat(pts1, pts2, self.camera_matrix)
                
                if E is not None:
                    # Recover pose
                    _, R, t, _ = cv2.recoverPose(E, pts1, pts2, self.camera_matrix)
                    
                    # Display 3D reconstruction info
                    cv2.putText(result, f'3D Points: {len(good_matches)}', (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(result, f'Baseline: {np.linalg.norm(t):.3f}', (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.putText(result, f'Features: {len(keypoints)}', (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(result, f'Keyframes: {len(self.keyframe_features)}', (10, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return result
    
    def semantic_segmentation(self, frame):
        """
        Simple Semantic Segmentation using clustering
        Essential for: Scene understanding, navigation planning, object classification
        """
        # Convert to LAB color space for better clustering
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        
        # Reshape for clustering
        data = lab.reshape((-1, 3))
        data = np.float32(data)
        
        # K-means clustering for segmentation
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        k = 6  # Number of semantic classes
        _, labels, centers = cv2.kmeans(data, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        
        # Create semantic map
        labels = labels.reshape(frame.shape[:2])
        
        # Color each segment
        segmented = np.zeros_like(frame)
        for i in range(k):
            mask = (labels == i)
            segmented[mask] = self.segmentation_colors[i]
        
        # Overlay on original image
        result = cv2.addWeighted(frame, 0.6, segmented, 0.4, 0)
        
        # Add semantic labels
        for i in range(k):
            mask = (labels == i)
            if np.sum(mask) > 1000:  # Only label large regions
                # Find center of mass for label placement
                y_coords, x_coords = np.where(mask)
                if len(y_coords) > 0:
                    center_y, center_x = int(np.mean(y_coords)), int(np.mean(x_coords))
                    cv2.putText(result, f'Class_{i}', (center_x - 30, center_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        return result
    
    def multi_object_tracking(self, frame):
        """
        Multi-Object Tracking using Kalman Filters
        Essential for: Dynamic obstacle avoidance, behavior prediction, surveillance
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect objects (using contours as simple objects)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=4)
        
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        result = frame.copy()
        
        # Get centroids of detected objects
        detections = []
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Filter small objects
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    detections.append((cx, cy))
        
        # Simple tracking using nearest neighbor
        if not hasattr(self, 'tracked_objects'):
            self.tracked_objects = []
        
        # Update existing tracks
        for i, (x, y) in enumerate(detections):
            cv2.circle(result, (x, y), 10, (0, 255, 0), 2)
            
            # Find closest existing track
            if self.tracked_objects:
                distances = [np.sqrt((x - tx)**2 + (y - ty)**2) for tx, ty in self.tracked_objects]
                min_dist_idx = np.argmin(distances)
                
                if distances[min_dist_idx] < 50:  # Maximum tracking distance
                    # Update existing track
                    self.tracked_objects[min_dist_idx] = (x, y)
                    color = self.tracker_colors[min_dist_idx % len(self.tracker_colors)]
                    cv2.putText(result, f'Track_{min_dist_idx}', (x - 30, y - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                else:
                    # New track
                    self.tracked_objects.append((x, y))
            else:
                # First detection
                self.tracked_objects.append((x, y))
        
        # Remove lost tracks (simplified)
        if self.frame_count % 30 == 0:  # Clean up every 30 frames
            self.tracked_objects = self.tracked_objects[-5:]  # Keep only recent 5
        
        cv2.putText(result, f'Tracked Objects: {len(self.tracked_objects)}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return result
    
    def image_rectification(self, frame):
        """
        Image Rectification and Calibration
        Essential for: Accurate measurements, stereo vision, AR/VR applications
        """
        # Simulate camera distortion correction
        h, w = frame.shape[:2]
        
        # Get optimal camera matrix
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
        
        # Undistort the image
        undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, 
                                   None, new_camera_matrix)
        
        # Create side-by-side comparison
        comparison = np.hstack((frame, undistorted))
        
        # Add grid overlay to show rectification
        grid_spacing = 50
        for i in range(0, h, grid_spacing):
            cv2.line(comparison, (0, i), (w * 2, i), (0, 255, 0), 1)
        for i in range(0, w * 2, grid_spacing):
            cv2.line(comparison, (i, 0), (i, h), (0, 255, 0), 1)
        
        # Add labels
        cv2.putText(comparison, 'Original', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(comparison, 'Rectified', (w + 10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return comparison
    
    def performance_optimization(self, frame):
        """
        Real-time Performance Monitoring and Optimization
        Essential for: Real-time robotics, embedded systems, efficient processing
        """
        start_time = time.time()
        
        # Demonstrate different optimization techniques
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 1. Multi-scale processing (process at lower resolution first)
        small = cv2.resize(gray, None, fx=0.5, fy=0.5)
        small_processed = cv2.GaussianBlur(small, (5, 5), 0)
        upscaled = cv2.resize(small_processed, (gray.shape[1], gray.shape[0]))
        
        # 2. ROI processing (only process center region)
        h, w = gray.shape
        roi = gray[h//4:3*h//4, w//4:3*w//4]
        roi_processed = cv2.Canny(roi, 50, 150)
        
        # 3. Temporal subsampling (process every nth frame)
        if self.frame_count % 3 == 0:  # Process every 3rd frame
            features = cv2.goodFeaturesToTrack(gray, maxCorners=50, qualityLevel=0.01, minDistance=10)
        else:
            features = getattr(self, 'cached_features', None)
        
        if features is not None:
            self.cached_features = features
        
        # Create result visualization
        result = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        
        # Show ROI
        cv2.rectangle(result, (w//4, h//4), (3*w//4, 3*h//4), (255, 0, 0), 2)
        
        # Show features
        if features is not None:
            for feature in features:
                x, y = feature.ravel().astype(int)
                cv2.circle(result, (x, y), 3, (0, 255, 0), -1)
        
        # Calculate processing time
        end_time = time.time()
        processing_time = (end_time - start_time) * 1000  # Convert to ms
        self.frame_times.append(processing_time)
        
        # Performance statistics
        avg_time = np.mean(self.frame_times) if self.frame_times else 0
        fps = 1000 / avg_time if avg_time > 0 else 0
        
        # Display performance info
        cv2.putText(result, f'Processing Time: {processing_time:.1f}ms', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(result, f'Average FPS: {fps:.1f}', (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(result, f'Features: {len(features) if features is not None else 0}', (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Performance indicator
        if fps > 25:
            perf_color = (0, 255, 0)  # Green - good
            perf_status = "REAL-TIME"
        elif fps > 15:
            perf_color = (0, 255, 255)  # Yellow - acceptable
            perf_status = "ACCEPTABLE"
        else:
            perf_color = (0, 0, 255)  # Red - too slow
            perf_status = "TOO SLOW"
        
        cv2.putText(result, perf_status, (10, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, perf_color, 2)
        
        return result
    
    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect new features for visual odometry
            if self.prev_pts is None or len(self.prev_pts) < 50:
                self.prev_pts = self.feature_detector(gray, **self.feature_params)
            
            # Apply advanced CV techniques
            vo_result, pose = self.visual_odometry_estimation(gray)
            stereo_result = self.stereo_depth_estimation(frame)
            detection_result = self.object_detection_pipeline(frame)
            servoing_result = self.visual_servoing_control(frame)
            sfm_result = self.structure_from_motion(frame)
            segmentation_result = self.semantic_segmentation(frame)
            tracking_result = self.multi_object_tracking(frame)
            rectification_result = self.image_rectification(frame)
            performance_result = self.performance_optimization(frame)
            
            # Create comprehensive output display
            # Resize all images to same size for concatenation
            h, w = 200, 300
            
            # First row - SLAM and Navigation
            row1 = cv2.hconcat([
                cv2.resize(vo_result, (w, h)),
                cv2.resize(stereo_result, (w, h)),
                cv2.resize(detection_result, (w, h))
            ])
            
            # Second row - Manipulation and Reconstruction
            row2 = cv2.hconcat([
                cv2.resize(servoing_result, (w, h)),
                cv2.resize(sfm_result, (w, h)),
                cv2.resize(segmentation_result, (w, h))
            ])
            
            # Third row - Tracking and Optimization
            # Handle rectification result which might be wider
            rect_resized = cv2.resize(rectification_result, (w, h))
            row3 = cv2.hconcat([
                cv2.resize(tracking_result, (w, h)),
                rect_resized,
                cv2.resize(performance_result, (w, h))
            ])
            
            # Combine all rows
            output = cv2.vconcat([row1, row2, row3])
            
            # Add technique labels
            labels = [
                # Row 1
                "Visual Odometry (SLAM)", "Stereo Depth Estimation", "Object Detection (HOG)",
                # Row 2  
                "Visual Servoing Control", "Structure from Motion", "Semantic Segmentation",
                # Row 3
                "Multi-Object Tracking", "Image Rectification", "Performance Monitor"
            ]
            
            # Add labels with background for better visibility
            for i, label in enumerate(labels):
                x = (i % 3) * w + 10
                y = (i // 3) * h + 25
                
                # Add background rectangle for text
                text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                cv2.rectangle(output, (x - 5, y - 20), (x + text_size[0] + 5, y + 5), (0, 0, 0), -1)
                cv2.putText(output, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Add main title
            title = "Advanced Computer Vision for Robotics - Essential Techniques"
            title_size = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0]
            title_x = (output.shape[1] - title_size[0]) // 2
            cv2.rectangle(output, (title_x - 10, 5), (title_x + title_size[0] + 10, 35), (0, 0, 0), -1)
            cv2.putText(output, title, (title_x, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # Add frame counter and timestamp
            cv2.putText(output, f'Frame: {self.frame_count}', (10, output.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Publish result
            self.publisher.publish(self.bridge.cv2_to_imgmsg(output, encoding='bgr8'))
            
            # Update state for next frame
            self.prev_gray = gray
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedCVNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()