import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
import numpy as np
from brisque.brisque import BRISQUE
from collections import deque
import numpy as np

class SensorConfidence(Node):
    def __init__(self):
        super().__init__("sensor_confidence_node")
        self.cam_sub = self.create_subscription(Image,"/camera/image_raw",self.image_callback,10)
        self.lidar_sub = self.create_subscription(PointCloud2,"/points",self.lidar_callback,10)
        self.imu_sub = self.create_subscription(Imu,"/imu_raw",self.imu_callback,10)
        self.gps_sub = self.create_subscription(NavSatFix,"/navsat",self.gps_callback,10)
        self.bridge = CvBridge()
        self.imu_data_buffer = deque(maxlen=1000)  

    def map_image_conf(self,score):
        if(score<=20.0):
            return 1.0
        elif(score>=100):
            return 0.0
        else:
            return 1.0-(score-20)/80


    def image_callback(self,msg):
        frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        obj = BRISQUE(url=False)
        image_score = obj.score(frame)
        cam_confidence = self.map_image_conf(image_score)
        self.get_logger().info(f"Image confidence score : {cam_confidence}")


    def lidar_callback(self, msg: PointCloud2):
        points = self.pointcloud2_to_numpy(msg)
        
        if points is None or len(points) < 100:
            self.get_logger().info("LIDAR Confidence: 0.000 (insufficient points)")
            return 0.0
        
        density_conf = self.calculate_point_density_quality(points)
        noise_conf = self.calculate_noise_quality(points)
        outlier_conf = self.calculate_outlier_quality(points)
        consistency_conf = self.calculate_surface_consistency(points)
        
        lidar_confidence = (density_conf * 0.4 + 
                        noise_conf * 0.2 + 
                        outlier_conf * 0.2 + 
                        consistency_conf * 0.2)
        
        self.get_logger().info(
            f"LiDAR Confidence score : {lidar_confidence:.3f} "
            f"(Density: {density_conf:.2f}, Noise: {noise_conf:.2f}, "
            f"Outliers: {outlier_conf:.2f}, Consistency: {consistency_conf:.2f})"
        )
        return lidar_confidence


    def pointcloud2_to_numpy(self, cloud_msg):
        try:
            cloud_data = cloud_msg.data
            dtype_list = []
            for field in cloud_msg.fields:
                if field.name in ['x', 'y', 'z']:
                    dtype_list.append((field.name, np.float32))
            if len(dtype_list) < 3:
                return None
            
            cloud_arr = np.frombuffer(cloud_data, dtype=dtype_list)
            points = np.column_stack([cloud_arr['x'], cloud_arr['y'], cloud_arr['z']])
            valid_mask = np.isfinite(points).all(axis=1)
            return points[valid_mask]
        except Exception as e:
            self.get_logger().error(f"Failed to convert PointCloud2: {e}")
            return None


    def calculate_point_density_quality(self, points, voxel_size=0.5):
        if len(points) < 50:
            return 0.0
        min_coords = np.min(points, axis=0)
        voxel_indices = ((points - min_coords) / voxel_size).astype(int)
        _, counts = np.unique(voxel_indices, axis=0, return_counts=True)
        if len(counts) < 5:
            return 0.5
        mean_density = np.mean(counts)
        std_density = np.std(counts)
        if mean_density < 1e-6:
            return 0.1
        cv = std_density / mean_density
        if cv < 0.5:
            density_confidence = 1.0 - (cv / 0.5) * 0.3
        elif cv < 2.0:
            density_confidence = 0.7 - ((cv - 0.5) / 1.5) * 0.5
        else:
            density_confidence = 0.2 / (1.0 + (cv - 2.0))
        return np.clip(density_confidence, 0.0, 1.0)


    def calculate_noise_quality(self, points):
        from scipy.spatial import cKDTree
        if len(points) < 100:
            return 0.0
        max_points = 1000
        indices = np.random.choice(len(points), max_points, replace=False) if len(points) > max_points else np.arange(len(points))
        sample_points = points[indices]
        tree = cKDTree(sample_points)
        roughness_values = []
        for i in range(min(200, len(sample_points))):
            distances, neighbors_idx = tree.query(sample_points[i], k=min(10, len(sample_points)-1)+1)
            neighbor_pts = sample_points[neighbors_idx[1:]]
            if len(neighbor_pts) < 3:
                continue
            centered = neighbor_pts - np.mean(neighbor_pts, axis=0)
            try:
                _, _, vh = np.linalg.svd(centered)
                normal = vh[-1]
                center = np.mean(neighbor_pts, axis=0)
                dists_to_plane = np.abs(np.dot(neighbor_pts - center, normal))
                roughness = np.std(dists_to_plane)
                roughness_values.append(roughness)
            except np.linalg.LinAlgError:
                continue
        if not roughness_values:
            return 0.5
        avg_roughness = np.mean(roughness_values)
        noise_threshold = 0.05
        noise_confidence = np.exp(-avg_roughness / noise_threshold)
        return np.clip(noise_confidence, 0.0, 1.0)


    def calculate_outlier_quality(self, points):
        if len(points) < 50:
            return 0.0
        centroid = np.mean(points, axis=0)
        distances = np.linalg.norm(points - centroid, axis=1)
        Q1 = np.percentile(distances, 25)
        Q3 = np.percentile(distances, 75)
        IQR = Q3 - Q1
        lower_bound = Q1 - 1.5 * IQR
        upper_bound = Q3 + 1.5 * IQR
        outliers = np.sum((distances < lower_bound) | (distances > upper_bound))
        outlier_ratio = outliers / len(points)
        return 1.0 - np.clip(outlier_ratio, 0.0, 1.0)


    def calculate_surface_consistency(self, points):
        from scipy.spatial import cKDTree
        if len(points) < 100:
            return 0.0
        max_points = 500
        indices = np.random.choice(len(points), max_points, replace=False) if len(points) > max_points else np.arange(len(points))
        sample_points = points[indices]
        tree = cKDTree(sample_points)
        normal_consistency = []
        eps = 1e-8
        for i in range(min(100, len(sample_points))):
            distances, neighbors_idx = tree.query(sample_points[i], k=min(8, len(sample_points)-1)+1)
            neighbor_pts = sample_points[neighbors_idx[1:]]
            if len(neighbor_pts) < 3:
                continue
            centered = neighbor_pts - np.mean(neighbor_pts, axis=0)
            try:
                _, s, _ = np.linalg.svd(centered)
                eigen_sum = np.sum(s)
                if eigen_sum < eps:
                    continue
                eigenvalues = s / (eigen_sum + eps)
                planarity = (eigenvalues[1] - eigenvalues[2]) / eigenvalues[0] if eigenvalues[0] > 0 else 0
                normal_consistency.append(planarity)
            except np.linalg.LinAlgError:
                continue
        if not normal_consistency:
            return 0.5
        avg_consistency = np.mean(normal_consistency)
        return np.clip(avg_consistency, 0.0, 1.0)

    def imu_callback(self,msg):
        angular_rate_z = msg.angular_velocity.z 
        self.imu_data_buffer.append(angular_rate_z)
        allan_var = self.compute_allan_variance(list(self.imu_data_buffer))
        imu_score = self.imu_allan_confidence_score(allan_var)
        self.get_logger().info(f"IMU Confidence Score  {imu_score}")


    def compute_allan_variance(self, data, tau=1):
        N = len(data)
        if N < 2 * tau:
            return 0.0

        data = np.array(data)
        segment_count = N // tau
        means = [np.mean(data[i * tau:(i + 1) * tau]) for i in range(segment_count)]
        means = np.array(means)
        diffs = means[1:] - means[:-1]
        allan_var = 0.5 * np.mean(diffs ** 2)
        return allan_var


    def imu_allan_confidence_score(self,allan_var):
        low_thresh = 1e-7
        high_thresh = 1e-4
        if allan_var < low_thresh:
            return 100.0
        elif allan_var > high_thresh:
            return 0.0
        else:
            score = 100.0 * (high_thresh - allan_var) / (high_thresh - low_thresh)
            return max(0.0, min(100.0, score))



    def gps_callback(self, msg):
        pos_cov = msg.position_covariance
        hdop_estimate = np.sqrt(pos_cov[0] + pos_cov[4]) 
        vdop_estimate = np.sqrt(pos_cov[8])  
        pdop_estimate = np.sqrt(pos_cov[0] + pos_cov[4] + pos_cov[8]) 
        
        status_confidence = min(msg.status.status / 2.0, 1.0) if msg.status.status >= 0 else 0.0

        hdop_conf = 1.0 / (1.0 + hdop_estimate / 2.0) if hdop_estimate > 0 else 0.0
        vdop_conf = 1.0 / (1.0 + vdop_estimate / 2.0) if vdop_estimate > 0 else 0.0
        pdop_conf = 1.0 / (1.0 + pdop_estimate / 2.0) if pdop_estimate > 0 else 0.0
        

        accuracy = np.sqrt(pos_cov[0] + pos_cov[4] + pos_cov[8])
        cn0_conf = 1.0 / (1.0 + accuracy / 5.0) if accuracy > 0 else 0.0 
        
        dop_confidence = (hdop_conf + vdop_conf + pdop_conf) / 3.0  
        gps_confidence = (status_confidence * 0.25 + 
                        dop_confidence * 0.5 + 
                        cn0_conf * 0.25)
        
        self.get_logger().info(f"GPS Confidence score: {gps_confidence:.3f} " f"(Status: {status_confidence:.2f}, "f"HDOP: {hdop_conf:.2f}, VDOP: {vdop_conf:.2f}, "f"PDOP: {pdop_conf:.2f}, C/N0: {cn0_conf:.2f})")
        
        return gps_confidence
    

def main(args=None):
    rclpy.init(args=args)
    node = SensorConfidence()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()