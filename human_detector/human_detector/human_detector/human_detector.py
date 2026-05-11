import os
from math import tan

import cv2
from cv_bridge.core import CvBridge
from geometry_msgs.msg import TransformStamped
from human_detector.human_detector_parameters import human_detector_parameters
from image_geometry import PinholeCameraModel
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision
from mediapipe.tasks.python.vision import PoseLandmarker, PoseLandmarkerOptions, RunningMode
from mediapipe.tasks.python.components.containers.landmark import NormalizedLandmark
import rclpy
from rclpy.time import Time
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos_overriding_options import QoSOverridingOptions
from rclpy.qos import qos_profile_sensor_data
from tf2_ros.transform_broadcaster import TransformBroadcaster
from message_filters import ApproximateTimeSynchronizer, Subscriber

_LEFT_HIP_INDEX = 23
_RIGHT_HIP_INDEX = 24

class HumanDetector(LifecycleNode):
    def __init__(self):
        super().__init__("human_detector")
        self.param_listener = human_detector_parameters.ParamListener(self)
        self.depth_image: Image = None
        self.image = None
        self.detected_landmarks = None
        self.detected_human_position_world = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.cv_bridge = CvBridge()
        self.model = PinholeCameraModel()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.person_pose_estimator = None
        self.camera_info = None

    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_configure")
        self.parameters = self.param_listener.get_params()
        if "optical" in self.parameters.camera_frame_id:
            self.get_logger().warn(
                "camera_frame_id points to an optical frame. This node publishes a planar target "
                "(x forward, y left, z 0), so a non-optical camera body frame such as 'rgbd_camera' "
                "should be used."
            )
        self.log_parameters()
        self.time_approximation_slope = self.parameters.time_approximation_slope

        if not self.parameters.pose_landmarker_model_path:
            self.get_logger().error("Parameter 'pose_landmarker_model_path' must point to a MediaPipe .task model.")
            return TransitionCallbackReturn.FAILURE

        if not os.path.exists(self.parameters.pose_landmarker_model_path):
            self.get_logger().error(
                f"Pose landmarker model not found: {self.parameters.pose_landmarker_model_path}"
            )
            return TransitionCallbackReturn.FAILURE

        base_options = mp_python.BaseOptions(
            model_asset_path=self.parameters.pose_landmarker_model_path
        )
        options = PoseLandmarkerOptions(
            base_options=base_options,
            running_mode=RunningMode.IMAGE,
            min_pose_detection_confidence=self.parameters.min_detection_confidence,
            min_tracking_confidence=self.parameters.min_tracking_confidence,
            min_pose_presence_confidence=self.parameters.min_detection_confidence,
        )
        self.person_pose_estimator = PoseLandmarker.create_from_options(options)
        # ---------------------------

        self.initialize_sync_subscribers()

        if self.parameters.publish_image_with_detected:
            self.image_with_detected_human_pub = self.create_publisher(Image, "image_with_detected_human", 10)
        self.timer = self.create_timer(1 / self.parameters.detected_human_transform_frequency, self.timer_callback)
        self.timer.cancel()

        return TransitionCallbackReturn.SUCCESS

    def initialize_sync_subscribers(self):
        sync_topics = [
            Subscriber(
                self,
                Image,
                self.parameters.rgb_image_topic,
                qos_profile=qos_profile_sensor_data,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),
            Subscriber(
                self,
                Image,
                self.parameters.depth_image_topic,
                qos_profile=qos_profile_sensor_data,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),
            Subscriber(
                self,
                CameraInfo,
                self.parameters.camera_info_topic,
                qos_profile=qos_profile_sensor_data,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),
        ]

        self.image_approx_time_sync = ApproximateTimeSynchronizer(
            sync_topics,
            queue_size=5,
            slop=self.time_approximation_slope,
        )
        self.image_approx_time_sync.registerCallback(self.on_image_data)

    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_activate")
        self.timer.reset()
        return super().on_activate(previous_state)

    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_deactivate")
        self.timer.cancel()
        return super().on_deactivate(previous_state)

    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_cleanup")
        self.destroy_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_shutdown")
        self.destroy_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_error")
        self.destroy_resources()
        return TransitionCallbackReturn.SUCCESS

    def destroy_resources(self):
        self.destroy_timer(self.timer)
        if self.person_pose_estimator is not None:
            self.person_pose_estimator.close()

    def log_parameters(self):
        self.get_logger().info(f"Human detector uses model: {self.parameters.pose_landmarker_model_path}.")
        if self.parameters.horizontal_fov > 0.0:
            self.get_logger().info(
                f"Human detector uses manual horizontal_fov: {self.parameters.horizontal_fov} rad."
            )
        self.get_logger().info(f"Human detector uses: {self.parameters.camera_frame_id} as camera link.")
        self.get_logger().info(f"Human detector subscribes rgb image: {self.parameters.rgb_image_topic}.")
        self.get_logger().info(f"Human detector subscribes depth image: {self.parameters.depth_image_topic}.")
        self.get_logger().info(f"Human detector subscribes camera info: {self.parameters.camera_info_topic}.")
        self.get_logger().info(f"Human detector uses: {self.parameters.detected_human_frame_id} as frame with human.")
        self.get_logger().info(
            "Human detector publishes transform to detected human with "
            f"{self.parameters.detected_human_transform_frequency} Hz."
        )
        self.get_logger().info(
            f"Mediapipe will use {self.parameters.min_detection_confidence} " "as min_detection_confidence"
        )

        self.get_logger().info(
            f"Mediapipe will use {self.parameters.min_tracking_confidence} " "as min_tracking_confidence"
        )
        if self.parameters.publish_image_with_detected:
            self.get_logger().info("Human detector will publish image with detected human.")

    def on_image_data(self, image: Image, depth_image: Image, info: CameraInfo):
        self.camera_info = info
        self.model.fromCameraInfo(self.camera_info)
        self.image_time_stamp = Time.from_msg(image.header.stamp)
        self.image = self.decode_rgb_image(image)
        self.depth_image = self.decode_depth_image(depth_image)
        self.store_human_pose()

    def decode_rgb_image(self, image: Image):
        if image.encoding == "rgb8":
            return self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")

        if image.encoding == "bgr8":
            bgr_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            return cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)

        if image.encoding == "rgba8":
            rgba_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="rgba8")
            return cv2.cvtColor(rgba_image, cv2.COLOR_RGBA2RGB)

        if image.encoding == "bgra8":
            bgra_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgra8")
            return cv2.cvtColor(bgra_image, cv2.COLOR_BGRA2RGB)

        self.get_logger().warn(
            f"Unsupported RGB encoding '{image.encoding}'. Attempting passthrough conversion."
        )
        rgb_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        if rgb_image.ndim == 2:
            return cv2.cvtColor(rgb_image, cv2.COLOR_GRAY2RGB)
        return rgb_image

    def decode_depth_image(self, depth_image: Image):
        if depth_image.encoding == "32FC1":
            return self.cv_bridge.imgmsg_to_cv2(depth_image, desired_encoding="32FC1")

        if depth_image.encoding in ("16UC1", "mono16"):
            depth_mm = self.cv_bridge.imgmsg_to_cv2(depth_image, desired_encoding="16UC1")
            return depth_mm.astype(np.float32) / 1000.0

        self.get_logger().warn(
            f"Unsupported depth encoding '{depth_image.encoding}'. Attempting passthrough conversion."
        )
        depth = self.cv_bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
        return depth.astype(np.float32)

    def are_rgb_image_same_size_as_depth_image(self):
        rgb_image_height, rgb_image_width, _ = self.image.shape
        depth_image_height, depth_image_width = self.depth_image.shape

        return rgb_image_height == depth_image_height and rgb_image_width == depth_image_width

    def should_detect_human(self):
        if self.camera_info is None or self.image is None or self.depth_image is None:
            self.get_logger().error(
                "No camera info or image or depth image are not stored. Human will not be detected."
            )
            return False

        if not self.are_rgb_image_same_size_as_depth_image():
            self.get_logger().error(
                "Dimensions of rgb image and depth image are not equal. Human will not be detected."
            )
            return False

        return True

    def store_human_pose(self):
        if not self.should_detect_human():
            return

        # --- MediaPipe Tasks API: wrap numpy array in mp.Image then detect ---
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=self.image)
        result = self.person_pose_estimator.detect(mp_image)

        # Store the first detected person's landmarks (list of NormalizedLandmark),
        # or None when nobody is detected — same semantics as the old pose_landmarks.
        self.detected_landmarks = result.pose_landmarks[0] if result.pose_landmarks else None
        # ---------------------------------------------------------------------

        x_pos_of_detected_person, y_pos_of_detected_person = self.get_position_of_human_in_the_image(
            self.detected_landmarks
        )

        self.get_3d_human_position(x_pos_of_detected_person, y_pos_of_detected_person)

    def get_3d_human_position(self, x_pos_of_detected_person, y_pos_of_detected_person):
        if x_pos_of_detected_person <= 0 or y_pos_of_detected_person <= 0:
            return

        depth_of_given_pixel = self.get_valid_depth_around_pixel(x_pos_of_detected_person, y_pos_of_detected_person)
        if not np.isfinite(depth_of_given_pixel) or depth_of_given_pixel <= 0.0:
            return

        point_xyz = self.project_pixel_to_3d(
            x_pos_of_detected_person,
            y_pos_of_detected_person,
            depth_of_given_pixel,
        )
        self.detected_human_position_world = {
            "x": point_xyz[2],
            "y": -point_xyz[0],
            "z": -point_xyz[1],
        }

    def get_valid_depth_around_pixel(self, x_pos_of_detected_person, y_pos_of_detected_person):
        height, width = self.depth_image.shape
        x_min = max(0, x_pos_of_detected_person - 2)
        x_max = min(width, x_pos_of_detected_person + 3)
        y_min = max(0, y_pos_of_detected_person - 2)
        y_max = min(height, y_pos_of_detected_person + 3)

        depth_window = self.depth_image[y_min:y_max, x_min:x_max]
        valid_depth = depth_window[np.isfinite(depth_window) & (depth_window > 0.0)]
        if valid_depth.size == 0:
            return float("nan")

        return float(np.median(valid_depth))

    def project_pixel_to_3d(self, x_pos_of_detected_person, y_pos_of_detected_person, depth_of_given_pixel):
        if self.parameters.horizontal_fov > 0.0:
            return self.project_pixel_to_3d_with_manual_fov(
                x_pos_of_detected_person,
                y_pos_of_detected_person,
                depth_of_given_pixel,
            )

        ray = self.model.projectPixelTo3dRay((x_pos_of_detected_person, y_pos_of_detected_person))
        ray_3d = [ray_element / ray[2] for ray_element in ray]
        return [ray_element * depth_of_given_pixel for ray_element in ray_3d]

    def project_pixel_to_3d_with_manual_fov(self, x_pos_of_detected_person, y_pos_of_detected_person, depth):
        height, width, _ = self.image.shape
        focal_length = width / (2.0 * tan(self.parameters.horizontal_fov / 2.0))
        center_x = (width - 1) / 2.0
        center_y = (height - 1) / 2.0

        x_optical = ((x_pos_of_detected_person - center_x) / focal_length) * depth
        y_optical = ((y_pos_of_detected_person - center_y) / focal_length) * depth
        z_optical = depth
        return [x_optical, y_optical, z_optical]

    def get_position_of_human_in_the_image(self, landmarks):
        x, y = 0, 0
        if self.detected_landmarks:
            # --- MediaPipe Tasks API: landmarks is already a plain list of
            #     NormalizedLandmark objects; index directly by integer constant ---
            left_hip_landmark = self.detected_landmarks[_LEFT_HIP_INDEX]
            right_hip_landmark = self.detected_landmarks[_RIGHT_HIP_INDEX]
            # ---------------------------------------------------------------------
            x, y = self.extract_hip_midpoint(left_hip_landmark, right_hip_landmark)

        return x, y

    def extract_hip_midpoint(self, left_hip_landmark, right_hip_landmark):
        height, width, _ = self.image.shape
        x = int(min((left_hip_landmark.x * width + right_hip_landmark.x * width) / 2, width - 1))
        y = int(min((left_hip_landmark.y * height + right_hip_landmark.y * height) / 2, height - 1))
        return x, y

    def timer_callback(self):
        if self.detected_human_position_world["x"] > 0.0:
            self.broadcast_timer_callback()
        self.publish_image_with_detected_human()

    def broadcast_timer_callback(self):
        transform = TransformStamped()
        transform.header.stamp = self.image_time_stamp.to_msg()
        transform.header.frame_id = self.parameters.camera_frame_id
        transform.child_frame_id = self.parameters.detected_human_frame_id
        transform.transform.translation.x = self.detected_human_position_world["x"]
        transform.transform.translation.y = self.detected_human_position_world["y"]
        transform.transform.translation.z = self.detected_human_position_world["z"]
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def draw_person_pose(self, image):
        if image is None or self.detected_landmarks is None:
            return image

        height, width, _ = image.shape

        for landmark in self.detected_landmarks:
            x = int(min(max(landmark.x * width, 0), width - 1))
            y = int(min(max(landmark.y * height, 0), height - 1))
            cv2.circle(image, (x, y), 3, (0, 255, 0), -1)

        left_hip_landmark = self.detected_landmarks[_LEFT_HIP_INDEX]
        right_hip_landmark = self.detected_landmarks[_RIGHT_HIP_INDEX]
        hip_midpoint = self.extract_hip_midpoint(left_hip_landmark, right_hip_landmark)
        cv2.circle(image, hip_midpoint, 6, (0, 0, 255), -1)

        return image

    def publish_image_with_detected_human(self):
        if not self.parameters.publish_image_with_detected:
            return

        if self.detected_landmarks is not None:
            modified_image_msg = self.cv_bridge.cv2_to_imgmsg(self.draw_person_pose(self.image.copy()))
            self.image_with_detected_human_pub.publish(modified_image_msg)
        elif self.image is not None:
            image_msg = self.cv_bridge.cv2_to_imgmsg(self.image)
            self.image_with_detected_human_pub.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    pose_detector = HumanDetector()
    rclpy.spin(pose_detector)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
