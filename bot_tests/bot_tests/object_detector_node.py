#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


class YoloAnnotator(Node):
    def __init__(self):
        super().__init__('object_detector_node')

        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/yolo_labeled')
        default_model_path = self._resolve_default_model_path()
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('confidence', 0.25)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence = self.get_parameter('confidence').get_parameter_value().double_value

        self.bridge = self._create_bridge()
        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10,
        )

        self.model = YOLO(model_path)
        self.get_logger().info(
            f'YOLO annotator listening on {input_topic} and publishing labeled frames to {output_topic}'
        )
        self.get_logger().info(f'Loaded model: {model_path}')

    def _create_bridge(self):
        try:
            from cv_bridge import CvBridge
        except Exception as exc:
            raise RuntimeError(
                'Failed to import cv_bridge'
            ) from exc

        return CvBridge()

    def _resolve_default_model_path(self) -> str:
        package_model_path = Path(get_package_share_directory('bot_tests')) / 'models' / 'yolov8n.pt'
        if package_model_path.is_file():
            return str(package_model_path)

        source_model_path = Path(__file__).resolve().parents[1] / 'models' / 'yolov8n.pt'
        if source_model_path.is_file():
            return str(source_model_path)

        raise FileNotFoundError(
            'Model weights not found'
        )

    def image_callback(self, msg: Image) -> None:
        from cv_bridge import CvBridgeError

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'Failed to convert input image: {exc}')
            return

        try:
            results = self.model(frame, conf=self.confidence, verbose=False)
            annotated = results[0].plot()
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'Failed to convert annotated image: {exc}')
            return

        annotated_msg.header = msg.header
        self.publisher.publish(annotated_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloAnnotator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
