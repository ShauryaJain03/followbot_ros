"""Adapt LIO-SAM mapping poses and raw lidar scans to traversability keyframes.

The external traversability_mapping package expects explicit, immutable
keyframes.  LIO-SAM publishes the corresponding mapping odometry and raw scan,
but its stock ground-truth keyframe simulator also broadcasts TF.  This adapter
deliberately publishes *only* KeyFrameAdditions, leaving LIO-SAM as the sole
owner of the map -> odom -> base_link transform chain.
"""

from __future__ import annotations

import math

import message_filters
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener
from traversability_msgs.msg import KeyFrame, KeyFrameAdditions


def _rotate(vector: tuple[float, float, float], quaternion) -> tuple[float, float, float]:
    """Rotate a vector by a geometry_msgs Quaternion without a TF broadcaster."""
    x, y, z = vector
    qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def _multiply_quaternions(lhs, rhs):
    """Return lhs * rhs as a geometry_msgs Quaternion instance."""
    result = lhs.__class__()
    result.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y
    result.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x
    result.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w
    result.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z
    return result


class LioTraversabilityKeyframeBridge(Node):
    """Create map-frame terrain keyframes without publishing TF or odometry."""

    def __init__(self) -> None:
        super().__init__('lio_traversability_keyframe_bridge')
        self.declare_parameter('cloud_topic', '/velodyne_points')
        self.declare_parameter('mapping_odom_topic', '/lio_sam/mapping/odometry')
        self.declare_parameter('additions_topic', '/traversability_keyframe_additions')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('keyframe_min_displacement_m', 0.25)
        self.declare_parameter('synchronizer_slop_s', 0.15)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.minimum_displacement = float(
            self.get_parameter('keyframe_min_displacement_m').value)
        synchronizer_slop = float(self.get_parameter('synchronizer_slop_s').value)
        self.keyframe_id = 0
        self.last_keyframe_position: tuple[float, float, float] | None = None

        self.publisher = self.create_publisher(
            KeyFrameAdditions, self.get_parameter('additions_topic').value, 10)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cloud_subscriber = message_filters.Subscriber(
            self, PointCloud2, self.get_parameter('cloud_topic').value,
            qos_profile=qos_profile_sensor_data)
        self.odom_subscriber = message_filters.Subscriber(
            self, Odometry, self.get_parameter('mapping_odom_topic').value)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.cloud_subscriber, self.odom_subscriber], queue_size=40,
            slop=synchronizer_slop)
        self.synchronizer.registerCallback(self._synchronised_callback)
        self.get_logger().info(
            'Publishing terrain keyframes from '
            f'{self.get_parameter("cloud_topic").value} + '
            f'{self.get_parameter("mapping_odom_topic").value}; no TF is published.')

    def _pose_in_map(self, odometry: Odometry) -> Pose | None:
        source_frame = odometry.header.frame_id
        pose = odometry.pose.pose
        if source_frame == self.map_frame:
            return pose
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, source_frame, Time.from_msg(odometry.header.stamp),
                timeout=Duration(seconds=0.1))
        except TransformException as error:
            self.get_logger().warn(
                f'No transform {self.map_frame} <- {source_frame}; skipping keyframe: {error}',
                throttle_duration_sec=2.0)
            return None

        rotated = _rotate(
            (pose.position.x, pose.position.y, pose.position.z),
            transform.transform.rotation)
        result = Pose()
        result.position.x = rotated[0] + transform.transform.translation.x
        result.position.y = rotated[1] + transform.transform.translation.y
        result.position.z = rotated[2] + transform.transform.translation.z
        result.orientation = _multiply_quaternions(transform.transform.rotation, pose.orientation)
        return result

    def _synchronised_callback(self, cloud: PointCloud2, odometry: Odometry) -> None:
        pose = self._pose_in_map(odometry)
        if pose is None:
            return
        position = (pose.position.x, pose.position.y, pose.position.z)
        if self.last_keyframe_position is not None:
            displacement = math.dist(position, self.last_keyframe_position)
            if displacement < self.minimum_displacement:
                return

        self.keyframe_id += 1
        keyframe = KeyFrame()
        keyframe.kf_timestamp_in_nanosec = Time.from_msg(cloud.header.stamp).nanoseconds
        keyframe.kf_id = self.keyframe_id
        keyframe.kf_pose = pose
        keyframe.kf_pointcloud = cloud
        keyframe.map_id = 0
        additions = KeyFrameAdditions()
        additions.keyframes.append(keyframe)
        self.publisher.publish(additions)
        self.last_keyframe_position = position


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LioTraversabilityKeyframeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
