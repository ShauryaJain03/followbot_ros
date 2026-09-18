"""Build conservative local and map-frame terrain cost grids from LiDAR clouds."""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass, field
from typing import DefaultDict, Iterable

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


UNKNOWN = -1
FREE = 0
ROUGH = 35
CAUTION = 65
LETHAL = 100


@dataclass
class CellSamples:
    """Bounded elevation samples for one grid cell."""

    heights: list[float] = field(default_factory=list)

    def extend(self, values: Iterable[float], limit: int) -> None:
        self.heights.extend(float(value) for value in values)
        if len(self.heights) > limit:
            self.heights = self.heights[-limit:]


class ElevationGrid:
    """A compact 2.5D grid whose cells retain only elevation samples."""

    def __init__(
        self,
        resolution: float,
        width_m: float,
        height_m: float,
        origin_x: float,
        origin_y: float,
        max_samples: int,
    ) -> None:
        self.resolution = resolution
        self.width = int(math.ceil(width_m / resolution))
        self.height = int(math.ceil(height_m / resolution))
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.max_samples = max_samples
        self.cells: DefaultDict[tuple[int, int], CellSamples] = defaultdict(CellSamples)

    def index_of(self, x: float, y: float) -> tuple[int, int] | None:
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= col < self.width and 0 <= row < self.height:
            return row, col
        return None

    def add_points(self, points: np.ndarray) -> None:
        """Insert XYZ points, discarding those outside this grid."""
        grouped: DefaultDict[tuple[int, int], list[float]] = defaultdict(list)
        for x, y, z in points:
            index = self.index_of(float(x), float(y))
            if index is not None:
                grouped[index].append(float(z))
        for index, heights in grouped.items():
            self.cells[index].extend(heights, self.max_samples)

    def classify(
        self,
        min_points: int,
        slope_limit: float,
        step_limit: float,
        roughness_limit: float,
        obstacle_height: float,
    ) -> np.ndarray:
        """Return OccupancyGrid-compatible terrain costs in row-major order."""
        costs = np.full((self.height, self.width), UNKNOWN, dtype=np.int16)
        ground = np.full((self.height, self.width), np.nan, dtype=np.float64)
        spread = np.full((self.height, self.width), np.nan, dtype=np.float64)

        for (row, col), cell in self.cells.items():
            if len(cell.heights) < min_points:
                continue
            samples = np.asarray(cell.heights, dtype=np.float64)
            # Low percentile is robust to isolated low returns while retaining ground.
            ground[row, col] = float(np.percentile(samples, 15.0))
            spread[row, col] = float(np.percentile(samples, 90.0) - np.percentile(samples, 10.0))

        for row in range(self.height):
            for col in range(self.width):
                if not math.isfinite(ground[row, col]):
                    continue

                neighbour_deltas = []
                for d_row, d_col in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    n_row, n_col = row + d_row, col + d_col
                    if 0 <= n_row < self.height and 0 <= n_col < self.width:
                        if math.isfinite(ground[n_row, n_col]):
                            neighbour_deltas.append(abs(ground[n_row, n_col] - ground[row, col]))

                max_delta = max(neighbour_deltas, default=0.0)
                slope = math.atan2(max_delta, self.resolution)
                roughness = spread[row, col]

                if roughness >= obstacle_height:
                    costs[row, col] = LETHAL
                elif slope > slope_limit or max_delta > step_limit:
                    costs[row, col] = LETHAL
                else:
                    severity = max(
                        slope / max(slope_limit, 1e-6),
                        max_delta / max(step_limit, 1e-6),
                        roughness / max(roughness_limit, 1e-6),
                    )
                    if severity >= 0.85:
                        costs[row, col] = CAUTION
                    elif severity >= 0.45:
                        costs[row, col] = ROUGH
                    else:
                        costs[row, col] = FREE
        return costs


def transform_points(points: np.ndarray, transform) -> np.ndarray:
    """Apply a TransformStamped to an Nx3 point array."""
    rotation = transform.transform.rotation
    translation = transform.transform.translation
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    matrix = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    offset = np.array([translation.x, translation.y, translation.z])
    return points @ matrix.T + offset


class TraversabilityAnalyzer(Node):
    """Publish rolling, accumulated, and fused terrain maps from LiDAR data."""

    def __init__(self) -> None:
        super().__init__('traversability_analyzer')
        self._declare_parameters()
        self.resolution = float(self.get_parameter('grid_resolution').value)
        self.local_frame = self.get_parameter('local_frame_id').value
        self.map_frame = self.get_parameter('map_frame_id').value
        self.max_samples = int(self.get_parameter('max_samples_per_cell').value)
        self.slope_limit = math.radians(float(self.get_parameter('slope_limit_deg').value))
        self.step_limit = float(self.get_parameter('step_height_limit_m').value)
        self.roughness_limit = float(self.get_parameter('roughness_limit_m').value)
        self.obstacle_height = float(self.get_parameter('obstacle_height_m').value)
        self.min_points = int(self.get_parameter('min_points_per_cell').value)
        self.max_range = float(self.get_parameter('max_sensor_range_m').value)
        self.min_height = float(self.get_parameter('min_sensor_height_m').value)
        self.max_height = float(self.get_parameter('max_sensor_height_m').value)
        self.publish_markers = bool(self.get_parameter('publish_markers').value)
        update_rate = float(self.get_parameter('terrain_update_rate').value)
        self.update_period = 1.0 / max(update_rate, 0.1)
        self.last_update_time = None

        local_width = float(self.get_parameter('local_width_m').value)
        local_height = float(self.get_parameter('local_height_m').value)
        self.local_origin = (-local_width / 2.0, -local_height / 2.0)
        self.local_size = (local_width, local_height)
        self.global_size = (
            float(self.get_parameter('global_width_m').value),
            float(self.get_parameter('global_height_m').value),
        )
        self.global_grid: ElevationGrid | None = None

        self.local_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('local_grid_topic').value, 1)
        self.global_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('global_grid_topic').value, 1)
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('costmap_topic').value, 1)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter('markers_topic').value, 1)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PointCloud2, self.get_parameter('pointcloud_topic').value,
            self.pointcloud_callback, 1)
        self.get_logger().info(
            f'Terrain mapper listening to {self.get_parameter("pointcloud_topic").value}; '
            f'local grid is {local_width:.1f} x {local_height:.1f} m at {self.resolution:.2f} m.'
        )

    def _declare_parameters(self) -> None:
        defaults = {
            'pointcloud_topic': '/points', 'local_grid_topic': '/terrain/local_grid',
            'global_grid_topic': '/terrain/global_grid', 'costmap_topic': '/terrain/costmap',
            'markers_topic': '/terrain/markers', 'local_frame_id': 'base_link',
            'map_frame_id': 'map', 'grid_resolution': 0.30, 'local_width_m': 30.0,
            'local_height_m': 30.0, 'global_width_m': 100.0, 'global_height_m': 100.0,
            'max_sensor_range_m': 15.0, 'min_sensor_height_m': -1.5,
            'max_sensor_height_m': 2.0, 'slope_limit_deg': 18.0,
            'step_height_limit_m': 0.16, 'roughness_limit_m': 0.08,
            'obstacle_height_m': 0.35, 'min_points_per_cell': 3,
            'max_samples_per_cell': 40, 'terrain_update_rate': 2.0,
            'publish_markers': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def pointcloud_callback(self, message: PointCloud2) -> None:
        """Classify the latest cloud locally and add map-aligned observations."""
        now = self.get_clock().now()
        if (self.last_update_time is not None and
                (now - self.last_update_time).nanoseconds < self.update_period * 1e9):
            return
        self.last_update_time = now
        points = self._read_points(message)
        if points.size == 0:
            return
        stamp = Time.from_msg(message.header.stamp)
        local_points = self._in_frame(
            points, self.local_frame, message.header.frame_id, stamp)
        if local_points is not None:
            local_grid = ElevationGrid(
                self.resolution, *self.local_size, *self.local_origin, self.max_samples)
            local_grid.add_points(local_points)
            local_costs = self._classify(local_grid)
            self.local_pub.publish(self._grid_message(
                local_grid, local_costs, self.local_frame, message.header.stamp))
            if self.publish_markers:
                self.marker_pub.publish(self._markers(
                    local_grid, local_costs, message.header.stamp,
                    self.local_frame, 0))

        map_points = self._in_frame(
            points, self.map_frame, message.header.frame_id, stamp)
        if map_points is None:
            return
        if self.global_grid is None:
            center = np.mean(map_points[:, :2], axis=0)
            self.global_grid = ElevationGrid(
                self.resolution, *self.global_size,
                float(center[0] - self.global_size[0] / 2.0),
                float(center[1] - self.global_size[1] / 2.0), self.max_samples)
            self.get_logger().info(
                'Initialized map-frame terrain grid around first registered cloud.')
        self.global_grid.add_points(map_points)
        global_costs = self._classify(self.global_grid)
        global_message = self._grid_message(
            self.global_grid, global_costs, self.map_frame, message.header.stamp)
        self.global_pub.publish(global_message)
        # The accumulated map includes every registered live observation.
        self.costmap_pub.publish(global_message)
        if self.publish_markers:
            self.marker_pub.publish(
                self._markers(
                    self.global_grid, global_costs, message.header.stamp,
                    self.map_frame, 1))

    def _read_points(self, message: PointCloud2) -> np.ndarray:
        structured_points = point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True)
        if structured_points.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        # ROS 2 Humble returns a structured NumPy array, including when only
        # x/y/z are requested. Column-stack preserves that API's field layout.
        points = np.column_stack((
            structured_points['x'], structured_points['y'], structured_points['z'],
        )).astype(np.float64, copy=False)
        ranges = np.linalg.norm(points[:, :2], axis=1)
        mask = (
            (ranges <= self.max_range) & (ranges > 0.05) &
            (points[:, 2] >= self.min_height) & (points[:, 2] <= self.max_height)
        )
        return points[mask]

    def _in_frame(
        self, points: np.ndarray, target: str, source: str, stamp: Time
    ) -> np.ndarray | None:
        if source == target:
            return points
        try:
            transform = self.tf_buffer.lookup_transform(
                target, source, stamp, timeout=Duration(seconds=0.05))
        except TransformException as error:
            self.get_logger().warn(
                f'No transform {target} <- {source}; skipping this terrain update: {error}',
                throttle_duration_sec=2.0)
            return None
        return transform_points(points, transform)

    def _classify(self, grid: ElevationGrid) -> np.ndarray:
        return grid.classify(self.min_points, self.slope_limit, self.step_limit,
                             self.roughness_limit, self.obstacle_height)

    def _grid_message(
        self, grid: ElevationGrid, costs: np.ndarray, frame: str, stamp
    ) -> OccupancyGrid:
        message = OccupancyGrid()
        message.header.frame_id = frame
        message.header.stamp = stamp
        message.info = MapMetaData()
        message.info.resolution = grid.resolution
        message.info.width = grid.width
        message.info.height = grid.height
        message.info.origin.position.x = grid.origin_x
        message.info.origin.position.y = grid.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = costs.reshape(-1).astype(np.int8).tolist()
        return message

    def _markers(
        self, grid: ElevationGrid, costs: np.ndarray, stamp, frame: str, marker_id: int
    ) -> MarkerArray:
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = stamp
        marker.ns = 'terrain'
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = grid.resolution
        marker.scale.y = grid.resolution
        marker.scale.z = 0.05
        marker.lifetime = Duration(seconds=0.0).to_msg()
        colours = {
            FREE: (0.1, 0.8, 0.2), ROUGH: (1.0, 0.75, 0.0),
            CAUTION: (1.0, 0.35, 0.0), LETHAL: (0.9, 0.05, 0.05),
        }
        for row, col in np.argwhere(costs >= FREE):
            cost = int(costs[row, col])
            point = Point()
            point.x = grid.origin_x + (col + 0.5) * grid.resolution
            point.y = grid.origin_y + (row + 0.5) * grid.resolution
            point.z = 0.05
            marker.points.append(point)
            red, green, blue = colours[cost]
            color = marker.color.__class__()
            color.r, color.g, color.b, color.a = red, green, blue, 0.75
            marker.colors.append(color)
        return MarkerArray(markers=[marker])


def main(args=None) -> None:
    """Run the terrain mapper."""
    rclpy.init(args=args)
    node = TraversabilityAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
