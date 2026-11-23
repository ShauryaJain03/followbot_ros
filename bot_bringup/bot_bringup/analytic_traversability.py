"""Deterministic local terrain scoring for FollowBot elevation maps."""

from array import array as float_array
from copy import deepcopy
import math

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout


def decode_layer(layer):
    """Return a GridMap Float32MultiArray as a row-major NumPy array."""
    data = np.asarray(layer.data, dtype=np.float32)
    dims = layer.layout.dim
    if len(dims) != 2:
        raise ValueError('GridMap layer must have two dimensions.')
    if dims[0].label == 'column_index' and dims[1].label == 'row_index':
        cols, rows = dims[0].size, dims[1].size
        return data.reshape((rows, cols), order='F')
    if dims[0].label == 'row_index' and dims[1].label == 'column_index':
        rows, cols = dims[0].size, dims[1].size
        return data.reshape((rows, cols), order='C')
    raise ValueError('Unsupported GridMap layer layout.')


def encode_layer(values):
    """Encode a row-major NumPy array in GridMap's column-major layout."""
    rows, cols = values.shape
    message = Float32MultiArray()
    message.layout = MultiArrayLayout()
    message.layout.dim = [
        MultiArrayDimension(
            label='column_index', size=cols, stride=rows * cols),
        MultiArrayDimension(label='row_index', size=rows, stride=rows),
    ]
    message.data = float_array('f')
    message.data.frombytes(np.ascontiguousarray(values.T, dtype=np.float32).tobytes())
    return message


def _score(value, limit):
    return np.clip(1.0 - value / limit, 0.0, 1.0)


def combine_scores(geometric, learned):
    """Return the conservative score where both estimators observed a cell."""
    combined = np.full_like(geometric, np.nan)
    valid = np.isfinite(geometric) & np.isfinite(learned)
    combined[valid] = np.minimum(geometric[valid], learned[valid])
    return combined


def compute_layers(elevation, resolution, max_slope_degrees, max_step_height_m,
                   max_roughness_m, roughness_radius_m, min_roughness_points):
    """Compute slope, step, roughness, and a conservative [0, 1] score."""
    elevation = np.asarray(elevation, dtype=np.float32)
    rows, cols = elevation.shape
    valid = np.isfinite(elevation)

    slope = np.full_like(elevation, np.nan)
    center = valid[1:-1, 1:-1]
    neighbors = (valid[:-2, 1:-1] & valid[2:, 1:-1] &
                 valid[1:-1, :-2] & valid[1:-1, 2:])
    mask = center & neighbors
    dz_drow = (elevation[2:, 1:-1] - elevation[:-2, 1:-1]) / (2.0 * resolution)
    dz_dcol = (elevation[1:-1, 2:] - elevation[1:-1, :-2]) / (2.0 * resolution)
    local_slope = np.arctan(np.hypot(dz_drow, dz_dcol))
    slope_interior = slope[1:-1, 1:-1]
    slope_interior[mask] = local_slope[mask]

    radius_cells = max(1, int(math.ceil(roughness_radius_m / resolution)))
    window_size = 2 * radius_cells + 1
    padded = np.pad(elevation, radius_cells, constant_values=np.nan)
    windows = np.lib.stride_tricks.sliding_window_view(
        padded, (window_size, window_size)).reshape(rows, cols, -1)
    window_valid = np.isfinite(windows)
    count = window_valid.sum(axis=-1)
    coordinates = np.arange(-radius_cells, radius_cells + 1, dtype=np.float32)
    grid_y, grid_x = np.meshgrid(coordinates * resolution,
                                 coordinates * resolution, indexing='ij')
    design = np.stack((grid_x.ravel(), grid_y.ravel(), np.ones(window_size ** 2)), axis=1)
    weights = window_valid.astype(np.float32)
    normal = np.einsum('...n,ni,nj->...ij', weights, design, design)
    normal += np.eye(3, dtype=np.float32) * 1e-6
    rhs = np.einsum('...n,...n,ni->...i', weights,
                    np.nan_to_num(windows), design)
    coefficients = np.linalg.solve(normal.reshape(-1, 3, 3), rhs.reshape(-1, 3))
    coefficients = coefficients.reshape(rows, cols, 3)
    fitted = np.einsum('...i,ni->...n', coefficients, design)
    residual = np.where(window_valid, windows - fitted, 0.0)
    roughness = np.sqrt(np.sum(residual ** 2, axis=-1) / np.maximum(count, 1))
    roughness[(count < min_roughness_points) | ~valid] = np.nan

    padded = np.pad(elevation, 1, constant_values=np.nan)
    step_windows = np.lib.stride_tricks.sliding_window_view(
        padded, (3, 3)).reshape(rows, cols, -1)
    offsets = np.arange(-1, 2, dtype=np.float32) * resolution
    step_y, step_x = np.meshgrid(offsets, offsets, indexing='ij')
    step_design = np.stack((step_x.ravel(), step_y.ravel(), np.ones(9)), axis=1)
    step_weights = np.isfinite(step_windows).astype(np.float32)
    step_normal = np.einsum(
        '...n,ni,nj->...ij', step_weights, step_design, step_design)
    step_normal += np.eye(3, dtype=np.float32) * 1e-6
    step_rhs = np.einsum(
        '...n,...n,ni->...i', step_weights, np.nan_to_num(step_windows), step_design)
    step_coefficients = np.linalg.solve(
        step_normal.reshape(-1, 3, 3), step_rhs.reshape(-1, 3)).reshape(rows, cols, 3)
    step_plane = np.einsum('...i,ni->...n', step_coefficients, step_design)
    step_residual = np.where(
        np.isfinite(step_windows), step_windows - step_plane,
        np.nan)
    with np.errstate(all='ignore'):
        # A one-sided ledge biases a local least-squares plane by about half
        # its height, while a continuous incline has no residual range.
        step = 2.0 * (
            np.nanmax(step_residual, axis=-1) - np.nanmin(step_residual, axis=-1))
    step[(np.sum(np.isfinite(step_windows), axis=-1) < 5) | ~valid] = np.nan

    score = np.minimum.reduce((
        _score(slope, math.radians(max_slope_degrees)),
        _score(step, max_step_height_m),
        _score(roughness, max_roughness_m),
    ))
    score[~(np.isfinite(slope) & np.isfinite(step) & np.isfinite(roughness))] = np.nan
    return slope, step, roughness, score


class AnalyticTraversability(Node):
    """Publish an explainable traversability GridMap from fused elevation."""

    def __init__(self):
        super().__init__('analytic_traversability')
        self.declare_parameter('input_topic', '/elevation_mapping_node/terrain_map')
        self.declare_parameter('output_topic', '/followbot_traversability/terrain_map')
        self.declare_parameter('max_slope_degrees', 20.0)
        self.declare_parameter('max_step_height_m', 0.12)
        self.declare_parameter('max_roughness_m', 0.05)
        self.declare_parameter('roughness_radius_m', 0.4)
        self.declare_parameter('min_roughness_points', 12)
        self._publisher = self.create_publisher(
            GridMap, self.get_parameter('output_topic').value, 1)
        self._subscription = self.create_subscription(
            GridMap, self.get_parameter('input_topic').value, self._callback, 1)

    def _callback(self, message):
        try:
            # shortcut: only CuPy's non-circular GridMaps are supported; add
            # index rotation if the upstream publisher changes its layout.
            if message.outer_start_index or message.inner_start_index:
                raise ValueError('Circular GridMap input is not supported.')
            layer_index = list(message.layers).index('elevation')
            elevation = decode_layer(message.data[layer_index])
            learned_index = list(message.layers).index('traversability')
            learned = decode_layer(message.data[learned_index])
            slope, step, roughness, traversability = compute_layers(
                elevation,
                message.info.resolution,
                self.get_parameter('max_slope_degrees').value,
                self.get_parameter('max_step_height_m').value,
                self.get_parameter('max_roughness_m').value,
                self.get_parameter('roughness_radius_m').value,
                self.get_parameter('min_roughness_points').value,
            )
            combined = combine_scores(traversability, learned)
            output = GridMap()
            output.header = deepcopy(message.header)
            output.info = deepcopy(message.info)
            output.layers = [
                'elevation', 'slope', 'step', 'roughness', 'traversability',
                'learned_traversability', 'combined_traversability',
            ]
            output.basic_layers = ['elevation']
            output.data = [encode_layer(layer) for layer in (
                elevation, slope, step, roughness, traversability, learned, combined)]
            self._publisher.publish(output)
        except (ValueError, np.linalg.LinAlgError) as error:
            self.get_logger().error(f'Could not compute traversability: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = AnalyticTraversability()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
