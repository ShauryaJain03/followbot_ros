import math

import numpy as np

from bot_bringup.analytic_traversability import (
    compute_layers,
    combine_scores,
    decode_layer,
    encode_layer,
)


def test_gridmap_layer_round_trip_preserves_values():
    source = np.arange(12, dtype=np.float32).reshape(3, 4)

    restored = decode_layer(encode_layer(source))

    np.testing.assert_array_equal(restored, source)


def test_combined_score_is_conservative_and_preserves_unknown():
    geometric = np.array([[0.8, 0.4], [np.nan, 0.9]], dtype=np.float32)
    learned = np.array([[0.7, 0.6], [0.5, np.nan]], dtype=np.float32)

    combined = combine_scores(geometric, learned)

    np.testing.assert_allclose(combined[0], [0.7, 0.4])
    assert np.isnan(combined[1, 0])
    assert np.isnan(combined[1, 1])


def test_flat_surface_is_fully_traversable():
    elevation = np.zeros((9, 9), dtype=np.float32)
    slope, step, roughness, score = compute_layers(
        elevation, 0.2, 20.0, 0.12, 0.05, 0.4, 12)

    assert math.isclose(slope[4, 4], 0.0)
    assert math.isclose(step[4, 4], 0.0)
    assert math.isclose(roughness[4, 4], 0.0, abs_tol=1e-6)
    assert math.isclose(score[4, 4], 1.0)


def test_step_at_limit_is_not_traversable():
    elevation = np.zeros((9, 9), dtype=np.float32)
    elevation[:, 5:] = 0.12
    _, step, _, score = compute_layers(
        elevation, 0.2, 45.0, 0.12, 1.0, 0.4, 12)

    assert math.isclose(step[4, 4], 0.12, abs_tol=1e-6)
    assert math.isclose(score[4, 4], 0.0, abs_tol=1e-6)


def test_continuous_incline_is_not_a_step():
    resolution = 0.2
    elevation = np.broadcast_to(
        np.arange(9, dtype=np.float32) * resolution * math.tan(math.radians(30.0)),
        (9, 9)).copy()
    _, step, _, score = compute_layers(
        elevation, resolution, 38.0, 0.20, 1.0, 0.4, 12)

    assert math.isclose(step[4, 4], 0.0, abs_tol=1e-5)
    assert score[4, 4] > 0.2
