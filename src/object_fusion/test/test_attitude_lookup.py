"""ground_segmentation.attitude_at: odometry attitude at a LiDAR sweep's capture stamp."""
import pytest

from object_fusion.ground_segmentation import attitude_at

S = [(0.00, 1.0, -1.0), (0.01, 2.0, -2.0), (0.02, 3.0, -3.0)]


def test_interpolates_between_neighbours():
    assert attitude_at(S, 0.005, 0.2) == pytest.approx((1.5, -1.5))


def test_nearest_sample_just_outside_the_buffer():
    assert attitude_at(S, 0.05, 0.2) == (3.0, -3.0)
    assert attitude_at(S, -0.05, 0.2) == (1.0, -1.0)


def test_no_levelling_without_a_close_sample():
    assert attitude_at(S, 0.5, 0.2) == (None, None)
    assert attitude_at([], 0.0, 0.2) == (None, None)
    gap = [(0.0, 1.0, 1.0), (1.0, 2.0, 2.0)]
    assert attitude_at(gap, 0.5, 0.2) == (None, None), "neither side within the gap"
