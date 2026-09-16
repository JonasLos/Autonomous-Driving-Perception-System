"""The ego frame, and the sign convention that is easy to get backwards."""
import math

import numpy as np
import pytest

from object_fusion import frames
from perception_common.lane_geometry import DEFAULT_EGO_YAW_DEG


def test_constant_is_imported_not_restated():
    # If these ever diverge, the object path and the lane path disagree about where the
    # vehicle is pointing, which is exactly the class of bug this package exists to remove.
    assert frames.EGO_YAW_IN_LIDAR_DEG == DEFAULT_EGO_YAW_DEG == -5.35


def test_tf_yaw_is_negative_and_points_rotate_positive():
    """The two directions differ in sign. Both are asserted so neither can silently flip."""
    assert frames.ego_tf_yaw_rad() == math.radians(-5.35)
    # A point on lidar_tc's +x axis is, in ego coordinates, rotated by +5.35 deg.
    p = frames.lidar_to_ego([[100.0, 0.0]])[0]
    assert p[1] > 0.0, "vehicle-left must be positive y after the correction"
    assert math.degrees(math.atan2(p[1], p[0])) == pytest.approx(5.35, abs=1e-9)


def test_nine_point_four_cm_per_metre():
    """The documented consequence of the shear, reproduced from the transform itself."""
    p = frames.lidar_to_ego([[1.0, 0.0]])[0]
    assert abs(p[1] - 0.0932) < 5e-4          # 9.3 cm per metre of range


def test_round_trip_is_identity():
    pts = np.array([[10.0, 3.0], [80.0, -12.0], [0.0, 0.0]])
    back = frames.ego_to_lidar(frames.lidar_to_ego(pts))
    assert np.allclose(back, pts, atol=1e-12)


def test_zero_correction_is_the_rollback():
    """correction_deg=0 must reproduce today's (defective) behaviour exactly."""
    pts = np.array([[50.0, 5.0]])
    assert np.allclose(frames.lidar_to_ego(pts, correction_deg=0.0), pts)
    assert frames.ego_tf_yaw_rad(0.0) == 0.0


def test_rotation_is_pure_yaw_no_translation():
    """ego shares lidar_tc's origin -- guessing an origin offset would be unmeasured."""
    assert np.allclose(frames.lidar_to_ego([[0.0, 0.0]])[0], [0.0, 0.0])
