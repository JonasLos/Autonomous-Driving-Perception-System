"""The projection must reproduce transform.py, not approximate it."""
import numpy as np
import pytest

from object_fusion.projection import image_size_from_proj, project_to_pixels
from perception_common.configs import PROJ


def test_image_size_comes_from_the_principal_point():
    """transform.py derives its fallback image size the same way; they must agree or the
    two would mask against different bounds and keep different points."""
    w, h = image_size_from_proj()
    assert (w, h) == (int(round(PROJ[0, 2] * 2)), int(round(PROJ[1, 2] * 2)))
    assert (w, h) == (2063, 1543)


def test_points_behind_the_camera_are_dropped_not_folded():
    """A negative depth divides to a positive-looking pixel and lands back inside the image.
    Dropping it is the difference between an object behind the car and a ghost ahead of it."""
    xyz, u, v = project_to_pixels(np.array([[-30.0, 0.0, -1.0], [30.0, 0.0, -1.0]]))
    assert xyz.shape[0] == 1
    assert xyz[0][0] > 0


def test_outputs_stay_index_aligned():
    """xyz, u and v are consumed together; a caller must never be able to pair a pixel with
    the wrong 3D point."""
    pts = np.array([[20.0, 0.0, -1.0], [-5.0, 0.0, -1.0], [40.0, 3.0, -1.0],
                    [60.0, 40.0, -1.0]])
    xyz, u, v = project_to_pixels(pts)
    assert xyz.shape[0] == u.size == v.size
    for p, uu, vv in zip(xyz, u, v):
        single_xyz, single_u, single_v = project_to_pixels(p.reshape(1, 3))
        assert single_u[0] == pytest.approx(uu, abs=1e-4)
        assert single_v[0] == pytest.approx(vv, abs=1e-4)


def test_empty_and_degenerate_input():
    for bad in (np.empty((0, 3)), np.empty((0, 3), dtype=np.float32)):
        xyz, u, v = project_to_pixels(bad)
        assert xyz.shape == (0, 3) and u.size == 0 and v.size == 0


def test_scaling_along_a_CAMERA_ray_preserves_the_pixel():
    """The defining property of a perspective divide.

    Note it must be a ray from the CAMERA centre, not from the LiDAR origin: tf_static puts
    camera_fl 1.21 m forward and 0.27 m left of lidar_tc, so a LiDAR ray carries real parallax
    -- doubling range along one moves the pixel by ~78 px at 20 m. An earlier version of this
    test assumed otherwise and was wrong about the geometry, not about the code.
    """
    from perception_common.configs import T1
    from perception_common.utils import inverse_rigid_transform

    p_lidar = np.array([20.0, 1.0, -0.5, 1.0])
    to_cam = inverse_rigid_transform(np.asarray(T1, dtype=np.float64))
    p_cam = to_cam @ p_lidar
    back = np.linalg.inv(to_cam)

    pixels = []
    for scale in (1.0, 1.7, 2.5):
        q_cam = np.array([p_cam[0] * scale, p_cam[1] * scale, p_cam[2] * scale, 1.0])
        q_lidar = back @ q_cam
        _, u, v = project_to_pixels(q_lidar[:3].reshape(1, 3))
        assert u.size == 1, "the scaled point should stay in frame"
        pixels.append((float(u[0]), float(v[0])))

    for u, v in pixels[1:]:
        assert u == pytest.approx(pixels[0][0], abs=1e-3)
        assert v == pytest.approx(pixels[0][1], abs=1e-3)


def test_the_principal_point_estimate_is_one_pixel_short_of_camera_info():
    """Regression for a real bug. transform.py uses camera_info's 2064x1544 once it has it and
    the 2*cx x 2*cy estimate only as a fallback. projection.py used the fallback permanently,
    silently dropping the last pixel column: 13 of 4519 detections then failed to reproduce
    fusion_node's published positions, every one on a box reaching past u = 2063."""
    from object_fusion.projection import CAMERA_INFO_WH
    fw, fh = image_size_from_proj()
    assert CAMERA_INFO_WH == (2064, 1544)
    assert (CAMERA_INFO_WH[0] - fw, CAMERA_INFO_WH[1] - fh) == (1, 1)


def test_a_point_in_the_last_pixel_column_survives_with_camera_info_bounds():
    from object_fusion.projection import CAMERA_INFO_WH
    from perception_common.configs import T1
    from perception_common.utils import inverse_rigid_transform

    # Build a camera-frame point that lands at u = 2063.5, then lift it back to the LiDAR frame.
    P = np.asarray(PROJ, dtype=np.float64)
    depth = 30.0
    u_target, v_target = 2063.5, 700.0
    x_cam = (u_target - P[0, 2]) * depth / P[0, 0]
    y_cam = (v_target - P[1, 2]) * depth / P[1, 1]
    to_cam = inverse_rigid_transform(np.asarray(T1, dtype=np.float64))
    p_lidar = (np.linalg.inv(to_cam) @ np.array([x_cam, y_cam, depth, 1.0]))[:3]

    assert project_to_pixels(p_lidar.reshape(1, 3))[0].shape[0] == 0, "fallback drops it"
    kept = project_to_pixels(p_lidar.reshape(1, 3), image_wh=CAMERA_INFO_WH)[0]
    assert kept.shape[0] == 1, "the real image bounds keep it"


def test_return_index_carries_per_point_attributes_exactly():
    """The ground flag rides along by index, so it can never be attached to the wrong point."""
    pts = np.array([[20.0, 0.0, -1.0], [-5.0, 0.0, -1.0], [40.0, 3.0, -1.0], [60.0, 40.0, -1.0]])
    kept, u, v, idx = project_to_pixels(pts, return_index=True)
    assert np.allclose(pts[idx].astype(np.float32), kept)
    assert 1 not in idx, "the point behind the sensor is not among the kept indices"


def test_pixel_ray_is_the_inverse_of_the_projection():
    from object_fusion.projection import pixel_ray, project_to_pixels
    pts = np.array([[20.0, -2.0, -2.46], [35.0, 3.0, -1.0], [60.0, -5.0, -2.2]])
    kept, u, v = project_to_pixels(pts, image_wh=(2064, 1544))
    for p, uu, vv in zip(kept, u, v):
        o, d = pixel_ray(uu, vv)
        s = (p - o) @ d
        assert np.linalg.norm(o + s * d - p) < 1e-3, "the ray passes through the point"


def test_camera_ground_position_recovers_a_cone_base_on_a_tilted_road():
    from object_fusion.projection import camera_ground_position, project_to_pixels
    # road rising 2 cm per metre to the left (y) -- a flat-road assumption would be wrong here
    rng = np.random.default_rng(0)
    gx, gy = rng.uniform(5, 60, 6000), rng.uniform(-15, 15, 6000)
    ground = np.column_stack([gx, gy, -2.46 + 0.02 * gy])
    base = np.array([[30.0, 4.0, -2.46 + 0.02 * 4.0]])
    _, u, v = project_to_pixels(base, image_wh=(2064, 1544))
    out = camera_ground_position(u[0], v[0], ground)
    assert out is not None
    xyz, support = out
    assert np.linalg.norm(xyz[:2] - base[0, :2]) < 0.3 and support >= 5


def test_camera_ground_position_refuses_without_ground_support_or_above_horizon():
    from object_fusion.projection import camera_ground_position
    assert camera_ground_position(1000.0, 1400.0, np.empty((0, 3))) is None
    assert camera_ground_position(1000.0, 100.0, np.array([[30.0, 0.0, -2.46]] * 10)) is None
