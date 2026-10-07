"""sphereformer_boundary on synthetic labelled sweeps."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sphereformer_boundary import (  # noqa: E402
    SweepAccumulator, TwistIntegrator, crop_ego, fixed_bin_boundary, legacy_boundary,
    lidar_to_vehicle, pack_cloud, vehicle_to_lidar,
)

YAW = -5.35
HALF = 3.5


def grid(x0, x1, y0, y1, step=0.25, z=-2.37):
    gx, gy = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1 + 1e-9, step))
    veh = np.column_stack((gx.ravel(), gy.ravel()))
    lid = vehicle_to_lidar(veh, YAW)
    return np.column_stack((lid, np.full(len(lid), z)))


def veh_y(pts):
    return lidar_to_vehicle(pts[:, :2], YAW)[:, 1]


def test_fixed_bins_follow_the_vehicle_axis_not_lidar_x():
    road = grid(3, 62, -HALF, HALF)
    left, right = fixed_bin_boundary(road, ego_yaw_deg=YAW)
    assert len(left) == 27 and len(right) == 27
    assert np.allclose(veh_y(left), HALF, atol=0.01)
    assert np.allclose(veh_y(right), -HALF, atol=0.01)
    # The legacy rule's bins span the road's own x-range, so they move with whatever road is
    # visible -- including road behind the vehicle, which becomes "boundary" too.
    behind = grid(-30, 0, -HALF, HALF)
    l_old, _ = legacy_boundary(np.vstack((behind, road)))
    assert (lidar_to_vehicle(l_old[:, :2], YAW)[:, 0] < 0).sum() >= 5
    l_new, _ = fixed_bin_boundary(np.vstack((behind, road)), ego_yaw_deg=YAW)
    assert (lidar_to_vehicle(l_new[:, :2], YAW)[:, 0] >= 5.0).all()


def test_connected_drops_a_patch_separated_by_terrain_and_keeps_an_open_lot():
    road = grid(3, 62, -HALF, HALF)
    patch = grid(20, 30, 10, 14)
    terrain = grid(3, 62, HALF + 0.25, 9.75)
    pts = np.vstack((road, patch))
    left, _ = fixed_bin_boundary(pts, other_xy=terrain[:, :2], ego_yaw_deg=YAW, connected=True)
    assert veh_y(left).max() < HALF + 0.1
    left_raw, _ = fixed_bin_boundary(pts, ego_yaw_deg=YAW)
    assert veh_y(left_raw).max() > 13.0
    # An open paved lot: no non-road evidence anywhere, so nothing splits and nothing drifts...
    lot = grid(3, 62, -25, 25)
    l_lot, r_lot = fixed_bin_boundary(lot, other_xy=np.empty((0, 2)), ego_yaw_deg=YAW,
                                      connected=True, require_evidence=False)
    assert np.allclose(veh_y(l_lot), 25, atol=0.01) and np.allclose(veh_y(r_lot), -25, atol=0.01)
    # ...and with edge evidence required, an extent with nothing seen beyond it is no edge.
    l_ev, r_ev = fixed_bin_boundary(lot, other_xy=np.empty((0, 2)), ego_yaw_deg=YAW)
    assert l_ev.size == 0 and r_ev.size == 0


def test_a_ring_arc_ending_inside_the_road_is_not_an_edge():
    road = grid(3, 62, -HALF, HALF)
    terrain = np.vstack((grid(3, 62, HALF + 0.25, 6), grid(3, 62, -6, -HALF - 0.25)))
    arc = (veh_y(road) > -0.5) | (lidar_to_vehicle(road[:, :2], YAW)[:, 0] < 38)
    pts = road[arc]                                    # beyond 38 m only the left part is seen
    _, r_no = fixed_bin_boundary(pts, ego_yaw_deg=YAW, require_evidence=False)
    assert veh_y(r_no).max() > -1.0                    # a bogus inner "edge" at -0.5
    _, r_ev = fixed_bin_boundary(pts, other_xy=terrain[:, :2], ego_yaw_deg=YAW)
    assert np.allclose(veh_y(r_ev), -HALF, atol=0.01)  # those bins are dropped instead


def test_connected_does_not_split_on_an_empty_ring_gap():
    rings = [grid(x, x + 0.5, -HALF, -1.0) for x in range(6, 60, 4)]
    rings += [grid(x, x + 0.5, 1.5, HALF) for x in range(6, 60, 4)]   # arcs, empty middle
    left, right = fixed_bin_boundary(np.vstack(rings), other_xy=np.empty((0, 2)),
                                     ego_yaw_deg=YAW, connected=True)
    assert np.allclose(veh_y(left), HALF, atol=0.01)
    assert np.allclose(veh_y(right), -HALF, atol=0.01)


def test_crop_ego_removes_the_roof():
    pts = np.array([[0.8, 0.1, -0.4, 0], [10.0, 1.0, -2.4, 0]], np.float32)
    assert crop_ego(pts, 2.5).shape[0] == 1


def test_twist_integrator_and_accumulator_carry_static_points():
    tw = TwistIntegrator()
    for k in range(101):
        tw.add(0.01 * k, 10.0, 0.0, 0.0)           # 10 m/s straight
    m = tw.motion(0.0, 0.5)
    assert m[1] == pytest.approx(5.0, abs=1e-6) and m[0] == 0.0
    assert tw.covers(0.5) and tw.covers(1.1) and not tw.covers(1.5)   # 0.2 s hold past 1.0
    acc = SweepAccumulator(3, ego_yaw_deg=YAW)
    p_world = np.array([[30.0, 3.5]])               # vehicle frame at t=0
    for t in (0.0, 0.25, 0.5):
        seen = p_world - (10.0 * t, 0.0)            # where it sits in the vehicle frame at t
        lid = vehicle_to_lidar(seen, YAW)
        acc.add(t, np.column_stack((lid, [[-2.4]])), np.empty((0, 2)))
    road, _ = acc.merged(tw)
    assert road.shape[0] == 3
    v = lidar_to_vehicle(road[:, :2], YAW)
    assert np.allclose(v, [[25.0, 3.5]] * 3, atol=1e-6)


def test_accumulator_without_odometry_keeps_only_the_newest_sweep():
    acc = SweepAccumulator(3)
    acc.add(0.0, np.zeros((5, 3)), np.empty((0, 2)))
    acc.add(0.1, np.ones((2, 3)), np.empty((0, 2)))
    road, _ = acc.merged(TwistIntegrator())
    assert road.shape[0] == 2


def test_pack_cloud_round_trips_with_a_uint8_label():
    xyz = np.random.default_rng(0).normal(size=(5, 3)).astype(np.float32)
    rgb = np.arange(5, dtype=np.uint32).view(np.float32)
    label = np.array([9, 1, 15, 9, 0], np.uint8)
    fields, step, data, n = pack_cloud(
        None, {"x": xyz[:, 0], "y": xyz[:, 1], "z": xyz[:, 2], "rgb": rgb, "label": label},
        {"x": np.float32, "y": np.float32, "z": np.float32, "rgb": np.float32, "label": np.uint8})
    assert n == 5 and step == 20 and len(data) == 100
    offs = {name: o for name, o, _ in fields}
    a = np.frombuffer(data, np.uint8).reshape(5, step)
    assert np.array_equal(a[:, offs["label"]], label)
    assert np.allclose(a[:, offs["y"]:offs["y"] + 4].copy().view(np.float32).ravel(), xyz[:, 1])
