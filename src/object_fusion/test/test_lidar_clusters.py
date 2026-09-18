"""lidar_clusters: what the 360-degree path is allowed to call an object."""
import numpy as np

from object_fusion.lidar_clusters import ClusterParams, cluster_nonground

RNG = np.random.default_rng(0)


def box(cx, cy, length, width, z0=-2.1, z1=-0.6, n=300, yaw=0.0):
    """Points on the surface-ish of an upright box in lidar_tc."""
    a = RNG.uniform(-length / 2, length / 2, n)
    b = RNG.uniform(-width / 2, width / 2, n)
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack([cx + a * c - b * s, cy + a * s + b * c, RNG.uniform(z0, z1, n)])


def test_two_cones_three_metres_apart_stay_two_objects():
    pts = np.vstack([box(20, 3, 0.4, 0.4, n=40), box(23, 3, 0.4, 0.4, n=40)])
    assert len(cluster_nonground(pts)) == 2


def test_a_car_is_one_cluster_with_its_own_size():
    cl = cluster_nonground(box(30, -4, 4.5, 1.8))
    assert len(cl) == 1
    assert abs(cl[0].x - 30) < 0.3 and abs(cl[0].y + 4) < 0.3
    assert 4.0 < cl[0].length < 5.2 and 1.4 < cl[0].width < 2.4


def test_a_far_object_with_spread_rings_is_not_split():
    """At 50 m VLP-32C rings are ~0.8 m apart; a fixed 0.45 m gap would shred the car."""
    ring = []
    for k in range(4):                                  # four sparse rings across one car
        ring.append(box(50 + 0.8 * k, 0, 0.05, 1.8, n=15))
    assert len(cluster_nonground(np.vstack(ring))) == 1


def test_a_wall_is_not_an_object():
    assert cluster_nonground(box(15, 8, 30.0, 0.3, n=2000)) == []


def test_a_flat_patch_is_not_an_object():
    flat = box(12, 2, 2.0, 2.0, z0=-2.2, z1=-2.15, n=200)
    assert cluster_nonground(flat) == []


def test_the_vehicle_own_body_is_excluded():
    assert cluster_nonground(box(0.0, 0.0, 2.0, 1.5, n=200)) == []


def test_it_sees_all_the_way_round():
    """The point of the path: an object BEHIND the car is found as readily as one ahead."""
    cl = cluster_nonground(np.vstack([box(-20, 0, 4.5, 1.8), box(0, 12, 4.5, 1.8, yaw=np.pi / 2)]))
    assert len(cl) == 2


def test_empty_and_out_of_band_inputs():
    assert cluster_nonground(np.empty((0, 3))) == []
    too_high = box(20, 0, 2, 2, z0=2.0, z1=3.0)          # an overhead sign
    assert cluster_nonground(too_high, ClusterParams()) == []
