"""lidar_clusters: what the 360-degree path is allowed to call an object."""
import dataclasses

import numpy as np
import pytest

from object_fusion import lidar_clusters
from object_fusion.lidar_clusters import ClusterParams

RNG = np.random.default_rng(0)


@pytest.fixture(autouse=True, params=["exact", "grid"])
def method(request):
    """Every shape rule below must hold for BOTH ways of linking components."""
    return request.param


@pytest.fixture(autouse=True)
def _use_method(method, monkeypatch):
    def cluster_nonground(xyz, params=ClusterParams()):
        return lidar_clusters.cluster_nonground(xyz, dataclasses.replace(params, method=method))
    monkeypatch.setitem(globals(), "cluster_nonground", cluster_nonground)


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


# ------------------------------------------- speed-ups must not change a single cluster

def _same_partition(a, b):
    """Two labelings describe the same partition iff label pairs map one-to-one."""
    na, nb = len(np.unique(a)), len(np.unique(b))
    return na == nb == len(np.unique(np.column_stack([a, b]), axis=0))


def test_banded_components_equal_the_brute_force_rule():
    """The gap-banded pair search keeps exactly the pairs with d <= max(gap_i, gap_j)."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components

    from object_fusion.lidar_clusters import _components
    # Vegetation-like: dense clumps near the car plus sparse far returns, so every band is used.
    pts = np.vstack([RNG.normal([8, 6], 1.5, (500, 2)), RNG.normal([25, -12], 3.0, (400, 2)),
                     RNG.uniform([-60, -60], [60, 60], (300, 2))])
    p = ClusterParams()
    gap = p.gap_base + p.gap_per_m * np.hypot(pts[:, 0], pts[:, 1])
    d = np.hypot(*(pts[:, None, :] - pts[None, :, :]).transpose(2, 0, 1))
    adj = (d <= np.maximum(gap[:, None], gap[None, :])) & ~np.eye(len(pts), dtype=bool)
    i, j = np.nonzero(adj)
    ref = connected_components(coo_matrix((np.ones(i.size), (i, j)), shape=adj.shape),
                               directed=False)[1]
    assert _same_partition(_components(pts, gap), ref)


def test_packed_voxel_key_equals_row_unique():
    from object_fusion.lidar_clusters import _voxelise
    xyz = np.vstack([RNG.uniform([-60, -60, -2.2], [60, 60, 0.9], (3000, 3)),
                     box(10, 2, 1.0, 1.0, n=500)])
    keys = np.floor(xyz / 0.2).astype(np.int64)
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    ref = np.zeros((counts.size, 3))
    np.add.at(ref, inv.reshape(-1), xyz)
    vox, n = _voxelise(xyz, 0.2)
    assert np.array_equal(n, counts)
    assert np.allclose(vox, ref / counts[:, None], atol=1e-9)


# ------------------------------------------------------------- lateral crop (default OFF)

def test_lateral_crop_is_off_by_default():
    assert ClusterParams().max_lateral == 0.0
    assert len(cluster_nonground(box(20, 18, 2, 2))) == 1      # 18 m to the side, still seen


def test_lateral_crop_keeps_near_drops_far():
    p = ClusterParams(max_lateral=15.0)
    cl = cluster_nonground(np.vstack([box(20, 12, 2, 2), box(20, -18, 2, 2)]), p)
    assert len(cl) == 1 and abs(cl[0].y - 12) < 0.5


def test_lateral_crop_is_measured_in_the_ego_frame():
    """lidar_tc is yawed -5.35 deg from ego; at x = 50 m that is ~4.7 m of lateral."""
    from object_fusion.frames import ego_tf_yaw_rad, lidar_to_ego
    rot = -ego_tf_yaw_rad(None)
    obj = box(50, 12.0, 2, 2)
    y_ego = lidar_to_ego([[50.0, 12.0]])[0, 1]
    assert y_ego > 15.0                      # 12 m in lidar_tc is beyond 15 m in ego
    assert cluster_nonground(obj, ClusterParams(max_lateral=15.0, lateral_rot_rad=rot)) == []
    assert len(cluster_nonground(obj, ClusterParams(max_lateral=15.0))) == 1   # unrotated: kept


# --------------------------------------------------- association (sustain, never birth)
def test_a_cluster_sustains_the_track_it_lands_on():
    from object_fusion.lidar_clusters import associate_clusters
    tracks = [(30.0, 2.0), (45.0, -6.0)]
    clusters = [(45.3, -6.2), (30.2, 1.9)]
    assert sorted(associate_clusters(tracks, clusters)) == [(0, 1), (1, 0)]


def test_a_cluster_far_from_every_track_is_dropped_not_born():
    """Clusters carry no class: a bush and a car are the same shape. Birth is the camera's job."""
    from object_fusion.lidar_clusters import associate_clusters
    assert associate_clusters([(30.0, 0.0)], [(30.0, 12.0)]) == []
    assert associate_clusters([], [(10.0, 0.0)]) == []


def test_the_gate_widens_with_range_but_stays_tight():
    from object_fusion.lidar_clusters import associate_clusters
    # gate = 1.0 + 0.02 r: 1.2 m at 10 m, 2.0 m at 50 m.
    assert associate_clusters([(10.0, 0.0)], [(11.1, 0.0)]) == [(0, 0)]    # 1.1 < 1.2
    assert associate_clusters([(10.0, 0.0)], [(11.5, 0.0)]) == []          # 1.5 > 1.2
    assert associate_clusters([(50.0, 0.0)], [(51.5, 0.0)]) == [(0, 0)]    # 1.5 < 2.0
    assert associate_clusters([(50.0, 0.0)], [(53.0, 0.0)]) == []          # 3.0 > 2.0


def test_one_cluster_cannot_feed_two_tracks():
    """Two tracks living off one cluster is how a duplicate becomes self-sustaining."""
    from object_fusion.lidar_clusters import associate_clusters
    pairs = associate_clusters([(20.0, 0.0), (20.6, 0.0)], [(20.3, 0.0)])
    assert len(pairs) == 1


def test_grid_stamping_equals_binary_dilation():
    """_components_grid stamps a disk at each cell; that must equal ndimage.binary_dilation."""
    from scipy import ndimage

    from object_fusion.lidar_clusters import _components_grid, _disk
    vox = np.vstack([RNG.normal([8, 6, -1], [1.5, 1.5, 0.5], (600, 3)),
                     RNG.uniform([-60, -60, -2], [60, 60, 0.5], (400, 3))])
    p = ClusterParams()
    gap = p.gap_base + p.gap_per_m * np.hypot(vox[:, 0], vox[:, 1])
    cell, bands = p.voxel, p.grid_bands
    ij = np.floor(vox[:, :2] / cell).astype(np.int64)
    ij -= ij.min(axis=0)
    r = np.hypot(vox[:, 0], vox[:, 1])
    linked = np.zeros(tuple(ij.max(axis=0) + 1), dtype=bool)
    edges = np.linspace(r.min(), r.max() + 1e-6, bands + 1)
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (r >= lo) & (r < hi) if hi < edges[-1] else (r >= lo)
        if m.any():
            img = np.zeros_like(linked)
            img[ij[m, 0], ij[m, 1]] = True
            rad = (float(gap[m].max()) / cell - 1.0) / 2.0
            linked |= ndimage.binary_dilation(img, structure=_disk(rad)) if rad >= 1.0 else img
    ref = ndimage.label(linked, structure=np.ones((3, 3), dtype=bool))[0][ij[:, 0], ij[:, 1]]
    got = _components_grid(vox, gap, cell, bands)
    assert len(np.unique(ref)) == len(np.unique(got)) == len(np.unique(np.column_stack([ref, got]),
                                                                         axis=0))
