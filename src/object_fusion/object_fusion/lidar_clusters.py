"""360-degree LiDAR object candidates from the non-ground cloud (to-do item 7, stage 1).

ROS-free. The camera sees ~33 degrees ahead-left; the VLP-32C sees all around. Clusters here are
meant to SUSTAIN tracks the camera started -- a cone the car is passing, a car alongside -- and
never to birth them: without a semantic check a cluster is as likely a bush, a pole or a kerb as
an object, and the radar work (item 8) already showed what unrefereed birth costs.

The input is Patchwork++'s NON-GROUND points, so this adds no ground model of its own.

Pipeline, per sweep:
1. crop to where a cluster can mean something: 2.5-120 m, a height band above the road, outside
   the ego vehicle's own footprint;
2. voxelise (0.2 m) -- clustering raw points costs millions of neighbour pairs in vegetation;
3. connected components with a RANGE-ADAPTIVE gap: VLP-32C rings spread apart with distance, so
   a fixed gap splits far objects and merges near ones;
4. describe each component and drop the shapes an object cannot have (a 30 m wall, a flat patch).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

__all__ = ["ClusterParams", "Cluster", "cluster_nonground", "associate_clusters",
           "CLUSTER_GATE_BASE_M", "CLUSTER_GATE_PER_M"]

#: Gate for calling a cluster the same object as a track: gate = base + per_m * range. Measured
#: as the presence test in scripts/lidar_cluster_ab.py, where a mirrored-position control found a
#: cluster only 0-1% of the time behind the car, so this width is not loose enough to latch onto
#: whatever is nearby.
CLUSTER_GATE_BASE_M = 1.0
CLUSTER_GATE_PER_M = 0.02


@dataclass(frozen=True)
class ClusterParams:
    #: Nearer than this is mostly the vehicle's own body and mirrors; farther than max, a
    #: VLP-32C object is a handful of returns and a cluster says little.
    min_range: float = 2.5
    #: 120 m since 2026-10-20 (was 60 m; the user's call). It is the ceiling, not a tuning: the
    #: input is ground_projection's non-ground cloud, and Patchwork++ labels ground only out to
    #: ground_max_range (120 m) -- beyond it road returns would cluster as objects. Live on the
    #: vehicle, grid method: 33 -> 40-42 ms/sweep (p90 42 -> 58-60), age at publish 92 -> 100 ms,
    #: still 10 Hz. The grid bands (grid_bands) span twice the range, so a band's largest gap
    #: exceeds its nearest point's own gap by ~0.24 m instead of ~0.12 m, about one voxel.
    max_range: float = 120.0
    #: Height band in lidar_tc (sensor 2.37 m above the road): 0.15 m to 3.3 m above ground.
    z_min: float = -2.22
    z_max: float = 0.93
    #: The ego vehicle's own footprint in lidar_tc, to exclude self-returns.
    ego_box: tuple = (-3.5, 1.5, -1.2, 1.2)        # x_min, x_max, y_min, y_max
    voxel: float = 0.2
    #: Neighbour gap: gap_base + gap_per_m * range. Ring spacing grows ~linearly with range.
    gap_base: float = 0.45
    gap_per_m: float = 0.012
    min_voxels: int = 3
    #: Shapes an object cannot have. A kerb or wall is long and low / long and flat.
    max_length: float = 12.0
    min_height: float = 0.25
    #: Lateral half-width in the EGO frame, 0 = off (the default). Vegetation is what makes this
    #: path slow: at the treeline on 2026-10-08 half the non-ground points sat > 19.7 m to the
    #: side and a sweep cost ~110 ms (15 m: 18 ms). ``lateral_rot_rad`` rotates lidar_tc into ego
    #: (lidar_tc is yawed 5.35 deg -- 5.6 m of lateral at 60 m), the rotation frames.lidar_to_ego
    #: applies; this module stays ROS-free, so the node passes it in.
    max_lateral: float = 0.0
    lateral_rot_rad: float = 0.0
    #: How components are linked. "exact": KD-tree pairs, d <= max(gap_i, gap_j) -- the method
    #: every coverage number in HANDOFF item 7 was measured with. "grid": a 2-D occupancy grid,
    #: dilated per range band and labelled (_components_grid) -- 6.8 ms for the full 60 m on the
    #: 2026-10-08 treeline sweeps where "exact" takes 47.5 ms (idle CPU; live load ~3x both), so
    #: it needs no lateral crop. Not identical (~85-90% of object-sized clusters within 0.5-1 m
    #: of an exact one), but its coverage of real objects on the selfcal replay is the same or
    #: better in every sector, at equal or lower chance (scripts/lidar_cluster_ab.py --method).
    #: This LIBRARY default stays "exact" (the harnesses and tests reference it); the NODE has
    #: launched with "grid" and no crop since 2026-10-20.
    method: str = "exact"
    grid_bands: int = 6


@dataclass
class Cluster:
    x: float
    y: float
    length: float
    width: float
    height: float
    yaw: float
    n_points: int
    range: float


def _voxelise(xyz, size):
    """Mean point and point count per occupied voxel, ordered by voxel key.

    One packed int64 key and a sort, rather than ``np.unique(axis=0)`` on the (N, 3) keys, which
    sorts rows lexicographically and was ~14 ms of a sweep in vegetation. Same voxels, same order.
    """
    keys = np.floor(xyz / size).astype(np.int64)
    keys -= keys.min(axis=0)
    span = keys.max(axis=0) + 1
    packed = (keys[:, 0] * span[1] + keys[:, 1]) * span[2] + keys[:, 2]
    order = np.argsort(packed, kind="stable")
    sorted_keys = packed[order]
    starts = np.flatnonzero(np.r_[True, sorted_keys[1:] != sorted_keys[:-1]])
    counts = np.diff(np.r_[starts, sorted_keys.size])
    sums = np.add.reduceat(xyz[order], starts, axis=0)
    return sums / counts[:, None], counts


#: Gap bands for the pair search in :func:`_components`. Any number gives the same pairs; six
#: measured fastest on the 2026-10-08 treeline sweeps.
_GAP_BANDS = 6


def _components(pts2d, gap):
    """Connected components over KD-tree pairs; a pair joins when it is closer than the LARGER of
    its two points' gaps, so a near point never vetoes a far neighbour's looser spacing.

    Searched per gap band: each band queries its own largest gap against every point and keeps a
    pair when it is within the band point's own gap. A pair passes for one of its ends exactly
    when d <= max(gap_i, gap_j), so the edge set is the old single query at gap.max() -- without
    searching the dense near field at the far field's radius.
    """
    n = pts2d.shape[0]
    tree = cKDTree(pts2d)
    rows, cols = [], []
    for band in np.array_split(np.argsort(gap, kind="stable"), _GAP_BANDS):
        if band.size == 0:
            continue
        sp = cKDTree(pts2d[band]).sparse_distance_matrix(tree, float(gap[band].max()),
                                                          output_type="ndarray")
        i, j = band[sp["i"]], sp["j"]
        ok = (sp["v"] <= gap[i]) & (i != j)
        rows.append(i[ok])
        cols.append(j[ok])
    i = np.concatenate(rows) if rows else np.empty(0, int)
    j = np.concatenate(cols) if cols else np.empty(0, int)
    graph = coo_matrix((np.ones(i.size), (i, j)), shape=(n, n))
    return connected_components(graph, directed=False)[1]


def _disk(radius):
    k = int(np.floor(radius))
    y, x = np.mgrid[-k:k + 1, -k:k + 1]
    return (x * x + y * y) <= radius * radius + 1e-9


def _components_grid(vox, gap, cell, bands):
    """Components on a 2-D occupancy grid instead of point pairs.

    Each voxel marks its ``cell``-sized cell. Cells of one range band are dilated by a disk of
    radius ``(gap_band / cell - 1) / 2`` cells, where gap_band is the band's largest gap, and the
    union is labelled with 8-connectivity: two disks touch when their centres are about
    2 * radius + 1 cells apart, i.e. one gap. Cost is the grid, not the number of neighbour pairs,
    which is what made the exact search slow in vegetation. Approximations against "exact": cell
    quantisation (0.2 m), a band's gap rather than each point's, and two bands meeting at the
    mean of their gaps rather than the larger.
    """
    r = np.hypot(vox[:, 0], vox[:, 1])
    edges = np.linspace(r.min(), r.max() + 1e-6, bands + 1)
    band = np.clip(np.searchsorted(edges, r, side="right") - 1, 0, bands - 1)
    radius = np.array([(float(gap[band == b].max()) / cell - 1.0) / 2.0 if (band == b).any()
                       else 0.0 for b in range(bands)])
    pad = int(np.floor(max(radius.max(), 0.0)))
    ij = np.floor(vox[:, :2] / cell).astype(np.int64)
    ij -= ij.min(axis=0) - pad
    linked = np.zeros(tuple(ij.max(axis=0) + 1 + pad), dtype=bool)
    # Dilation by a symmetric disk is the union of the disk stamped at every occupied cell, so
    # stamp it: a few writes per cell, where ndimage.binary_dilation swept the whole ~600 x 500
    # grid once per band (15 of the 20 ms). Identical result.
    for b in range(bands):
        cells = ij[band == b]
        if cells.size == 0:
            continue
        for di, dj in np.argwhere(_disk(max(radius[b], 0.0))) - int(np.floor(max(radius[b], 0.0))):
            linked[cells[:, 0] + di, cells[:, 1] + dj] = True
    labels, _ = ndimage.label(linked, structure=np.ones((3, 3), dtype=bool))
    return labels[ij[:, 0], ij[:, 1]]


def cluster_nonground(xyz, params: ClusterParams = ClusterParams()):
    """Non-ground points (N, 3) in lidar_tc -> list of :class:`Cluster`."""
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    if xyz.shape[0] == 0:
        return []
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    x0, x1, y0, y1 = params.ego_box
    ego = (xyz[:, 0] > x0) & (xyz[:, 0] < x1) & (xyz[:, 1] > y0) & (xyz[:, 1] < y1)
    keep = ((r >= params.min_range) & (r <= params.max_range) & ~ego
            & (xyz[:, 2] >= params.z_min) & (xyz[:, 2] <= params.z_max))
    if params.max_lateral > 0.0:
        a = params.lateral_rot_rad
        keep &= np.abs(np.sin(a) * xyz[:, 0] + np.cos(a) * xyz[:, 1]) <= params.max_lateral
    if not keep.any():
        return []
    vox, counts = _voxelise(xyz[keep], params.voxel)
    vr = np.hypot(vox[:, 0], vox[:, 1])
    gap = params.gap_base + params.gap_per_m * vr
    if params.method == "grid":
        labels = _components_grid(vox, gap, params.voxel, params.grid_bands)
    else:
        labels = _components(vox[:, :2], gap)

    # One sort, then a slice per component -- not a full-length mask per label, which in
    # vegetation means hundreds of components times thousands of voxels.
    order = np.argsort(labels, kind="stable")
    sorted_labels = labels[order]
    starts = np.flatnonzero(np.r_[True, sorted_labels[1:] != sorted_labels[:-1]])
    ends = np.r_[starts[1:], sorted_labels.size]
    out = []
    for s, e in zip(starts, ends):
        idx = order[s:e]
        if idx.size < params.min_voxels:
            continue
        p = vox[idx]
        height = float(p[:, 2].max() - p[:, 2].min()) + params.voxel
        if height < params.min_height:
            continue
        c = p[:, :2].mean(axis=0)
        if idx.size >= 3:
            ev, evec = np.linalg.eigh(np.cov((p[:, :2] - c).T))
            major = evec[:, 1]
        else:
            major = np.array([1.0, 0.0])
        along = (p[:, :2] - c) @ major
        across = (p[:, :2] - c) @ np.array([-major[1], major[0]])
        length = float(np.ptp(along)) + params.voxel
        width = float(np.ptp(across)) + params.voxel
        if length > params.max_length:
            continue
        out.append(Cluster(x=float(c[0]), y=float(c[1]), length=length, width=width,
                           height=height, yaw=float(np.arctan2(major[1], major[0])),
                           n_points=int(counts[idx].sum()), range=float(np.hypot(*c))))
    return out


def associate_clusters(track_xy, cluster_xy, gate_base=CLUSTER_GATE_BASE_M,
                       gate_per_m=CLUSTER_GATE_PER_M):
    """Pair existing tracks with clusters. Returns ``[(track_index, cluster_index), ...]``.

    Clusters may only SUSTAIN a track, never birth one, so this never reports an unmatched
    cluster: without a semantic check a cluster is as likely a bush or a kerb as an object.
    One cluster serves at most one track (the assignment is exclusive), because two tracks
    feeding on one cluster is how a duplicate becomes self-sustaining.
    """
    from object_fusion.association import solve_assignment

    t = np.asarray(track_xy, dtype=np.float64).reshape(-1, 2)
    c = np.asarray(cluster_xy, dtype=np.float64).reshape(-1, 2)
    if t.shape[0] == 0 or c.shape[0] == 0:
        return []
    d = np.linalg.norm(t[:, None, :] - c[None, :, :], axis=2)
    gate = gate_base + gate_per_m * np.linalg.norm(t, axis=1)
    return solve_assignment(d, d <= gate[:, None])
