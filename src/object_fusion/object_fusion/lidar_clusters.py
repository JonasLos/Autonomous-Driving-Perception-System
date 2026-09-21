"""360-degree LiDAR object candidates from the non-ground cloud (to-do item 7, stage 1).

ROS-free. The camera sees ~33 degrees ahead-left; the VLP-32C sees all around. Clusters here are
meant to SUSTAIN tracks the camera started -- a cone the car is passing, a car alongside -- and
never to birth them: without a semantic check a cluster is as likely a bush, a pole or a kerb as
an object, and the radar work (item 8) already showed what unrefereed birth costs.

The input is Patchwork++'s NON-GROUND points, so this adds no ground model of its own.

Pipeline, per sweep:
1. crop to where a cluster can mean something: 2.5-60 m, a height band above the road, outside
   the ego vehicle's own footprint;
2. voxelise (0.2 m) -- clustering raw points costs millions of neighbour pairs in vegetation;
3. connected components with a RANGE-ADAPTIVE gap: VLP-32C rings spread apart with distance, so
   a fixed gap splits far objects and merges near ones;
4. describe each component and drop the shapes an object cannot have (a 30 m wall, a flat patch).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
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
    max_range: float = 60.0
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
    keys = np.floor(xyz / size).astype(np.int64)
    _, first, inv, counts = np.unique(keys, axis=0, return_index=True, return_inverse=True,
                                      return_counts=True)
    sums = np.zeros((counts.size, 3))
    np.add.at(sums, inv.reshape(-1), xyz)
    return sums / counts[:, None], counts


def _components(pts2d, gap):
    """Connected components over KD-tree pairs; a pair joins when it is closer than the LARGER of
    its two points' gaps, so a near point never vetoes a far neighbour's looser spacing."""
    n = pts2d.shape[0]
    pairs = cKDTree(pts2d).query_pairs(float(gap.max()), output_type="ndarray")
    if pairs.size:
        d = np.linalg.norm(pts2d[pairs[:, 0]] - pts2d[pairs[:, 1]], axis=1)
        pairs = pairs[d <= np.maximum(gap[pairs[:, 0]], gap[pairs[:, 1]])]
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])) if len(pairs)
                       else (np.empty(0), (np.empty(0, int), np.empty(0, int))), shape=(n, n))
    return connected_components(graph, directed=False)[1]


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
    if not keep.any():
        return []
    vox, counts = _voxelise(xyz[keep], params.voxel)
    vr = np.hypot(vox[:, 0], vox[:, 1])
    labels = _components(vox[:, :2], params.gap_base + params.gap_per_m * vr)

    out = []
    for lab in np.unique(labels):
        m = labels == lab
        if m.sum() < params.min_voxels:
            continue
        p = vox[m]
        height = float(p[:, 2].max() - p[:, 2].min()) + params.voxel
        if height < params.min_height:
            continue
        c = p[:, :2].mean(axis=0)
        if m.sum() >= 3:
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
                           n_points=int(counts[m].sum()), range=float(np.hypot(*c))))
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
