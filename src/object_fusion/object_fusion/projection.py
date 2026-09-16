"""LiDAR -> camera pixels, reproducing transform.py's projection exactly, in numpy.

ROS-free. This exists so object_fusion can publish a ground-removed projection without
touching transform.py or re-deriving its calibration: PROJ and T1 are IMPORTED from
perception_common.configs, read-only, so there is one calibration in the repo and not two.

The maths is transform.py's, line for line:

    m1  = inv(T1) @ pc_homogeneous     # lidar frame -> camera frame
    uv1 = PROJ @ m1                    # camera frame -> pixels
    u,v = uv1[:2] / uv1[2]             # perspective divide

with the same mask against the active image bounds. torch is not used -- this runs in a
CPU-only container on ~57k points per sweep, where numpy is the simpler choice.

The lateral/range CROP is deliberately NOT reproduced. transform.py crops to |y| <= 20 m,
which past ~67 m is narrower than the image itself. Widening it was measured and changed the
far-field range error by ~0.35 m out of 32 m, so the crop is not load-bearing for accuracy --
but there is no reason to re-impose a bound that throws away in-image points.
"""

from __future__ import annotations

import math

import numpy as np

from perception_common.configs import PROJ, T1
from perception_common.utils import inverse_rigid_transform

__all__ = ["image_size_from_proj", "project_to_pixels"]


#: The camera's actual image size, from /camera_fl/camera_info on the reference bag.
#:
#: transform.py uses camera_info's width/height once it has matched a camera_info message, and
#: falls back to 2*cx x 2*cy only when camera_info is missing. That fallback is ONE PIXEL SHORT
#: in each axis (2063x1543), because cx = 1031.312 and cy = 771.391 are not exact half-sizes.
#:
#: This module originally used the fallback permanently, which silently dropped the last pixel
#: column. Found by the ground A/B self-check: 13 of 4519 detections failed to reproduce
#: fusion_node's published positions (by up to 0.255 m), and every one of them sat on a stamp
#: with a box reaching past u = 2063. Callers that have camera_info must pass its size.
CAMERA_INFO_WH = (2064, 1544)


def image_size_from_proj(proj=None):
    """Image size implied by the principal point -- transform.py's FALLBACK, 1 px short.

    Prefer the real camera_info size; see CAMERA_INFO_WH.
    """
    p = PROJ if proj is None else proj
    return int(round(float(p[0, 2]) * 2.0)), int(round(float(p[1, 2]) * 2.0))


def project_to_pixels(xyz, image_wh=None, proj=None, extrinsic=None, return_index=False):
    """Project LiDAR-frame points to pixels. Returns ``(xyz_kept, u, v)``.

    With ``return_index`` also returns the indices into ``xyz`` of the kept points, so per-point
    attributes (a ground flag, an intensity) can be carried through without re-matching.

    ``xyz`` is (N,3) in the LiDAR frame. Only points that land inside the image AND in front
    of the camera survive; the returned arrays are index-aligned, so a caller can never pair
    a pixel with the wrong 3D point.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or xyz.shape[0] == 0:
        empty = (np.empty((0, 3), np.float32), np.empty(0, np.float32),
                 np.empty(0, np.float32))
        return empty + (np.empty(0, np.int64),) if return_index else empty

    P = np.asarray(PROJ if proj is None else proj, dtype=np.float64)
    E = np.asarray(T1 if extrinsic is None else extrinsic, dtype=np.float64)
    w, h = image_size_from_proj(P) if image_wh is None else image_wh

    homo = np.hstack([xyz, np.ones((xyz.shape[0], 1))])
    cam = homo @ inverse_rigid_transform(E).T        # lidar -> camera
    uv1 = cam @ P.T                                  # camera -> pixels (still homogeneous)

    z = uv1[:, 2]
    # Behind the camera divides by a negative depth and folds onto the image; drop it.
    valid = z > 1e-6
    u = np.full(z.shape, np.nan)
    v = np.full(z.shape, np.nan)
    u[valid] = uv1[valid, 0] / z[valid]
    v[valid] = uv1[valid, 1] / z[valid]

    keep = valid & (u > 0) & (u < w) & (v > 0) & (v < h)
    out = (np.ascontiguousarray(xyz[keep], dtype=np.float32),
           np.ascontiguousarray(u[keep], dtype=np.float32),
           np.ascontiguousarray(v[keep], dtype=np.float32))
    return out + (np.flatnonzero(keep),) if return_index else out


def pixel_ray(u, v, proj=None, extrinsic=None):
    """Ray through pixel (u, v) in the LiDAR frame -> ``(origin, unit direction)``.

    The exact inverse of project_to_pixels: ``extrinsic`` is the camera pose in the LiDAR frame
    (T1), so the ray starts at the camera centre and is rotated out of the camera axes.
    """
    P = np.asarray(PROJ if proj is None else proj, dtype=np.float64)
    E = np.asarray(T1 if extrinsic is None else extrinsic, dtype=np.float64)
    d_cam = np.linalg.solve(P[:3, :3], np.array([float(u), float(v), 1.0]))
    d = E[:3, :3] @ d_cam
    return E[:3, 3].copy(), d / np.linalg.norm(d)


def camera_ground_position(u, v, ground_xyz, *, z_guess=-2.37, radius=2.0, min_points=5,
                           iters=4, max_range=80.0, proj=None, extrinsic=None):
    """Where the ray through pixel (u, v) meets the LOCAL road -> ``(xyz, support)`` or None.

    For a camera detection with no usable LiDAR return on the object (a cone missed this sweep),
    range it from where its box's bottom edge touches the road. Two things make the naive version
    wrong on this vehicle, and both are handled here:

    * the camera is pitched ~2.7 deg down and mounted 1.2 m ahead of the LiDAR (T1); treating
      image row cy as the horizon put a 30 m cone at ~200 m;
    * a grazing ray is hypersensitive to road height -- at 30 m the ray is ~3 deg below
      horizontal, so a 0.8 deg road tilt moves the answer by ~25%. So no flat-road assumption:
      the height comes from LiDAR GROUND returns within ``radius`` of the intersection, iterated
      until it settles. Without ``min_points`` of support there is no answer (None), rather than
      one from an assumed plane.

    ``support`` is the number of ground returns behind the final height.
    """
    o, d = pixel_ray(u, v, proj, extrinsic)
    if d[2] >= -1e-6:
        return None                                   # at or above the horizon
    g = np.asarray(ground_xyz, dtype=np.float64).reshape(-1, 3)
    z = float(z_guess)
    p, support = None, 0
    for _ in range(max(1, int(iters))):
        s = (z - o[2]) / d[2]
        if s <= 0.0:
            return None
        p_new = o + s * d
        if math.hypot(p_new[0], p_new[1]) > max_range:
            return None
        near = np.hypot(g[:, 0] - p_new[0], g[:, 1] - p_new[1]) < radius if g.size else np.zeros(0, bool)
        support = int(np.count_nonzero(near))
        if support < min_points:
            return None
        z_new = float(np.median(g[near, 2]))
        if p is not None and abs(z_new - z) < 0.01:
            p = p_new
            break
        p, z = p_new, z_new
    s = (z - o[2]) / d[2]
    return (o + s * d, support) if s > 0.0 else None
