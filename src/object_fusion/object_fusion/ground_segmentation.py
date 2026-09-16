"""Ground / non-ground segmentation of a full LiDAR sweep. ROS-free.

WHY THIS EXISTS, in measured terms. The camera+LiDAR path selects 3D points with a 2D image
box, and a box around a vehicle also contains the road in front of it. The road is NEARER, so
a nearest-cluster rule adopts it. Measured against radar range on the reference replay:

    band        median range error
    20-40 m     -1.34 m
    80-100 m    -7.59 m
    100-175 m   -32.10 m

Two competing explanations were tested and one was eliminated. Widening transform.py's crop
3x laterally moved the far field by ~0.35 m out of 32 -- point starvation is NOT the cause.
And the boxes do contain the vehicle: median z-spread inside a far-field box is 1.24 m against
a ~1.5 m car, with only 14.6% of 80-100 m boxes holding flat ground alone. The returns are
there; the selection rule discards them.

Closer in, the evidence is just as direct. fusion_node's ground rejection does not run below
``ground_rejection_min_range`` (25 m by default), and simply lowering that bound to 5 m halved
the range-error spread in the 15-25 m band (sd 2.24 -> 1.15 m) and cut >1 m position jumps
inside 15 m from 25.9% to 16.2%. Ground removal works; it was switched off where it was needed.

That 25 m bound exists to protect short objects -- a cone stands ~0.5 m and the margin is
0.4 m -- which is exactly the trade a proper ground model removes: segment the ground
geometrically rather than by a blanket height margin, and a cone stops being collateral.

BACKENDS. ``patchworkpp`` is the real one (Lee et al., IROS 2022): concentric zones, region-wise
plane fitting, adaptive elevation thresholds, reflected-noise removal. It wants the FULL sweep
-- its zone model degrades on a narrow forward wedge -- which is why the node feeding it
subscribes to the raw cloud rather than to the already-cropped projection.

``fallback`` is a deliberately simple polar-grid stand-in so this module imports, and the tests
run, without the dependency. It is NOT equivalent: one plane per polar cell, no adaptive
thresholding, no noise removal. Treat it as a floor, not as a substitute.
"""

from __future__ import annotations

import bisect

import numpy as np

__all__ = ["GroundSegmenter", "segment_ground_fallback", "patchworkpp_available",
           "levelling_rotation", "road_plane_slopes", "quaternion_roll_pitch_deg",
           "slopes_from_attitude", "ODOM_TILT_COEF", "LevelledGroundSegmenter", "attitude_at"]


def patchworkpp_available() -> bool:
    try:
        import pypatchworkpp  # noqa: F401
        return True
    except Exception:
        return False


def segment_ground_fallback(xyz, *, n_rings=12, n_sectors=36, max_range=120.0,
                            sensor_height=2.37, height_margin=0.25,
                            max_slope=0.25, min_cell_points=6):
    """Polar-grid ground segmentation. Returns a boolean mask, True where GROUND.

    One plane per (ring, sector) cell, seeded from that cell's lowest points. Per-cell rather
    than global because these roads fall ~1.4 cm per metre, so a single flat threshold across
    the sweep misclassifies the far field on any real camber or grade.

    ``sensor_height`` is measured on this vehicle: the LiDAR sits 2.37 m above the road
    (jeep_selfcal_loc measured 2.366 +- 0.005; a direct median of near-field ground returns gives
    2.394; this module shipped with 2.46, which was wrong). Points far above the expected road
    are never even considered as seeds, which stops a van roof from defining the ground plane for
    its cell.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    n = xyz.shape[0]
    ground = np.zeros(n, dtype=bool)
    if n == 0:
        return ground

    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    r = np.hypot(x, y)
    az = np.arctan2(y, x)

    in_range = (r > 0.5) & (r <= max_range)
    if not np.any(in_range):
        return ground

    # EQUAL-AREA rings (sqrt of a linear r^2 spacing). Note what that means, because an earlier
    # comment here claimed the opposite: equal-area rings are WIDEST near the sensor -- with the
    # defaults the first ring spans 0.5 to 34.6 m. A vehicle at 25-40 m can therefore dominate
    # its cell and have the ground plane fitted THROUGH it; on the reference replay this
    # fallback emptied 9.8% of 25-40 m detections that way. It is a stand-in, not a substitute.
    edges = np.sqrt(np.linspace(0.5 ** 2, max_range ** 2, n_rings + 1))
    ring = np.clip(np.digitize(r, edges) - 1, 0, n_rings - 1)
    sect = np.clip(((az + np.pi) / (2 * np.pi) * n_sectors).astype(int), 0, n_sectors - 1)
    cell = ring * n_sectors + sect

    expected = -float(sensor_height)
    for c in np.unique(cell[in_range]):
        idx = np.flatnonzero((cell == c) & in_range)
        if idx.size < min_cell_points:
            continue
        zc = z[idx]
        # Seeds: the lowest points, but never ones implausibly far above the road.
        lo = np.percentile(zc, 20.0)
        seed = idx[(zc <= lo + height_margin) & (zc < expected + 1.0)]
        if seed.size < 3:
            continue
        # Plane z = a*x + b*y + c through the seeds.
        A = np.column_stack([x[seed], y[seed], np.ones(seed.size)])
        try:
            coef, *_ = np.linalg.lstsq(A, z[seed], rcond=None)
        except np.linalg.LinAlgError:
            continue
        if np.hypot(coef[0], coef[1]) > max_slope:
            continue                       # implausible grade: the fit caught a wall
        pred = coef[0] * x[idx] + coef[1] * y[idx] + coef[2]
        ground[idx] = (z[idx] - pred) < height_margin
    return ground


def levelling_rotation(slope_x, slope_y):
    """Rotation taking the plane ``z = slope_x*x + slope_y*y + c`` to horizontal.

    Maps the plane normal (-slope_x, -slope_y, 1) onto +z by the smallest rotation (Rodrigues),
    so it introduces no yaw: azimuths, which Patchwork++'s zones are built on, barely move.
    """
    n = np.array([-float(slope_x), -float(slope_y), 1.0])
    n /= np.linalg.norm(n)
    v = np.cross(n, [0.0, 0.0, 1.0])
    s = float(np.linalg.norm(v))
    if s < 1e-12:
        return np.eye(3)
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - n[2]) / s ** 2)


def road_plane_slopes(xyz, *, r_min=3.0, r_max=20.0, z_max=-1.5, iters=60, inlier_m=0.10,
                      max_tilt_deg=6.0, min_inliers=200, seed=0):
    """RANSAC road plane in the near field -> ``(slope_x, slope_y)``, or None if not trustworthy.

    Candidates are low returns (``z < z_max``) 3-20 m out. Only near-horizontal hypotheses are
    scored (tilt under ``max_tilt_deg``), so a wall or a truck side cannot win, and the final
    plane is a least-squares refit on the inliers. None when fewer than ``min_inliers`` support
    it -- the caller should then fall back rather than level by noise. Deterministic (``seed``).
    """
    p = np.asarray(xyz, dtype=np.float64)
    r = np.hypot(p[:, 0], p[:, 1])
    cand = p[(r > r_min) & (r < r_max) & (p[:, 2] < z_max)]
    n = cand.shape[0]
    if n < min_inliers:
        return None
    rng = np.random.default_rng(seed)
    tri = cand[rng.integers(0, n, (iters, 3))]
    nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    length = np.linalg.norm(nrm, axis=1)
    ok = length > 1e-9
    nrm, anchor = nrm[ok] / length[ok, None], tri[ok, 0]
    upright = np.abs(nrm[:, 2]) > np.cos(np.radians(max_tilt_deg))
    nrm, anchor = nrm[upright], anchor[upright]
    if nrm.shape[0] == 0:
        return None
    sub = cand[rng.integers(0, n, min(n, 3000))]
    dist = np.abs(np.einsum("hsk,hk->hs", sub[None] - anchor[:, None], nrm))
    best = int(np.argmax((dist < inlier_m).sum(axis=1)))
    inl = np.abs((cand - anchor[best]) @ nrm[best]) < inlier_m
    if int(inl.sum()) < min_inliers:
        return None
    A = np.column_stack([cand[inl, 0], cand[inl, 1], np.ones(int(inl.sum()))])
    coef, *_ = np.linalg.lstsq(A, cand[inl, 2], rcond=None)
    if np.degrees(np.arctan(np.hypot(coef[0], coef[1]))) > max_tilt_deg:
        return None
    return float(coef[0]), float(coef[1])


def quaternion_roll_pitch_deg(qx, qy, qz, qw):
    """(roll, pitch) in degrees from a quaternion, ZYX convention. Signs are whatever the
    publisher uses -- the NovAtel odom's pitch is inverted relative to INSPVA -- so callers map
    attitude to slopes through a FITTED linear model rather than assuming a convention."""
    roll = np.degrees(np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)))
    pitch = np.degrees(np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)))
    return float(roll), float(pitch)


def attitude_at(samples, t, max_gap):
    """Interpolate (roll, pitch) at ``t`` from time-ordered ``(stamp, roll, pitch)`` samples.

    Linear between the two neighbours when they are within ``max_gap`` of each other; otherwise
    the nearest sample if it is within ``max_gap`` of ``t``; otherwise (None, None).
    """
    if not samples:
        return None, None
    stamps = [a[0] for a in samples]
    i = bisect.bisect_left(stamps, t)
    if 0 < i < len(samples):
        (t0, r0, p0), (t1, r1, p1) = samples[i - 1], samples[i]
        if 0.0 < t1 - t0 <= max_gap:
            w = (t - t0) / (t1 - t0)
            return r0 + w * (r1 - r0), p0 + w * (p1 - p0)
    nearest = min((k for k in (i - 1, i) if 0 <= k < len(samples)),
                  key=lambda k: abs(stamps[k] - t))
    if abs(stamps[nearest] - t) <= max_gap:
        return samples[nearest][1], samples[nearest][2]
    return None, None


#: Near-field road-plane slopes in ``lidar_tc`` as a linear function of /novatel/oem7/odom attitude
#: in degrees: ``(slope_x, slope_y) = roll * ROW0 + pitch * ROW1 + ROW2``. MEASURED with
#: scripts/lean_ab.py, pooled over selfcal_loc_2026-09-08 and -09-03 (2427 sweeps): residual road
#: tilt 1.30 deg unlevelled -> 0.51 deg. Each bag's own fit held on the other (out of sample
#: 1.41 -> 0.59 deg and 1.19 -> 0.61 deg). Why fitted, not rigid: the INS measures attitude against
#: gravity, but a banked road tilts with the car, so only ~0.8x of roll (0.5x of pitch) shows up
#: as road tilt in the sensor; the constant row is the static LiDAR mount tilt (~0.8 deg); the
#: small cross terms are the 5.35 deg yaw between lidar_tc and the vehicle axis.
ODOM_TILT_COEF = ((-0.00107, -0.01451),      # per degree of roll
                  (+0.00884, -0.00048),      # per degree of pitch
                  (-0.00919, -0.01419))      # constant


def slopes_from_attitude(roll_deg, pitch_deg, coef=ODOM_TILT_COEF, include_mount=True):
    """Predicted road-plane ``(slope_x, slope_y)`` in lidar_tc from odometry roll/pitch (deg).

    ``include_mount=False`` drops the constant row: only the attitude-driven part is returned."""
    c = np.asarray(coef, dtype=np.float64)
    s = float(roll_deg) * c[0] + float(pitch_deg) * c[1] + (c[2] if include_mount else 0.0)
    return float(s[0]), float(s[1])


class GroundSegmenter:
    """Ground segmentation with a chosen backend. ``segment(xyz) -> ground mask``."""

    def __init__(self, backend="auto", sensor_height=2.37, max_range=None, patchwork_params=None,
                 **fallback_kw):
        """``max_range`` applies to Patchwork++, whose library default is 80 m -- it does not
        segment beyond that, so the far field keeps its road returns unless this is raised.
        ``patchwork_params``: extra ``pypatchworkpp.Parameters`` attributes by name."""
        self.sensor_height = float(sensor_height)
        self.max_range = max_range
        self._fallback_kw = fallback_kw
        self._pw = None
        if backend == "auto":
            backend = "patchworkpp" if patchworkpp_available() else "fallback"
        if backend == "patchworkpp":
            import pypatchworkpp
            params = pypatchworkpp.Parameters()
            params.sensor_height = self.sensor_height
            params.verbose = False
            if self.max_range is not None:
                params.max_range = float(self.max_range)
            for name, value in (patchwork_params or {}).items():
                if not hasattr(params, name):
                    raise ValueError(f"pypatchworkpp.Parameters has no {name!r}")
                setattr(params, name, value)
            self._pw = pypatchworkpp.patchworkpp(params)
        elif backend != "fallback":
            raise ValueError(f"unknown backend {backend!r}")
        self.backend = backend

    def segment(self, xyz, intensity=None, level=None) -> np.ndarray:
        """Boolean mask over ``xyz`` (N,3), True where the point is GROUND.

        ``intensity``: per-point reflectivity scaled to [0, 1] (VLP-32C: raw / 255), or None for
        zeros. ``level``: a 3x3 rotation applied to a COPY of the cloud before segmenting (see
        levelling_rotation); the mask stays index-aligned with ``xyz``, so no inverse is needed.
        """
        xyz = np.asarray(xyz, dtype=np.float64)
        if xyz.shape[0] == 0:
            return np.zeros(0, dtype=bool)
        if level is not None:
            xyz = xyz @ np.asarray(level, dtype=np.float64).T
        if self._pw is None:
            return segment_ground_fallback(
                xyz, sensor_height=self.sensor_height, **self._fallback_kw)
        # Patchwork++ takes Nx4 (x, y, z, intensity). It exposes getGroundIndices directly --
        # an earlier version of this wrapper, written before the API was checked, rebuilt the
        # mask by matching rounded coordinates, which was slow and could collide.
        inten = (np.zeros(xyz.shape[0]) if intensity is None
                 else np.asarray(intensity, dtype=np.float64).ravel())
        self._pw.estimateGround(np.column_stack([xyz, inten]))
        mask = np.zeros(xyz.shape[0], dtype=bool)
        idx = np.asarray(self._pw.getGroundIndices(), dtype=np.int64)
        if idx.size:
            mask[idx] = True
        return mask


class LevelledGroundSegmenter:
    """Ground labels levelled by odometry attitude NEAR the car, unchanged beyond ``near_range``.

    WHY. User report: in curves, road near the car shows as non-ground. Measured with
    scripts/lean_ab.py against an independent flat-road reference inside the vehicle's path
    corridor, levelling the sweep by ``slopes_from_attitude`` before Patchwork++ beat production on
    every drive and both cross-validation directions (model fitted on one drive, scored on the
    other), for road AND off-road terrain:

        road called non-ground, straight / curve     production      levelled
        selfcal 09-08 (model fitted on 09-03)        3.4% / 0.8%     2.4% / 0.4%
        selfcal 09-03 (model fitted on 09-08)        3.1% / 6.6%     2.8% / 4.3%
        adps 08-25 11-50-45 (straight only)          0.2%            0.1%
        off-road terrain, 09-08                      4.2% / 6.0%     3.2% / 3.5%

    WHY ONLY NEAR. Levelling the whole sweep regressed the camera-LiDAR boxes: the depth-gate
    spike rate went 1.5% -> 2.6%, cones at 60-80 m 5.7% -> 18.9% (scripts/neighbour_ab.py
    --level odom) -- the ~0.8 deg constant mount term moves the far field ~1 m at 70 m. Dropping
    that term kept the boxes but lost the near-field gain (09-03 curves 6.6% -> 7.2%). So two
    Patchwork++ instances run on every sweep, one levelled, one not, and the levelled labels are
    used only inside ``near_range``: near-field numbers above, box metrics identical to
    unlevelled (--level odom-near: spikes 1.5%, radar spread unchanged to 0.01 m). Cost: one more
    Patchwork++ pass, ~4 ms.

    Pitch/roll signs are the odometry's own; the coefficients absorb the convention.
    """

    def __init__(self, backend="auto", sensor_height=2.37, max_range=None, near_range=25.0,
                 coef=ODOM_TILT_COEF, patchwork_params=None):
        self.near_range = float(near_range)
        self.coef = coef
        self.plain = GroundSegmenter(backend, sensor_height, max_range, patchwork_params)
        self.levelled = GroundSegmenter(self.plain.backend, sensor_height, max_range,
                                        patchwork_params)
        self.backend = self.plain.backend

    def segment(self, xyz, roll_deg=None, pitch_deg=None):
        """Ground mask. Without attitude (None) this is exactly the unlevelled GroundSegmenter."""
        xyz = np.asarray(xyz, dtype=np.float64)
        plain = self.plain.segment(xyz)
        if roll_deg is None or pitch_deg is None or xyz.shape[0] == 0:
            return plain
        level = levelling_rotation(*slopes_from_attitude(roll_deg, pitch_deg, self.coef))
        near = np.hypot(xyz[:, 0], xyz[:, 1]) < self.near_range
        return np.where(near, self.levelled.segment(xyz, level=level), plain)
