"""Obstacle-robust road boundaries from road-labelled LiDAR points.

The two existing producers -- ``sphereformer_ros`` and SAM3 + ``sam3_mask_transform`` -- both
publish the OUTERMOST extent of whatever they labelled road. An obstacle near the edge hides
the road behind it (a car is labelled car, and its LiDAR/camera shadow is labelled nothing),
so the extent, and with it the published boundary, jumps inward to the obstacle's inner side.
SAM3 additionally keeps only the largest mask contour and splits it at the row's mean x, so a
car's outline lands in the boundary and a car cutting the far road off removes that road.

This module re-derives the boundary from road-labelled points, whatever labelled them, in four
independently switchable steps:

1. OCCLUSION. A per-bin road extreme with an obstacle (or an obstacle's shadow) just outside
   it is not an edge, it is the end of what could be seen. It is marked OCCLUDED and removed,
   never moved.
2. EXTRACTION. Only road continuing the ego corridor is kept: per bin, the lateral run of road
   (obstacles and their shadows bridging gaps) that overlaps the previous bin's run -- so a car
   splitting the road does not disconnect the far road, and a road-labelled patch across a
   field does not become an extreme.
3. FIT. A robust (Huber) polynomial per side over the surviving samples bridges the removed
   ones, with bounded extrapolation; a side with too little support is placed at the other
   side offset by the road width.
4. TRACKING. The boundary is held at fixed stations ahead of the vehicle, carried forward
   with ego motion between frames, and updated per station by a scalar Kalman filter, so a
   stretch hidden for a moment coasts on what was seen before instead of disappearing.

Everything here is in the ``ego`` frame (vehicle-longitudinal, origin at the LiDAR; see
:mod:`object_fusion.frames`) -- binning along ``lidar_tc`` x instead would shear the bins by
the 5.35 deg mount yaw. Free of rclpy, so the unit tests and offline A/B exercise exactly the
production rule.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

from object_fusion import frames
from object_fusion.ego_motion import EgoTwist, TwistBuffer

__all__ = [
    "ROAD_CLASS",
    "OBSTACLE_CLASSES",
    "decode_semkitti_rgb",
    "BoundaryParams",
    "EDGE",
    "OCCLUDED",
    "EMPTY",
    "UNSUPPORTED",
    "ShadowMap",
    "box_footprint_points",
    "select_run",
    "bin_extremes",
    "fit_side",
    "Measurement",
    "measure",
    "BoundaryTracker",
    "resample",
    "mask_road_pixels",
    "BoundaryPipeline",
]

# ---- SemanticKITTI learned classes as sphereformer_ros colours them ----------------------
# Copied from src/sphereformer_ros/src/SphereFormer_changes/semantic-kitti.yaml (learning_map_inv
# + color_map, BGR there, RGB here -- the node swaps them). That yaml is not installed in this
# image, and the colours are the fixed SemanticKITTI palette, so restating them is safe; the
# unit test pins the road and car entries.
_SEMKITTI_RGB = np.array([
    (0, 0, 0),          # 0 unlabeled
    (100, 150, 245),    # 1 car
    (100, 230, 245),    # 2 bicycle
    (30, 60, 150),      # 3 motorcycle
    (80, 30, 180),      # 4 truck
    (0, 0, 255),        # 5 other-vehicle
    (255, 30, 30),      # 6 person
    (255, 40, 200),     # 7 bicyclist
    (150, 30, 90),      # 8 motorcyclist
    (255, 0, 255),      # 9 road
    (255, 150, 255),    # 10 parking
    (75, 0, 75),        # 11 sidewalk
    (175, 0, 75),       # 12 other-ground
    (255, 200, 0),      # 13 building
    (255, 120, 50),     # 14 fence
    (0, 175, 0),        # 15 vegetation
    (135, 60, 0),       # 16 trunk
    (150, 240, 80),     # 17 terrain
    (255, 240, 150),    # 18 pole
    (255, 0, 0),        # 19 traffic-sign
], dtype=np.uint32)

ROAD_CLASS = 9
#: Things that sit ON the road and hide it. Static scenery (poles, fences) is not in the set:
#: it bounds the road rather than occluding it.
OBSTACLE_CLASSES = (1, 2, 3, 4, 5, 6, 7, 8)

_PACKED = (_SEMKITTI_RGB[:, 0] << 16) | (_SEMKITTI_RGB[:, 1] << 8) | _SEMKITTI_RGB[:, 2]
_ORDER = np.argsort(_PACKED)
_PACKED_SORTED = _PACKED[_ORDER]


def decode_semkitti_rgb(rgb) -> np.ndarray:
    """Class ids from a PointCloud2 ``rgb`` field (float32-packed or uint32). -1 = unknown."""
    rgb = np.asarray(rgb)
    packed = (rgb.view(np.uint32) if rgb.dtype == np.float32 else rgb.astype(np.uint32))
    packed = packed & 0xFFFFFF
    i = np.clip(np.searchsorted(_PACKED_SORTED, packed), 0, len(_PACKED_SORTED) - 1)
    hit = _PACKED_SORTED[i] == packed
    return np.where(hit, _ORDER[i], -1).astype(np.int32)


# ---- parameters ---------------------------------------------------------------------------

@dataclass
class BoundaryParams:
    # Region. x_min skips the hood/near-blind ring; x_max matches what the segmenters label
    # reliably. INVENTED, to be tuned on replay.
    x_min: float = 5.0
    x_max: float = 60.0
    y_max: float = 25.0
    bin_len: float = 2.0
    min_bin_points: int = 3
    # Occlusion: how far beyond a road extreme an obstacle or its shadow makes it "occluded".
    occl_margin: float = 1.0
    az_res_deg: float = 0.5
    # Obstacle points closer than this are the ego vehicle itself: SphereFormer labels the roof
    # and hood returns (0.8-0.9 m from lidar_tc) "car", and each one would shadow a whole
    # bearing. Measured on perception_input_2026-09-22_14-21-45: 42% of samples read occluded
    # until these were excluded.
    ego_radius: float = 3.0
    # Obstacle returns needed before a bearing is shadowed, or before an extreme counts as having
    # an obstacle just beyond it. Single misclassified "car" points were enough to occlude whole
    # bearings on the selfcal replay. Box-filled fused objects are dense and always qualify.
    min_obstacle_points: int = 3
    # Edge evidence: a non-road return within this distance beyond the extreme makes it an edge.
    # Without it, a bin that only caught a short arc of a LiDAR ring (rings are circles; a 2 m
    # x-bin often holds a ring segment that stops well inside the road) reads as a narrow road.
    edge_evidence_m: float = 1.0
    min_other_in_gap: int = 2
    # Extraction: lateral gap that separates two road runs within one bin -- when non-road returns
    # lie in it -- and the corridor the first bin's run must overlap.
    run_gap: float = 1.5
    seed_half_width: float = 2.0
    # Fit.
    huber_delta: float = 0.3
    min_fit_samples: int = 4
    max_extrap: float = 10.0
    min_support_frac: float = 0.3
    default_width: float = 7.0
    min_width: float = 2.5
    min_width_bins: int = 3
    # Seen road is a floor under the fit: an OCCLUDED or UNSUPPORTED extreme is not an edge,
    # but the road visibly reaches at least that far, so the published edge is pushed out to
    # it -- by at most this much, so one misclassified road return cannot drag it outward.
    max_floor_push: float = 1.5
    # Measurement sigmas handed to the tracker [m]: supported, extrapolated, mirrored.
    sigma_meas: float = 0.3
    sigma_extrap: float = 0.6
    sigma_mirror: float = 0.8
    # Tracking.
    q_per_s: float = 0.05        # m^2 of lateral random walk per second
    q_per_m: float = 0.01        # m^2 per metre travelled (road curvature the frame shift misses)
    gate_m: float = 1.5
    reset_frac: float = 0.6
    reset_min_compared: int = 4
    # A station whose measurement is gated this many frames running takes the measurement: by
    # then the TRACK is what is wrong. Without it one bad initialisation coasted for coast_s
    # (measured: a left edge stuck at -4.4 m, right of the right edge, for ~1 s through lmp_v10c).
    reinit_after_gated: int = 3
    coast_s: float = 3.0
    # Output.
    output_step: float = 1.0

    def update(self, **kw):
        for k, v in kw.items():
            if not hasattr(self, k):
                raise KeyError(k)
            setattr(self, k, type(getattr(self, k))(v))

    def anchors(self) -> np.ndarray:
        n = int(math.floor((self.x_max - self.x_min) / self.bin_len))
        return self.x_min + self.bin_len * (np.arange(n) + 0.5)


#: Per-bin side states. EDGE: non-road seen just beyond the road extreme. OCCLUDED: an obstacle
#: or its shadow just beyond it. UNSUPPORTED: nothing seen beyond it -- the bin's ring arcs
#: simply stopped there. EMPTY: no road in the bin. Only EDGE samples are fitted.
EDGE, OCCLUDED, EMPTY, UNSUPPORTED = 0, 1, 2, 3


# ---- step 1 primitives --------------------------------------------------------------------

class ShadowMap:
    """Per-azimuth nearest obstacle range, seen from the sensor at the frame origin.

    A point is shadowed when it lies at or beyond the nearest obstacle on its bearing: the
    sensor could not have seen road there whether or not road is there. Neighbouring azimuth
    bins are folded in so a sparse obstacle does not leave stripes of "visible" between returns.
    """

    def __init__(self, obstacle_xy, az_res_deg: float = 0.5, min_range: float = 1.0,
                 min_points: int = 1):
        self.res = math.radians(az_res_deg)
        self.n = int(round(2 * math.pi / self.res))
        self.min_r = np.full(self.n, np.inf)
        xy = np.asarray(obstacle_xy, dtype=np.float64).reshape(-1, 2)
        if xy.shape[0]:
            r = np.hypot(xy[:, 0], xy[:, 1])
            keep = r >= min_range
            if keep.any():
                b, r = self._bin(xy[keep]), r[keep]
                if min_points <= 1:
                    np.minimum.at(self.min_r, b, r)
                else:
                    # The min_points-th nearest return per bearing: one misclassified point must
                    # not shadow a whole bearing out to infinity.
                    order = np.lexsort((r, b))
                    b, r = b[order], r[order]
                    first = np.r_[0, np.flatnonzero(np.diff(b)) + 1]
                    counts = np.diff(np.r_[first, b.size])
                    ok = counts >= min_points
                    self.min_r[b[first[ok]]] = r[first[ok] + min_points - 1]
                self.min_r = np.minimum(self.min_r,
                                        np.minimum(np.roll(self.min_r, 1), np.roll(self.min_r, -1)))

    def _bin(self, xy):
        az = np.arctan2(xy[:, 1], xy[:, 0])
        return np.floor((az + math.pi) / self.res).astype(np.int64) % self.n

    def occluded(self, xy) -> np.ndarray:
        xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        if not np.isfinite(self.min_r).any():
            return np.zeros(xy.shape[0], dtype=bool)
        return np.hypot(xy[:, 0], xy[:, 1]) >= self.min_r[self._bin(xy)]


def box_footprint_points(centers, sizes, yaws, margin: float = 0.0, step: float = 0.4):
    """Fill each oriented box footprint (inflated by ``margin``) with points ``step`` apart."""
    out = []
    for (cx, cy), (lx, ly), yaw in zip(np.asarray(centers).reshape(-1, 2),
                                       np.asarray(sizes).reshape(-1, 2), np.asarray(yaws).ravel()):
        hx, hy = 0.5 * max(lx, 0.5) + margin, 0.5 * max(ly, 0.5) + margin
        gx = np.linspace(-hx, hx, max(2, int(math.ceil(2 * hx / step)) + 1))
        gy = np.linspace(-hy, hy, max(2, int(math.ceil(2 * hy / step)) + 1))
        g = np.stack(np.meshgrid(gx, gy), axis=-1).reshape(-1, 2)
        c, s = math.cos(yaw), math.sin(yaw)
        out.append(g @ np.array([[c, s], [-s, c]]) + (cx, cy))
    return np.concatenate(out) if out else np.empty((0, 2))


# ---- step 2 -------------------------------------------------------------------------------

def select_run(ys_road, ys_obs, ref, gap: float, bridged=None, ys_other=None,
               min_other: int = 2):
    """The lateral run of this bin that continues the road the vehicle is on.

    Road and obstacle ys together are split into runs wherever consecutive points are more than
    ``gap`` apart AND at least ``min_other`` non-road returns (``ys_other``) lie in the gap -- an
    empty gap is a sampling hole between ring arcs, not a separation -- and not where
    ``bridged(y_mid)`` says the gap is an obstacle's shadow, which hides road rather than
    separating it. With ``ys_other=None`` every wide gap separates. Of the runs holding road, the one overlapping ``ref`` (the
    previous bin's chosen extent) most is returned as ``(lo, hi)`` over ROAD points only; None
    when no run touches ``ref``. Obstacles join runs but never set an extreme: a car on the edge
    keeps the road it hides connected without becoming the boundary itself.

    Why per-bin runs rather than 2D connectivity on a BEV grid: LiDAR ground rings are 3-6 m
    apart along x beyond 20 m (measured on perception_input_2026-09-22_14-21-45), so any grid
    fine enough to resolve a road edge splits the road at every ring. Bins that hold no ring are
    simply skipped here, and ``ref`` carries across them.
    """
    ys_road = np.asarray(ys_road, dtype=np.float64)
    if ys_road.size == 0:
        return None
    ys_obs = np.asarray(ys_obs, dtype=np.float64)
    ys = np.concatenate([ys_road, ys_obs])
    is_road = np.concatenate([np.ones(ys_road.size, bool), np.zeros(ys_obs.size, bool)])
    order = np.argsort(ys, kind="stable")
    ys, is_road = ys[order], is_road[order]
    breaks = np.flatnonzero(np.diff(ys) > gap)
    if ys_other is not None and breaks.size:
        oth = np.sort(np.asarray(ys_other, dtype=np.float64))
        n_in = (np.searchsorted(oth, ys[breaks + 1], side="left")
                - np.searchsorted(oth, ys[breaks], side="right"))
        breaks = breaks[n_in >= min_other]
    if bridged is not None and breaks.size:
        mids = 0.5 * (ys[breaks] + ys[breaks + 1])
        breaks = breaks[~bridged(mids)]
    lo_ref, hi_ref = ref
    best, best_key = None, None
    for run in np.split(np.arange(ys.size), breaks + 1):
        r = run[is_road[run]]
        if r.size == 0:
            continue
        lo, hi = ys[run[0]], ys[run[-1]]
        overlap = min(hi, hi_ref) - max(lo, lo_ref)
        if overlap < -gap:
            continue
        key = (overlap, -abs(0.5 * (lo + hi) - 0.5 * (lo_ref + hi_ref)))
        if best_key is None or key > best_key:
            best, best_key = (ys[r[0]], ys[r[-1]]), key
    return best


# ---- per-bin extremes (+ step 1) ----------------------------------------------------------

def bin_extremes(road_xy, road_z, obstacle_xy, shadow: ShadowMap, p: BoundaryParams,
                 occlusion: bool = True, extraction: bool = True, other_xy=None):
    """Leftmost / rightmost road per x-bin, with an EDGE / OCCLUDED / EMPTY state per side.

    Returns ``(xc, left, right, left_state, right_state, z, kept_fraction)``.
    """
    xc = p.anchors()
    nb = xc.size
    left = np.full(nb, np.nan)
    right = np.full(nb, np.nan)
    z = np.full(nb, np.nan)
    ls = np.full(nb, EMPTY, dtype=np.int8)
    rs = np.full(nb, EMPTY, dtype=np.int8)
    road_xy = np.asarray(road_xy, dtype=np.float64).reshape(-1, 2)
    road_z = np.asarray(road_z, dtype=np.float64).ravel()
    obs = np.asarray(obstacle_xy, dtype=np.float64).reshape(-1, 2)

    b = np.floor((road_xy[:, 0] - p.x_min) / p.bin_len).astype(np.int64)
    ok = (b >= 0) & (b < nb) & (np.abs(road_xy[:, 1]) <= p.y_max)
    ob = np.floor((obs[:, 0] - p.x_min) / p.bin_len).astype(np.int64) if obs.shape[0] else None
    oth = None if other_xy is None else np.asarray(other_xy, dtype=np.float64).reshape(-1, 2)
    othb = (np.floor((oth[:, 0] - p.x_min) / p.bin_len).astype(np.int64)
            if oth is not None else None)
    probes = np.linspace(0.25, p.occl_margin, max(2, int(round(p.occl_margin / 0.25))))
    ref = (-p.seed_half_width, p.seed_half_width)
    n_in, n_kept = int(ok.sum()), 0

    for i in range(nb):
        m = ok & (b == i)
        if m.sum() < p.min_bin_points:
            if occlusion and shadow.occluded([[xc[i], 0.5 * (ref[0] + ref[1])]])[0]:
                ls[i] = rs[i] = OCCLUDED
            continue
        ys = road_xy[m, 1]
        zs = road_z[m] if road_z.size else np.full(ys.size, np.nan)
        near_obs = (obs[np.abs(ob - i) <= 0, 1] if ob is not None else np.empty(0))
        other_y = oth[othb == i, 1] if oth is not None else None
        if extraction:
            x_i = xc[i]
            run = select_run(ys, near_obs, ref, p.run_gap,
                             bridged=lambda y: shadow.occluded(
                                 np.column_stack((np.full(y.size, x_i), y))),
                             ys_other=other_y, min_other=p.min_other_in_gap)
            if run is None:
                continue
            keep = (ys >= run[0]) & (ys <= run[1])
            if keep.sum() < p.min_bin_points:
                continue
            ys, zs = ys[keep], zs[keep]
            ref = run
        n_kept += ys.size
        left[i], right[i] = ys.max(), ys.min()
        z[i] = np.median(zs)
        ls[i] = rs[i] = EDGE
        if not occlusion:
            continue
        near_obs = (obs[np.abs(ob - i) <= 1, 1] if ob is not None else np.empty(0))
        for y0, sgn, st in ((left[i], 1.0, ls), (right[i], -1.0, rs)):
            probe = np.column_stack((np.full(probes.size, xc[i]), y0 + sgn * probes))
            hit = shadow.occluded(probe).any()
            if not hit and near_obs.size:
                d = sgn * (near_obs - y0)
                hit = int(((d >= -0.5) & (d <= p.occl_margin)).sum()) >= p.min_obstacle_points
            if hit:
                st[i] = OCCLUDED
            elif other_y is not None:
                d = sgn * (other_y - y0)
                if not ((d > 0.0) & (d <= p.edge_evidence_m)).any():
                    st[i] = UNSUPPORTED
    return xc, left, right, ls, rs, z, (n_kept / n_in if n_in else 1.0)


# ---- step 3 -------------------------------------------------------------------------------

@dataclass
class Curve:
    coeffs: np.ndarray            # highest power first, in x / 50 m
    x_lo: float
    x_hi: float

    def __call__(self, x):
        return np.polyval(self.coeffs, np.asarray(x, dtype=np.float64) / 50.0)


def fit_side(x, y, p: BoundaryParams):
    """Huber IRLS polynomial y(x); degree grows with the supported span. None if too sparse."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    if x.size < p.min_fit_samples:
        return None
    span = x.max() - x.min()
    deg = 1 if span < 12.0 else (2 if span < 25.0 else 3)
    # At least three samples per coefficient beyond the line, or a cubic chases two outliers.
    deg = min(deg, max(1, x.size // 3))
    if deg < 1:
        return None
    xs = x / 50.0
    w = np.ones_like(x)
    coeffs = np.polyfit(xs, y, deg)
    for _ in range(8):
        r = np.abs(y - np.polyval(coeffs, xs))
        w = np.where(r <= p.huber_delta, 1.0, p.huber_delta / np.maximum(r, 1e-9))
        coeffs = np.polyfit(xs, y, deg, w=np.sqrt(w))
    return Curve(coeffs, float(x.min()), float(x.max()))


# ---- one frame ----------------------------------------------------------------------------

@dataclass
class Measurement:
    x: np.ndarray
    left: np.ndarray              # NaN where no measurement
    right: np.ndarray
    sigma_left: np.ndarray
    sigma_right: np.ndarray
    z: np.ndarray
    left_state: np.ndarray        # per-bin EDGE / OCCLUDED / EMPTY before the fit
    right_state: np.ndarray
    width: float                  # median width over bins where both sides are EDGE; NaN if none
    kept_fraction: float          # road points surviving extraction
    raw_left: np.ndarray          # per-bin road extremes before occlusion/fit, for markers
    raw_right: np.ndarray
    stats: dict = field(default_factory=dict)


def _interp_supported(xc, y, valid):
    out = np.full(xc.size, np.nan)
    idx = np.flatnonzero(valid)
    if idx.size == 0:
        return out
    span = (xc >= xc[idx[0]]) & (xc <= xc[idx[-1]])
    out[span] = np.interp(xc[span], xc[idx], y[idx])
    return out


def measure(road_xy, road_z, obstacle_xy, p: BoundaryParams, *, occlusion=True,
            extraction=True, fit=True, width_hint=None, other_xy=None) -> Measurement:
    """Steps 1-3 on one frame, all in the ego frame.

    ``other_xy`` are returns positively labelled NOT road and not an obstacle (terrain,
    vegetation, sidewalk...). They are the evidence that an extreme is an edge and that a lateral
    gap is a separation; without them (None) neither test is applied.
    """
    road_xy = np.asarray(road_xy, dtype=np.float64).reshape(-1, 2)
    road_z = np.asarray(road_z, dtype=np.float64).ravel()
    obstacle_xy = np.asarray(obstacle_xy, dtype=np.float64).reshape(-1, 2)
    obstacle_xy = obstacle_xy[np.hypot(obstacle_xy[:, 0], obstacle_xy[:, 1]) >= p.ego_radius]
    shadow = ShadowMap(obstacle_xy if occlusion or extraction else np.empty((0, 2)),
                       p.az_res_deg, min_points=p.min_obstacle_points)

    xc, left, right, ls, rs, z, kept = bin_extremes(road_xy, road_z, obstacle_xy, shadow, p,
                                                    occlusion=occlusion, extraction=extraction,
                                                    other_xy=other_xy)
    both = (ls == EDGE) & (rs == EDGE)
    width = (float(np.median(left[both] - right[both])) if both.sum() >= p.min_width_bins
             else float("nan"))
    if not (width >= p.min_width):
        width = float("nan")

    sig_l = np.full(xc.size, p.sigma_meas)
    sig_r = np.full(xc.size, p.sigma_meas)
    lv, rv = ls == EDGE, rs == EDGE
    if not fit:
        yl, yr = _interp_supported(xc, left, lv), _interp_supported(xc, right, rv)
    else:
        yl, yr = np.full(xc.size, np.nan), np.full(xc.size, np.nan)
        seen = (ls != EMPTY) | (rs != EMPTY)
        n_seen = max(1, int(seen.sum()))
        curves = {}
        for name, y, v in (("l", left, lv), ("r", right, rv)):
            c = fit_side(xc[v], y[v], p) if v.sum() / n_seen >= p.min_support_frac else None
            curves[name] = c
        for name, out, sig in (("l", yl, sig_l), ("r", yr, sig_r)):
            c = curves[name]
            if c is None:
                continue
            dom = (xc >= max(p.x_min, c.x_lo - p.max_extrap)) & (xc <= min(p.x_max, c.x_hi + p.max_extrap))
            out[dom] = c(xc[dom])
            sig[dom & ((xc < c.x_lo) | (xc > c.x_hi))] = p.sigma_extrap
        w = width if np.isfinite(width) else (width_hint if width_hint and width_hint >= p.min_width
                                              else p.default_width)
        if curves["l"] is None and curves["r"] is not None:
            m = np.isfinite(yr)
            yl[m], sig_l[m] = yr[m] + w, p.sigma_mirror
        elif curves["r"] is None and curves["l"] is not None:
            m = np.isfinite(yl)
            yr[m], sig_r[m] = yl[m] - w, p.sigma_mirror
        for out, raw, st, sgn in ((yl, left, ls, 1.0), (yr, right, rs, -1.0)):
            floor = np.isfinite(out) & np.isfinite(raw) & ((st == OCCLUDED) | (st == UNSUPPORTED))
            push = np.clip(sgn * (raw - out), 0.0, p.max_floor_push)
            out[floor] += sgn * push[floor]

    # A road narrower than min_width (or with its edges crossed) is not a measurement of a road:
    # a sparse fit extrapolated through an occluded stretch produced a left edge 1.2 m RIGHT of
    # the right edge on the upstream replay.
    _drop_crossed(yl, yr, p.min_width)

    zf = z.copy()
    if np.isfinite(z).any():
        idx = np.flatnonzero(np.isfinite(z))
        zf = np.interp(xc, xc[idx], z[idx])
    stats = {
        "occluded_l": int((ls == OCCLUDED).sum()), "occluded_r": int((rs == OCCLUDED).sum()),
        "edge_l": int(lv.sum()), "edge_r": int(rv.sum()),
        "unsupported_l": int((ls == UNSUPPORTED).sum()), "unsupported_r": int((rs == UNSUPPORTED).sum()),
    }
    return Measurement(xc, yl, yr, sig_l, sig_r, zf, ls, rs, width, kept, left, right, stats)


# ---- step 4 -------------------------------------------------------------------------------

class _Side:
    def __init__(self, n):
        self.y = np.full(n, np.nan)
        self.var = np.full(n, np.inf)
        self.t_upd = np.full(n, -np.inf)
        self.n_gated = np.zeros(n, dtype=np.int64)


def _drop_crossed(left, right, min_width, t_left=None, t_right=None):
    """NaN out stations where the edges are closer than ``min_width`` (or crossed).

    Neither side can be trusted there; with update times given, only the side updated longer ago
    is dropped (the fresher one is the better evidence), else both are.
    """
    bad = np.isfinite(left) & np.isfinite(right) & (left - right < min_width)
    if not bad.any():
        return bad
    if t_left is None:
        left[bad] = np.nan
        right[bad] = np.nan
        return bad
    older_left = bad & (t_left < t_right)
    older_right = bad & (t_right < t_left)
    same = bad & ~older_left & ~older_right
    left[older_left | same] = np.nan
    right[older_right | same] = np.nan
    return bad


class BoundaryTracker:
    """Per-station scalar Kalman filters on lateral offset, carried forward by ego motion.

    State lives at the fixed stations ``p.anchors()`` in the CURRENT ego frame. Between frames
    the frame moves by ``(dpsi, d)`` (from :meth:`object_fusion.ego_motion.TwistBuffer.increment`,
    expressed in the old frame); the held boundary is transformed as world-fixed points,
    ``p' = R(dpsi)^T (p - d)``, and resampled at the stations without extrapolating. Stations
    the vehicle has driven onto from beyond the held range start empty.

    A station whose innovation is outside the gate is not updated (it coasts). When most
    compared stations of a side fail the gate together the road itself has changed -- a junction,
    a lane-change into a different carriageway -- and that side is re-initialised from the
    measurement rather than fought.
    """

    def __init__(self, p: BoundaryParams):
        self.p = p
        self.reset()

    def reset(self):
        n = self.p.anchors().size
        self.sides = {"l": _Side(n), "r": _Side(n)}
        self.z = np.full(n, np.nan)
        self.t = None
        self.width = float("nan")
        self.resets = 0
        self.gated = 0
        self.reinits = 0

    def predict(self, t: float, increment) -> bool:
        """Move the held boundary to time ``t``. False (and a reset) when it cannot be moved."""
        if self.t is None:
            self.t = t
            return True
        dt = t - self.t
        if dt <= 0.0:
            return True
        if increment is None:
            self.reset()
            self.t = t
            return False
        dpsi, d = increment
        d = np.asarray(d, dtype=np.float64).reshape(2)
        c, s = math.cos(dpsi), math.sin(dpsi)
        Rt = np.array([[c, s], [-s, c]])
        xa = self.p.anchors()
        q = self.p.q_per_s * dt + self.p.q_per_m * float(np.hypot(*d))
        for side in self.sides.values():
            v = np.isfinite(side.y)
            ny, nvar, nt = np.full(xa.size, np.nan), np.full(xa.size, np.inf), np.full(xa.size, -np.inf)
            ng = np.zeros(xa.size, dtype=np.int64)
            if v.sum() >= 2:
                pts = (np.column_stack((xa[v], side.y[v])) - d) @ Rt.T
                order = np.argsort(pts[:, 0])
                px, py = pts[order, 0], pts[order, 1]
                inside = (xa >= px[0]) & (xa <= px[-1])
                ny[inside] = np.interp(xa[inside], px, py)
                nvar[inside] = np.interp(xa[inside], px, side.var[v][order]) + q
                # The oldest update among the two neighbours, so a coasted stretch cannot be
                # refreshed by being interpolated against a fresh one.
                tv = side.t_upd[v][order]
                j = np.clip(np.searchsorted(px, xa[inside]), 1, px.size - 1)
                nt[inside] = np.minimum(tv[j - 1], tv[j])
                gv = side.n_gated[v][order]
                ng[inside] = np.maximum(gv[j - 1], gv[j])
            side.y, side.var, side.t_upd, side.n_gated = ny, nvar, nt, ng
        self.t = t
        return True

    def update(self, m: Measurement):
        p = self.p
        r_gate = p.gate_m
        for name, my, ms in (("l", m.left, m.sigma_left), ("r", m.right, m.sigma_right)):
            side = self.sides[name]
            mv = np.isfinite(my)
            tv = np.isfinite(side.y)
            both = mv & tv
            r = ms ** 2
            innov = my - side.y
            bad = both & (np.abs(innov) > r_gate + 3.0 * np.sqrt(np.where(both, side.var + r, 0.0)))
            if both.sum() >= p.reset_min_compared and bad.sum() >= p.reset_frac * both.sum():
                self.resets += 1
                side.y[:] = np.nan
                side.var[:] = np.inf
                side.t_upd[:] = -np.inf
                side.n_gated[:] = 0
                tv[:] = False
                both[:] = False
                bad[:] = False
            self.gated += int(bad.sum())
            side.n_gated[bad] += 1
            reinit = bad & (side.n_gated >= p.reinit_after_gated)
            if reinit.any():
                self.reinits += int(reinit.sum())
                side.y[reinit] = my[reinit]
                side.var[reinit] = r[reinit] * 4.0
                side.t_upd[reinit] = self.t
                side.n_gated[reinit] = 0
            upd = both & ~bad
            k = side.var[upd] / (side.var[upd] + r[upd])
            side.y[upd] += k * innov[upd]
            side.var[upd] *= (1.0 - k)
            side.t_upd[upd] = self.t
            init = mv & ~tv
            side.y[init] = my[init]
            side.var[init] = r[init] * 4.0
            side.t_upd[init] = self.t
            side.n_gated[upd | init] = 0
            stale = np.isfinite(side.y) & (self.t - side.t_upd > p.coast_s)
            side.y[stale] = np.nan
            side.var[stale] = np.inf
        L, R = self.sides["l"], self.sides["r"]
        crossed = _drop_crossed(L.y, R.y, p.min_width, L.t_upd, R.t_upd)
        for side in (L, R):
            gone = crossed & ~np.isfinite(side.y)
            side.var[gone] = np.inf
            side.n_gated[gone] = 0
        zm = np.isfinite(m.z)
        self.z[zm] = m.z[zm]
        if np.isfinite(m.width):
            self.width = m.width if not np.isfinite(self.width) else 0.8 * self.width + 0.2 * m.width

    def output(self):
        return self.sides["l"].y.copy(), self.sides["r"].y.copy(), self.z.copy()


def resample(x, y, z, step: float):
    """Contiguous valid runs of ``y`` resampled to ``step`` spacing. Returns (N,3) ego xyz."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    z = np.asarray(z, dtype=np.float64)
    v = np.isfinite(y)
    if v.sum() == 0:
        return np.empty((0, 3))
    zfill = z if np.isfinite(z).all() else np.where(np.isfinite(z), z, np.nanmedian(z) if np.isfinite(z).any() else 0.0)
    out = []
    idx = np.flatnonzero(v)
    runs = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
    for run in runs:
        if run.size == 1:
            out.append(np.array([[x[run[0]], y[run[0]], zfill[run[0]]]]))
            continue
        xs = np.arange(x[run[0]], x[run[-1]] + 1e-9, step)
        out.append(np.column_stack((xs, np.interp(xs, x[run], y[run]), np.interp(xs, x[run], zfill[run]))))
    return np.concatenate(out)


# ---- one pipeline per node, shared with the offline harness ---------------------------------

def mask_road_pixels(mask, extraction: bool = True) -> np.ndarray:
    """Road pixels from SAM3's mask-id image (0 = background, 1..N = mask ids).

    The image arrives JPEG-compressed, so a 3x3 opening removes the ringing compression leaves
    around the outline -- that is decoding, not a step to switch off. With ``extraction`` the
    holes inside the road (a car, a person) are filled: road continues under them, and the
    ground flag still keeps the lifted points off the obstacle itself.
    """
    road = ndimage.binary_opening(np.asarray(mask) >= 1, structure=np.ones((3, 3), bool))
    return ndimage.binary_fill_holes(road) if extraction else road


SOURCES = ("sphereformer", "sam3")
STEPS = ("occlusion", "extraction", "fit", "tracking")


class BoundaryPipeline:
    """Everything the node does between decoded inputs and published points, minus rclpy.

    Inputs arrive in ``lidar_tc`` (clouds) or ``ego`` (fused objects); work happens in ``ego``;
    outputs are (N,3) ``lidar_tc`` points, so they drop in for the raw topics.
    """

    def __init__(self, p: BoundaryParams | None = None, ego_yaw_deg: float | None = None,
                 objects_max_age: float = 0.5, obstacle_max_skew: float = 0.15):
        self.p = p or BoundaryParams()
        self.ego_yaw_deg = frames.EGO_YAW_IN_LIDAR_DEG if ego_yaw_deg is None else ego_yaw_deg
        self.steps = {k: True for k in STEPS}
        self.use_objects = True
        self.objects_max_age = objects_max_age
        self.obstacle_max_skew = obstacle_max_skew
        self.twist = TwistBuffer(duration=8.0)
        self.trackers = {s: BoundaryTracker(self.p) for s in SOURCES}
        self._objects = deque(maxlen=64)
        self._seg = deque(maxlen=20)          # (stamp, obstacle ego xy) per SphereFormer sweep
        self.rewinds = 0
        self.stats = {s: {"frames": 0, "bins": 0, "occluded": 0, "unsupported": 0,
                          "empty_sides": 0, "no_odom": 0} for s in SOURCES}

    def reset(self):
        for tr in self.trackers.values():
            tr.reset()

    #: An odometry stamp this far behind the newest one is a bag rewind or a switch to an older
    #: bag, not jitter. TwistBuffer drops out-of-order samples by design, so without this a
    #: replay of an older bag after a newer one starves odometry for the whole run (measured:
    #: no_odom on 493 of 746 frames after switching from a Sep 22 to a Sep 8 bag).
    REWIND_S = 1.0

    # -- inputs --
    def add_odom(self, t, vx, vy, wz):
        newest = self.twist.newest()
        if newest is not None and t < newest.stamp - self.REWIND_S:
            self.rewinds += 1
            self.twist = TwistBuffer(duration=8.0)
            self._objects.clear()
            self._seg.clear()
            self.reset()
        self.twist.add(EgoTwist(t, vx, vy, wz))

    def add_objects(self, t, centers, sizes, yaws, vels):
        self._objects.append((t, np.asarray(centers, float).reshape(-1, 2),
                              np.asarray(sizes, float).reshape(-1, 2),
                              np.asarray(yaws, float).ravel(), np.asarray(vels, float).reshape(-1, 2)))

    def sphereformer_frame(self, t, xyz, cls):
        """One labelled sweep (lidar_tc xyz, SemanticKITTI class ids) -> (left, right, m)."""
        xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        cls = np.asarray(cls)
        ego = self._to_ego(xyz[:, :2])
        obs = ego[np.isin(cls, OBSTACLE_CLASSES)]
        self._seg.append((t, obs))
        road = cls == ROAD_CLASS
        other = cls >= 10                    # parking .. traffic-sign: seen, and not road
        return self._process("sphereformer", t, ego[road], xyz[road, 2], obs, ego[other])

    def sam3_frame(self, t, xyz, u, v, ground, road_px, image_wh):
        """SAM3 road pixels lifted onto one projection (lidar_tc xyz + pixel u, v)."""
        xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        mh, mw = road_px.shape
        W, H = image_wh
        ui = np.clip((np.asarray(u) * (mw / W)).astype(np.int64), 0, mw - 1)
        vi = np.clip((np.asarray(v) * (mh / H)).astype(np.int64), 0, mh - 1)
        road = road_px[vi, ui]
        other = ~road
        if ground is not None and self.steps["extraction"]:
            # A road pixel over a car's lower body lifts onto the car; road is ground.
            road = road & ground
        ego = self._to_ego(xyz[:, :2])
        return self._process("sam3", t, ego[road], xyz[road, 2], self._nearest_seg(t), ego[other])

    # -- internals --
    def _to_ego(self, xy):
        return frames.lidar_to_ego(xy, correction_deg=self.ego_yaw_deg)

    def _to_lidar(self, xy):
        return frames.ego_to_lidar(xy, correction_deg=self.ego_yaw_deg)

    def _nearest_seg(self, t):
        best = None
        for ts, obs in self._seg:
            d = abs(ts - t)
            if d <= self.obstacle_max_skew and (best is None or d < best[0]):
                best = (d, obs)
        return best[1] if best else np.empty((0, 2))

    def object_points(self, t):
        """Fused boxes carried to ``t`` (own velocity, then the ego frame's motion), filled."""
        if not self.use_objects:
            return np.empty((0, 2))
        cands = [o for o in self._objects if abs(t - o[0]) <= self.objects_max_age]
        if not cands:
            return np.empty((0, 2))
        ts, c, sz, yaw, vel = min(cands, key=lambda o: abs(t - o[0]))
        if c.shape[0] == 0:
            return np.empty((0, 2))
        dt = t - ts
        c = c + vel * dt
        if dt > 0.0:
            inc = self.twist.increment(ts, t)
            if inc is None:
                return np.empty((0, 2))
            dpsi, d = inc
            cs, sn = math.cos(dpsi), math.sin(dpsi)
            c = (c - d) @ np.array([[cs, sn], [-sn, cs]]).T
            yaw = yaw - dpsi
        return box_footprint_points(c, sz, yaw, margin=0.2)

    def _process(self, src, t, road_xy, road_z, obs_xy, other_xy):
        obj = self.object_points(t)
        obstacles = np.concatenate([obs_xy, obj]) if obj.shape[0] else obs_xy
        tr = self.trackers[src]
        st = self.stats[src]
        m = measure(road_xy, road_z, obstacles, self.p, occlusion=self.steps["occlusion"],
                    extraction=self.steps["extraction"], fit=self.steps["fit"],
                    width_hint=tr.width, other_xy=other_xy if self.steps["occlusion"]
                    or self.steps["extraction"] else None)
        if self.steps["tracking"]:
            if tr.t is not None and t < tr.t - 1e-6:
                tr.reset()                   # bag rewind
            inc = ((0.0, np.zeros(2)) if tr.t is None or t <= tr.t
                   else self.twist.increment(tr.t, t))
            if not tr.predict(t, inc):
                st["no_odom"] += 1
            tr.update(m)
            yl, yr, z = tr.output()
        else:
            yl, yr, z = m.left, m.right, m.z
        out = []
        for y in (yl, yr):
            pts = resample(m.x, y, z, self.p.output_step)
            if pts.shape[0]:
                pts[:, :2] = self._to_lidar(pts[:, :2])
            else:
                st["empty_sides"] += 1
            out.append(pts)
        st["frames"] += 1
        st["bins"] += 2 * m.x.size
        st["occluded"] += m.stats["occluded_l"] + m.stats["occluded_r"]
        st["unsupported"] += m.stats["unsupported_l"] + m.stats["unsupported_r"]
        return out[0], out[1], m, (yl, yr, z)
