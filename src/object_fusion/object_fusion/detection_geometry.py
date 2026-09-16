"""Turning the LiDAR returns inside a 2D box into an extent, a heading and a covariance.

ROS-free. What the shipped ``fusion_node`` does today is take the component-wise MEDIAN of
the surviving points as the object centre, a hardcoded 1.5 m cube as the size, and identity
as the orientation. This module is the replacement for all three, and it runs in the NEW
detector node -- ``fusion_node`` itself is not touched, and its rules cannot be imported from
here anyway because ``yolo_ros`` is built only into the YOLO image while this package runs in
the CPU-only transform image.

WHAT IS RECOVERABLE, from this vehicle's measured ring geometry. The VLP-32C's pitch is
0.333 deg across rings 9-25, which is the band that images anything past 30 m, and the
azimuthal sample spacing is ~0.0035*r:

    range        20 m        80 m
    ring spacing 0.12 m      0.47 m
    rings on a 1.5 m body    ~12        ~3
    columns across a 1.8 m car  ~25     ~6
    width        +/-0.2 m, real        +/-0.3 m, still real
    length       +/-0.3 m oblique, unobservable head-on   NOT recoverable
    yaw from shape  +/-5-10 deg, usable                   NOT recoverable

So the fit is only attempted below :data:`SHAPE_FIT_MAX_RANGE_M`, and that bound is DERIVED,
not invented: 40 m is where 0.23 m ring spacing still puts ~6 rings on a 1.5 m body. Past it
the cloud is a flat plate and a rectangle fit returns a confident arbitrary angle.

THE CENTROID TRAP, which is the subtlest thing here. Both the LiDAR and the radar report a
VISIBLE SURFACE -- the LiDAR the visible face, the radar the bumper/plate scattering centre.
Measured on selfcal_loc_2026-09-08_11-47-43, fused range sits a near-constant -1.1 m from
radar range out to 80 m, and that number is the LiDAR-versus-radar scattering-centre
difference, NOT the surface-to-centroid offset. Correcting the LiDAR to a centroid while
leaving the radar alone would build a permanent inconsistency between the two measurement
models; at 30 Hz against 10 Hz the radar wins, and the published position converges to
whatever the radar's surface is -- confidently. So every measurement model carries its own
offset function and the state stays at the centroid.
"""

from __future__ import annotations

import math

import numpy as np

__all__ = [
    "reject_ground_returns", "nearest_depth_cluster", "select_object_points",
    "SHAPE_FIT_MAX_RANGE_M", "SHAPE_FIT_MIN_POINTS", "YAW_FROM_SHAPE", "YAW_FROM_VELOCITY",
    "YAW_FROM_RAY", "fit_rectangle", "choose_yaw", "surface_to_centroid_offset",
    "ExtentFilter", "camera_only_range", "camera_only_range_variance",
]

#: Derived from the measured ring pitch; see the module docstring.
SHAPE_FIT_MAX_RANGE_M = 40.0

#: Below this the rectangle is under-determined. INVENTED (start 12); sweep it offline.
SHAPE_FIT_MIN_POINTS = 12

#: Speed above which the velocity heading beats any point fit. INVENTED.
YAW_FROM_VELOCITY_MIN_SPEED = 2.0

YAW_FROM_SHAPE = "shape_fit"
YAW_FROM_VELOCITY = "velocity"
YAW_FROM_RAY = "ray_default"


def fit_rectangle(px, py, *, step_deg=2.0, min_points=SHAPE_FIT_MIN_POINTS):
    """Search-based rotated-rectangle fit over one box's points. ``None`` if under-determined.

    Returns ``(yaw_rad, length, width, cx, cy, quality)``. ``yaw_rad`` is the direction of the
    longer side, in [0, pi).

    Vectorised over candidate angles -- 45 angles x N points as one array. A per-point Python
    loop is ~100x worse and would surface as 10 Hz jitter; that is the whole reason for the
    einsum-free formulation below.

    Only the first 90 degrees is searched: a rectangle has 4-fold symmetry, so the residual
    orientation ambiguity is resolved downstream from the velocity direction rather than here.

    ``quality`` is the Zhang-style closeness score normalised by point count -- a good fit
    puts most points near one of the four edges. It is comparable between fits of different
    sizes, which is what a threshold needs.
    """
    px = np.asarray(px, dtype=np.float64).ravel()
    py = np.asarray(py, dtype=np.float64).ravel()
    n = px.size
    if n < int(min_points):
        return None

    th = np.radians(np.arange(0.0, 90.0, float(step_deg)))
    c = np.cos(th)[:, None]
    s = np.sin(th)[:, None]
    a = c * px[None, :] + s * py[None, :]
    b = -s * px[None, :] + c * py[None, :]

    amin = a.min(1, keepdims=True); amax = a.max(1, keepdims=True)
    bmin = b.min(1, keepdims=True); bmax = b.max(1, keepdims=True)
    da = np.minimum(a - amin, amax - a)
    db = np.minimum(b - bmin, bmax - b)
    score = (1.0 / np.maximum(np.minimum(da, db), 0.05)).sum(1) / n

    k = int(np.argmax(score))
    ck, sk = float(c[k, 0]), float(s[k, 0])
    span_a = float(amax[k, 0] - amin[k, 0])
    span_b = float(bmax[k, 0] - bmin[k, 0])
    mid_a = 0.5 * float(amax[k, 0] + amin[k, 0])
    mid_b = 0.5 * float(bmax[k, 0] + bmin[k, 0])
    # Rotate the centre back out of the fitted axes.
    cx = ck * mid_a - sk * mid_b
    cy = sk * mid_a + ck * mid_b

    if span_a >= span_b:
        length, width, yaw = span_a, span_b, float(th[k])
    else:
        length, width, yaw = span_b, span_a, float(th[k]) + math.pi / 2.0
    return yaw % math.pi, length, width, cx, cy, float(score[k])


def choose_yaw(range_m, n_points, speed, velocity_heading, ray_heading, fit=None):
    """Pick a heading and say where it came from. Returns ``(yaw_rad, source)``.

    A cascade, because the consumer must be told when the yaw is fabricated:

    1. the shape fit, only close in and only with enough points;
    2. the track's own velocity direction, which past 40 m beats any point fit and is what
       fires most of the time on a highway;
    3. the sensor ray, i.e. "the object faces us" -- least wrong for a leading vehicle.
    """
    if (fit is not None and float(range_m) <= SHAPE_FIT_MAX_RANGE_M
            and int(n_points) >= SHAPE_FIT_MIN_POINTS):
        return float(fit[0]), YAW_FROM_SHAPE
    if float(speed) > YAW_FROM_VELOCITY_MIN_SPEED:
        return float(velocity_heading), YAW_FROM_VELOCITY
    return float(ray_heading), YAW_FROM_RAY


def surface_to_centroid_offset(length, width, ray_heading, object_heading):
    """Distance to push a visible-surface point back along the ray to reach the centroid.

    ``0.5 * (L*|cos phi| + W*|sin phi|)`` with ``phi`` the angle between the ray and the
    object's longitudinal axis -- the half-extent of the box in the viewing direction.

    Use this ONLY where a rectangle fit is unavailable. Where the fit succeeded its centre is
    already the centroid and no bias model is needed, which is strictly better.

    The caller must add this correction's own uncertainty (~0.5*sigma_L) into the along-ray
    R. Correcting the mean without inflating the covariance makes the filter over-trust a
    corrected position, which is a worse failure than the original bias because it looks
    converged.
    """
    phi = float(ray_heading) - float(object_heading)
    return 0.5 * (abs(float(length) * math.cos(phi)) + abs(float(width) * math.sin(phi)))


class ExtentFilter:
    """Running extent estimate: a decayed upper quantile, never a mean.

    Extent from a partial view is ONE-SIDED. Occlusion only ever makes an object look shorter,
    never longer, so the mean of a sequence of such measurements is biased low and never
    recovers. Taking the running maximum with a slow decay tracks the true extent from below
    and forgets a bad merge.

    ``decay`` 0.02 per update and the 1.5x prior clamp are INVENTED.
    """

    __slots__ = ("value", "prior", "decay", "clamp")

    def __init__(self, prior, decay=0.02, clamp=1.5):
        self.prior = float(prior)
        self.value = float(prior)
        self.decay = float(decay)
        self.clamp = float(clamp)

    def update(self, measured=None):
        self.value *= (1.0 - self.decay)
        if measured is not None:
            self.value = max(self.value, float(measured))
        # A bad merge must not inflate the extent without bound.
        self.value = min(self.value, self.prior * self.clamp)
        # ...and decay must not collapse it below something physical.
        self.value = max(self.value, 0.4 * self.prior)
        return self.value


def camera_only_range(v_bottom_px, fy, cy, camera_height_m, class_height_m=None,
                      box_height_px=None):
    """Range for a 2D box with NO LiDAR returns. ``None`` when neither estimate is available.

    WARNING -- NOT VALID FOR THIS VEHICLE'S CAMERA, and no longer used by the detector. The ground
    intercept assumes image row ``cy`` is the horizon (optical axis level) and ignores the camera
    extrinsic; camera_fl is pitched ~2.7 deg down, which put a 30 m cone at ~200 m. Use
    projection.camera_ground_position. Kept only for its covariance companion and tests.

    Today such a detection is dropped silently. Two independent estimates are available and
    the NEARER is taken, because under-ranging an obstacle is the safe direction:

    * the ground-plane intercept of the ray through the box's bottom edge, which is
      well-conditioned for anything standing on the road;
    * ``f * H_class / h_px`` from a class height prior, which survives when the bottom edge is
      occluded.

    Returns ``(range_m, source)``.
    """
    est = []
    if v_bottom_px is not None and fy and camera_height_m:
        dv = float(v_bottom_px) - float(cy)
        if dv > 1e-6:                      # must be below the horizon to intersect the road
            est.append((float(fy) * float(camera_height_m) / dv, "ground_intercept"))
    if class_height_m and box_height_px and float(box_height_px) > 1e-6:
        est.append((float(fy) * float(class_height_m) / float(box_height_px), "class_height"))
    if not est:
        return None
    return min(est, key=lambda e: e[0])


def camera_only_range_variance(range_m, fy, camera_height_m, sigma_px=2.0):
    """Variance of a ground-intercept range. Grows as r^4 -- that is the whole point.

    ``reject_ground``'s own measurement gives the sensitivity of the ground intercept to a
    pixel of box-edge error as ``r^2/(f*h)``: 0.01 m per pixel at 10 m, 0.15 m at 40 m, 0.60 m
    at 80 m. The standard deviation is therefore proportional to r^2, so the VARIANCE goes as
    r^4. Emitting these detections with an honest r^4 variance is exactly what stops them from
    dragging a well-observed track; emitting them with a flat variance would be worse than
    dropping them.

    ``sigma_px`` = 2 px is INVENTED.
    """
    r = float(range_m)
    denom = float(fy) * float(camera_height_m)
    if denom <= 0.0:
        return float("inf")
    sigma = (r * r / denom) * float(sigma_px)
    return sigma * sigma


# ------------------------------------------------------------------ point selection
# Re-implemented here rather than imported. The equivalent rules live in yolo_ros'
# fusion_node, which is built only into the YOLO image; this package runs in the CPU-only
# transform image, and adding yolo_ros there would drag in ultralytics and torch. The offline
# harness can reach the originals by file path (importlib) to compare; a node cannot.

def reject_ground_returns(px, py, pz, *, min_range=25.0, margin=0.4, min_points=2):
    """Drop road returns from a box before the depth clustering runs.

    The road in front of a distant vehicle falls inside its 2D box and is NEARER than the
    vehicle, so a nearest-cluster rule adopts it and the object is published several metres
    early. The size of that error grows as r^2/(f*h) -- 0.01 m per pixel of box-edge error at
    10 m, 0.60 m at 80 m -- which is why the gate is on RANGE rather than on height alone:
    below min_range the error is under a decimetre, while a short object (a cone stands
    ~0.5 m) lies almost entirely within margin of the road and filtering there would strip it
    for no benefit.

    A low percentile rather than the minimum, so one stray low return cannot drag the estimate
    down and quietly disable the margin. A box left with fewer than min_points falls back to
    the unfiltered set, so a sparsely-sampled object is never placed worse than before.
    """
    px = np.asarray(px, dtype=np.float64).ravel()
    py = np.asarray(py, dtype=np.float64).ravel()
    pz = np.asarray(pz, dtype=np.float64).ravel()
    if px.size < 2 or margin <= 0.0:
        return px, py, pz
    # The NEAREST return decides, not the median: it is the one the clustering would adopt.
    if float(np.min(px)) < float(min_range):
        return px, py, pz
    ground_z = float(np.percentile(pz, 10.0))
    keep = pz > ground_z + float(margin)
    if int(np.count_nonzero(keep)) < int(min_points):
        return px, py, pz
    return px[keep], py[keep], pz[keep]


def nearest_depth_cluster(px, py, pz):
    """Keep only the nearest depth cluster inside a box.

    Two objects overlapping in image space drop two separated groups of returns into one box,
    and a median over both lands between them where nothing is. Sorting by depth and cutting
    at the first significant gap keeps the foreground object alone.

    Depth is ``px`` -- forward range in the LiDAR frame. Clustering on ``pz`` instead splits
    by HEIGHT, which on flat ground finds no gap at all and lets through the very blending
    this guards against. Do not "fix" the road problem by clustering on height; the road is
    genuinely nearer, so the nearest cluster is the correct answer to the question asked here,
    just not the wanted one. That is what reject_ground_returns is for, upstream.
    """
    px = np.asarray(px, dtype=np.float64).ravel()
    py = np.asarray(py, dtype=np.float64).ravel()
    pz = np.asarray(pz, dtype=np.float64).ravel()
    if px.size < 2:
        return px, py, pz
    order = np.argsort(px)
    sx = px[order]
    diffs = np.diff(sx)
    med = float(np.median(diffs))
    mad = float(np.median(np.abs(diffs - med)))
    gaps = np.where(diffs > max(med + 3.0 * mad, 0.05))[0]
    if gaps.size == 0:
        return px, py, pz
    fg = order[: gaps[0] + 1]
    return px[fg], py[fg], pz[fg]


def select_object_points(px, py, pz, ground=None, *, ground_min_range=10.0, margin=0.4,
                         min_points=2, empty_fallback=False):
    """Box points -> the points that belong to the object. Returns ``(px, py, pz, used_ground)``.

    The configuration adopted after scoring it against radar range on the reference replay
    (``scripts/ground_ab.py --backend patchworkpp --max-range 120``), all 4519 detections, with
    the empty-box fallback still ON (see stage 1 for why it is now off), versus fusion_node's rule:

        band      range error sd      jitter (2nd difference) median / p90
        15-25 m   2.24 -> 1.06        0.40 / 3.24 -> 0.12 / 1.21
        25-40 m   1.04 -> 0.69        0.25 / 4.33 -> 0.15 / 2.43
        40-60 m   2.30 -> 1.58        0.31 / 6.73 -> 0.17 / 2.16
        60-80 m   4.31 -> 2.76        0.30 / 2.64 -> 0.30 / 2.88

    Three stages, in order:

    1. Drop points a full-sweep ground segmentation flagged as ground (``ground``, from
       Patchwork++ in ground_projection_node). When that leaves the box EMPTY -- 3.7% of
       detections -- return nothing (``empty_fallback=False``, the default) or every point.

       The fallback was the original choice, so that no detection was ever lost. It was
       reversed after ``scripts/neighbour_ab.py``: 165 of the 169 emptied boxes are traffic
       cones the LiDAR missed that frame, and falling back placed them on whatever else was in
       the box -- usually a road ring several metres nearer or farther. Those frames were 3.5%
       of detections and 27% of all >2 m spikes; dropping them took the spike rate 8.2% ->
       5.2% and the 60-80 m range spread against radar 2.76 -> 1.91 m. A box with nothing but
       road in it has no LiDAR range to give. The caller may range it from the camera instead.
    2. The percentile ground cut, from ``ground_min_range`` (10 m; see the camera detector for
       that sweep). Kept on top of segmentation because the two together beat either alone.
    3. Nearest depth cluster.

    ``ground`` of ``None`` means no segmentation is available (an input topic without the flag),
    and the rule reduces exactly to the percentile cut plus the depth cluster.
    """
    px = np.asarray(px, dtype=np.float64).ravel()
    py = np.asarray(py, dtype=np.float64).ravel()
    pz = np.asarray(pz, dtype=np.float64).ravel()
    used = False
    if ground is not None:
        g = np.asarray(ground, dtype=bool).ravel()
        keep = ~g
        if np.any(keep):
            px, py, pz = px[keep], py[keep], pz[keep]
            used = True
        elif not empty_fallback:
            return px[:0], py[:0], pz[:0], True
    px, py, pz = reject_ground_returns(px, py, pz, min_range=ground_min_range,
                                       margin=margin, min_points=min_points)
    px, py, pz = nearest_depth_cluster(px, py, pz)
    return px, py, pz, used
