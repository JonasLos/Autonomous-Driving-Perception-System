"""Constant-velocity EKF for road objects: prediction, measurement models, updates.

No rclpy, no message types -- plain numpy, so the offline A/B harness and the unit tests
exercise this exact code rather than a copy that drifts. That is the same split
``perception_common.stamp_sync`` and ``radar_ros.radar_geometry`` already keep.

STATE  ``x = [px, py, vx, vy]``, positions in the sensor frame at the CURRENT epoch,
velocities GROUND-REFERENCED and expressed in the same axes.

Why CV, and not CA or CTRV:

* CA is out because acceleration is not observable here. Estimating it from second
  differences at 10 Hz gives sigma ~ sigma_p*sqrt(6)/dt^2, which at sigma_p = 0.5 m and
  dt = 0.1 s is ~120 m/s^2. The states would be pure noise amplifiers.
* CTRV is out because its extra value is turn rate, which needs observable HEADING, and
  heading is not observable at the ranges that matter: the LiDAR shape fit dies past ~40 m
  (VLP-32C ring pitch 0.333 deg puts only ~3 rings on a 1.5 m body at 80 m) and radar azimuth
  at 0.5-1.0 deg is a whole lane-width of lateral error at 80 m. A CTRV filter would estimate
  turn rate from noise. Heading comes from the velocity direction instead, which past 40 m is
  strictly better than any point fit. Not choosing CTRV and not choosing a UKF is ONE
  decision, not two -- CTRV's Jacobian is where EKF linearisation starts to hurt.

Ground-referenced velocity, not relative, for three reasons: the output should say "this car
is doing 15 m/s", which is what a planner wants; CV process noise is physical for a real
vehicle's acceleration but not for a relative velocity that steps whenever ego brakes; and
the radar range-rate model then carries the explicit ego term, which is the term already
verified against a bag.

Extent, yaw and z are NOT in the kinematic state. They are not dynamically coupled to planar
motion, their measurement noise is wildly non-Gaussian and one-sided, and coupling them lets
a bad shape fit move the position. They get their own estimators in
:mod:`object_fusion.detection_geometry`.
"""

from __future__ import annotations

import math

import numpy as np

__all__ = [
    "RADAR_SIGMA_RANGE", "RADAR_SIGMA_RANGE_RATE", "RADAR_SIGMA_AZIMUTH_DEG",
    "CAMERA_GATE_CHI2", "RADAR_GATE_CHI2", "MAX_CONSECUTIVE_REJECTS",
    "FORCED_UPDATE_P_INFLATION", "gated_update",
    "RANGE_TRUST_MAX_M", "SIGMA_ALONG_TABLE",
    "sigma_along", "sigma_cross", "range_is_trustworthy",
    "process_noise", "predict", "kalman_update", "wrap_deg",
    "radar_h_and_H", "lidar_measurement", "init_from_radar",
    "compensated_range_rate", "camera_radar_range_cap",
]

# --------------------------------------------------------------------------- radar noise
#: Datasheet, and consistent with everything measured here. The ESR resolves range to ~0.1 m
#: out to 175 m -- it is the best range reference on the vehicle, which is what makes it
#: usable as the ruler that measured SIGMA_ALONG_TABLE below.
RADAR_SIGMA_RANGE = 0.1

#: Bounded by measurement, not invented: ``radar_ab.py --check-conventions`` put the
#: static-target range-rate residual against -v_ego*cos(az) at median -0.06 m/s, p90
#: |residual| 0.45 m/s over 16219 observations. 0.2 is a conservative read of that, and it is
#: an UPPER bound because the residual also contains ego-velocity error.
RADAR_SIGMA_RANGE_RATE = 0.2

#: Datasheet azimuth is ~0.5 deg; inflated to 1.0 for multipath and extended-target
#: scattering. THE INFLATION IS INVENTED. It matters little when LiDAR is present (1.0 deg is
#: 1.4 m of lateral at 80 m against the LiDAR's ~1.1 m) and carries the whole lateral estimate
#: when it is not, which is the behaviour wanted in both regimes and falls out of the
#: covariance without a special case.
RADAR_SIGMA_AZIMUTH_DEG = 1.0

# ------------------------------------------------------------------- camera+LiDAR range
#: Beyond this the fused range is not noisy, it is WRONG, and no covariance expresses a bias.
#: Measured on selfcal_loc_2026-09-08_11-47-43 (1701 azimuth-gated pairs): median error
#: -7.59 m in the 80-100 m band and -32.10 m in 100-175 m, against about -1.1 m below 80 m.
#: Above this bound the along-ray component is DROPPED rather than inflated -- see
#: :func:`lidar_measurement`.
RANGE_TRUST_MAX_M = 80.0

#: Measured robust spread of (fused range - radar range), by radar-range band, from that same
#: replay. ``(range_m, sigma_m, n_pairs)``.
#:
#: This is deliberately a TABLE, not a closed form. Neither candidate form fits: a linear
#: sqrt(a^2+(br)^2) fitted over the trusted band predicts 1.46-2.62 m where 0.59-1.18 m was
#: measured, and a flat value misses the rise past 55 m. The all-bins fit of ~0.074*r quoted
#: in the Phase 0 report is dominated by the far bins, where this model does not apply at all.
#: Interpolating the measurement is more honest than forcing a wrong curve through it.
#:
#: CAVEAT, and it matters for anyone tightening these: this spread is a MIXTURE width, not
#: clean sensor noise. Each band blends well-placed objects with road-adopted ones, and the
#: contaminated fraction grows with range -- which is why the 15 m bin (1.32) reads wider than
#: the 25 and 35 m bins (0.59, 0.66). Treating it as Gaussian is already an approximation; it
#: is a defensible one only because the gross failures are excluded by RANGE_TRUST_MAX_M.
#: RE-MEASURED 2026-09-15 for the rule the detector runs TODAY (Patchwork++ ground flags,
#: percentile cut, nearest depth cluster, empty-box drop, DepthJumpGate), from
#: `scripts/neighbour_ab.py --dump` + the same azimuth-only radar matching. The old table below
#: came from the superseded /fused_bbox rule and was up to 1.5x too wide at 65-80 m, which made
#: the filter under-trust its input (camera NIS median 0.13 against a target of 2).
#:
#:      band      old sigma    new sigma (n)
#:      15-25 m   1.32 / 0.59  1.27 (258)
#:      25-35 m   0.66         0.64 (300)
#:      35-45 m   1.18         1.09 (308)
#:      45-55 m   1.40         1.30 (358)
#:      55-65 m   (1.40)       1.53 (260)
#:      65-80 m   3.75         1.85 (286)
#:      80-100 m  8.34         6.82 (146)
#:
#: Matching also fixed (to-do item 3): a radar return counts as the same object when it lies
#: inside the object's own ANGULAR EXTENT (5.6 deg at 10 m, 1.15 deg at 80 m -- a fixed +-1 deg
#: cannot match a car that subtends 10 deg) and within a deliberately wide range window
#: (3 m + 50% of range), which excludes matches that cannot be the same object at all: a 6.7 m
#: truck was being scored against a 69.9 m return on the same bearing.
#:
#: The 5-15 m band still reads -3.95 m median (sd 4.94, n=76) and is NOT in the table. Those
#: objects sit at 15-26 deg azimuth -- the edge of the camera's field of view, where radar
#: coverage is sparse -- so radar is not a usable ruler there. The 20 m entry is carried down.
SIGMA_ALONG_TABLE = (
    (20.0, 1.27, 258), (30.0, 0.64, 300), (40.0, 1.09, 308), (50.0, 1.30, 358),
    (60.0, 1.53, 260), (72.0, 1.85, 286), (90.0, 6.82, 146), (150.0, 20.21, 17),
)

#: Floor, so a near-field object never claims better than decimetre knowledge of its own
#: surface. INVENTED.
SIGMA_ALONG_FLOOR = 0.35

#: Chi-square gate on the camera+LiDAR update (2 dof, 99%). MEASURED, not invented, and the
#: reason it exists rather than a larger sigma is worth stating.
#:
#: NIS over 2312 camera updates on the selfcal replay, against the target of mean ~2 (the
#: measurement dimension) with ~95% under the chi-square 95th percentile:
#:
#:     sigma x1, no gate     mean 11.158  median 1.538  under-gate 71.2%   overconfident
#:     sigma x4, no gate     mean  1.868  median 0.239  under-gate 91.2%   mean fixed, core ruined
#:     sigma x1, gate 9.21   mean  1.544  median 0.533  under-gate 92.8%   36% rejected
#:
#: No single Gaussian sigma satisfies both the mean and the under-gate fraction, because the
#: along-ray error is a MIXTURE (clean returns plus road-adopted ones), not a Gaussian.
#: Inflating to swallow the tail under-weights the ~64% of updates that are fine -- at x4 the
#: median NIS collapses to 0.24 against a target of 1.39. Gating rejects the outlier
#: population instead and leaves the good measurements at full weight.
#:
#: WHAT THE REJECTED POPULATION ACTUALLY IS -- corrected after measuring it by range band:
#:
#:     0-20 m   40.0% rejected      60-80 m    5.0%
#:     20-40 m  25.3%               80-100 m   3.3%
#:     40-60 m  15.4%               all       16.8%
#:
#: Rejection is concentrated in the NEAR field, not the far field. An earlier note here
#: claimed these were the road-adoption population; that population lives past 80 m, where
#: rejection is 3.3%. The mechanism is the opposite: past RANGE_TRUST_MAX_M the measurement
#: collapses to a 1-D lateral one with a wide sigma_cross and passes easily, while close in it
#: is 2-D with sigma_along at its tightest (0.59-1.32 m), so ordinary extent and centroid
#: variation trips the gate.
#:
#: The honest reading is that sigma_along is too TIGHT in the near field, not that 40% of
#: near-field measurements are outliers. Re-derive it there before tightening anything else.
CAMERA_GATE_CHI2 = 9.21

#: A GATE WITH NO ESCAPE LOCKS TRACKS OUT, and that is not hypothetical here. Measured on the
#: reference replay: 719 rejections fell into only 181 runs -- median run 2, p90 9, and one
#: track rejected 67 CONSECUTIVE measurements. Runs of 10 or more were 9.9% of runs but 47.1%
#: of all rejections.
#:
#: The mechanism is self-reinforcing: a rejected measurement does not update the track, so the
#: state drifts further from the truth, so the next innovation is larger, so it is rejected
#: too. A locked-out track then coasts on prediction alone while its own measurements are
#: thrown away -- which is why the filtered state scored WORSE than the raw measurement
#: against radar range (median 4.01 m vs 3.12 m) before this escape existed.
#:
#: So the gate is kept -- the innovation really is heavy-tailed, empirical NIS quantiles run
#: 4-12x chi-square(2) at q90-q99 while the bulk sits BELOW it -- but it is given an escape.
MAX_CONSECUTIVE_REJECTS = 3

#: How much to inflate the position covariance before a forced update. A track that has
#: rejected several measurements in a row is confidently wrong: without inflation the Kalman
#: gain is tiny and one forced update barely moves it, so it would simply re-lock.
FORCED_UPDATE_P_INFLATION = 25.0


def gated_update(x, P, y, H, R, *, gate_chi2, consecutive_rejects,
                 max_consecutive=MAX_CONSECUTIVE_REJECTS,
                 inflation=FORCED_UPDATE_P_INFLATION):
    """Innovation gate with an escape. Returns ``(x, P, nis, applied, forced)``.

    Below ``max_consecutive`` this is an ordinary gated update. At or above it the gate is
    bypassed, the position covariance is inflated first so the measurement can actually pull
    the state, and the update is applied regardless. See MAX_CONSECUTIVE_REJECTS for why.
    """
    if consecutive_rejects >= max_consecutive:
        P = np.array(P, dtype=np.float64, copy=True)
        P[:2, :2] *= float(inflation)
        x, P, nis, _ = kalman_update(x, P, y, H, R)
        return x, P, nis, True, True
    x, P, nis, applied = kalman_update(x, P, y, H, R, gate_chi2=gate_chi2)
    return x, P, nis, applied, False


#: Radar needed no such treatment. Same replay, 39349 updates: mean NIS 1.587 with the
#: under-gate fraction between 94.5% and 96.9% in EVERY range band from 0 to 180 m. Slightly
#: conservative (mean below the dimension of 3), which is the safe side. Left alone.
RADAR_GATE_CHI2 = 16.266

#: Cross-ray spread of a camera+LiDAR object. BRACKETED BY MEASUREMENT 2026-09-15 (it was
#: invented at 0.30 + 0.010 r, which turned out to be wider than even the upper bound):
#:
#:   * upper bound, radar azimuth as the ruler (`scratchpad cross_sigma.py` over the
#:     neighbour_ab dump): the lateral spread of matched pairs is 0.18 m at 15-25 m rising to
#:     0.78 m at 80-100 m -- and at EVERY band that is what the ESR's own ~0.5 deg azimuth
#:     contributes on its own (0.17 -> 0.79 m). Deconvolved, the object's share is ~0: the
#:     camera+LiDAR lateral error is smaller than radar can resolve, so this only bounds it.
#:   * lower estimate, the track's own lateral jitter (second difference, var = 6 sigma^2, so
#:     immune to slow bias): 0.045 m at 15-25 m, 0.052 at 25-35, 0.061 at 45-55, 0.14 at 65-80.
#:
#: The model sits between them: the jitter is white noise only, while the bound also carries
#: slowly varying error (extrinsic, boresight, which face of the object is visible). Still the
#: least-supported number in the camera model -- it cannot be measured properly without a ruler
#: better than the radar's azimuth.
SIGMA_CROSS_BASE = 0.10
SIGMA_CROSS_PER_M = 0.004


def sigma_along(range_m: float) -> float:
    """Along-ray sigma for a camera+LiDAR object, interpolated from the measured table."""
    r = float(range_m)
    xs = [e[0] for e in SIGMA_ALONG_TABLE]
    ys = [e[1] for e in SIGMA_ALONG_TABLE]
    return max(SIGMA_ALONG_FLOOR, float(np.interp(r, xs, ys)))


def sigma_cross(range_m: float) -> float:
    """Cross-ray sigma for a camera+LiDAR object. Invented; see SIGMA_CROSS_BASE."""
    return SIGMA_CROSS_BASE + SIGMA_CROSS_PER_M * max(0.0, float(range_m))


def range_is_trustworthy(range_m: float, limit: float = RANGE_TRUST_MAX_M) -> bool:
    """Whether the fused along-ray component may be used at all at this range."""
    return float(range_m) <= float(limit)


def wrap_deg(d):
    """Wrap an angle difference in degrees to (-180, 180]."""
    return (np.asarray(d, dtype=np.float64) + 180.0) % 360.0 - 180.0


# ------------------------------------------------------------------------------ dynamics
def process_noise(dt, sigma_long, sigma_lat, heading=None):
    """Piecewise-constant white-acceleration Q, anisotropic in the track's own axes.

    ``heading`` rotates the acceleration covariance into the track's along/cross axes, so a
    vehicle is allowed to brake hard and change lanes gently rather than both equally. Pass
    ``None`` for an isotropic Q, which is what an unclassified or slow track gets -- a heading
    taken from a near-zero velocity is noise.

    sigma_long 2.0 and sigma_lat 1.0 m/s^2 (covering ~0.2 g braking and a 3 s lane change) started
    INVENTED and are now MEASURED -- see below, and HANDOFF item 2: they are what keeps lag near
    zero, and also why the camera NIS reads low (the track's P dominates the innovation).

    MEASURED, and the result is not what this model hoped for. Sweeping Q against radar range
    on the reference replay, |range error| improves MONOTONICALLY as Q grows:

        sigma_long  0.25  0.50  1.00  2.00  4.00   |  raw measurement
        median [m]  3.72  3.68  3.67  3.52  3.03   |  3.12
        mean   [m] 12.00 12.04 11.70 11.23 10.72   |  9.87
        p90    [m] 32.41 32.42 32.36 31.93 30.25   |  21.31

    Larger Q means trusting the measurement more and the motion model less, so a filter that
    keeps improving as Q grows is a filter whose MOTION MODEL IS NOT EARNING ITS KEEP -- in the
    limit it just becomes the raw measurement. Only the median crosses raw, and only at
    sigma_long 4.0; the mean and p90 never do.

    What the filter DOES buy is stability against large excursions: frame-to-frame jumps over
    2 m fall from 15.3% (raw) to 11.7-12.4% (filtered) at every Q tried. So the trade is
    "slightly worse typical range accuracy, meaningfully fewer big jumps", and whether that is
    worth having depends on the consumer. It is NOT the range-accuracy win this design assumed.

    SUPERSEDED: that verdict was measured on the old /fused_bbox stream with radar updates off.
    On the current measurements, radar held out, the filter wins (median 1.57 -> 1.21 m, jumps
    4.4% -> 2.9%), and publish_mode defaults to filtered since 2026-09-17.
    """
    dt = float(dt)
    Sa = np.diag([float(sigma_long) ** 2, float(sigma_lat) ** 2])
    if heading is not None:
        c, s = math.cos(heading), math.sin(heading)
        Rh = np.array([[c, -s], [s, c]])
        Sa = Rh @ Sa @ Rh.T
    return np.block([[Sa * dt ** 4 / 4.0, Sa * dt ** 3 / 2.0],
                     [Sa * dt ** 3 / 2.0, Sa * dt ** 2]])


def predict(x, P, dt, dpsi, d_xy, Q):
    """One CV step plus the ego frame change. Returns ``(x, P)``.

    ``dpsi`` and ``d_xy`` describe how the SENSOR frame moved over ``dt``, both expressed in
    the frame as it was at the start of the step -- exactly what
    :func:`object_fusion.ego_motion.frame_increment` returns.

    A point fixed in the world moves in frame coordinates as ``p' = R(dpsi)^T (p - d)``, and a
    ground-referenced velocity rotates only. Combined with constant velocity:

        p- = R(dpsi)^T (p + v*dt - d)
        v- = R(dpsi)^T v

    which is LINEAR in the state, so this step is exact and the only nonlinearity in the whole
    filter is the radar measurement.

    Approximation and its size: the object's displacement is integrated in the start-of-step
    axes while the axes rotate during the step. The neglected term is ~0.5*omega*dt*v*dt --
    1.5 cm at omega = 0.2 rad/s, dt = 0.1 s, v = 15 m/s. Do not sub-step for it.
    """
    x = np.asarray(x, dtype=np.float64).reshape(4)
    P = np.asarray(P, dtype=np.float64).reshape(4, 4)
    dt = float(dt)
    c, s = math.cos(dpsi), math.sin(dpsi)
    Rt = np.array([[c, s], [-s, c]])            # R(dpsi)^T : old frame -> new frame
    A = np.zeros((4, 4))
    A[:2, :2] = Rt
    A[:2, 2:] = Rt * dt
    A[2:, 2:] = Rt
    b = np.zeros(4)
    b[:2] = -Rt @ np.asarray(d_xy, dtype=np.float64).reshape(2)
    x_out = A @ x + b
    P_out = A @ P @ A.T + np.asarray(Q, dtype=np.float64)
    return x_out, 0.5 * (P_out + P_out.T)


def kalman_update(x, P, y, H, R, gate_chi2=None):
    """Joseph-form update. Returns ``(x, P, nis, applied)``.

    ``y`` is the innovation ``z - h(x)``, already angle-wrapped by the caller where relevant.

    Joseph form is not stylistic here. Radar at ~30 Hz plus camera at 10 Hz is thousands of
    small sequential updates over a track's life, and the naive ``(I - KH)P`` loses symmetry
    and positive-definiteness over that many.

    ``gate_chi2`` rejects rather than clamps: a measurement outside the gate is not applied at
    all and ``applied`` comes back False, so the caller can count it. Clamping a wild
    measurement into the gate would let a persistent bias walk the state.
    """
    x = np.asarray(x, dtype=np.float64).reshape(-1)
    P = np.asarray(P, dtype=np.float64)
    y = np.atleast_1d(np.asarray(y, dtype=np.float64))
    H = np.atleast_2d(np.asarray(H, dtype=np.float64))
    R = np.atleast_2d(np.asarray(R, dtype=np.float64))

    S = H @ P @ H.T + R
    Sinv = np.linalg.inv(S)
    nis = float(y @ Sinv @ y)
    if gate_chi2 is not None and nis > float(gate_chi2):
        return x, P, nis, False
    K = P @ H.T @ Sinv
    x_out = x + K @ y
    I_KH = np.eye(x.size) - K @ H
    P_out = I_KH @ P @ I_KH.T + K @ R @ K.T
    return x_out, 0.5 * (P_out + P_out.T), nis, True


# --------------------------------------------------------------------- measurement models
def radar_h_and_H(x, R_sl, t_sl, v_ego_s):
    """Radar measurement ``h(x) = [range, range_rate, azimuth_deg]`` and its Jacobian.

    The state is Cartesian in the LiDAR frame (L); the radar measures polar in its own frame
    (S). ``R_sl``/``t_sl`` map a point L -> S -- exactly what
    ``radar_fusion_node._lookup(radar_frame, lidar_frame)`` already returns. ``v_ego_s`` is the
    velocity of the RADAR ORIGIN (lever arm included) expressed in S.

    This is deliberately NOT a Cartesian position measurement. Converting (rho, az) to (x, y)
    smears the radar's excellent 0.1 m range into the lateral direction through the
    conversion's cross-covariance, and lets its 1.4 m azimuth error at 80 m corrupt lateral
    state the LiDAR already knows to ~1.1 m. Measuring in the sensor's native space is the
    same argument ``radar_geometry.associate`` already makes for gating, applied to the update.

    SIGN SELF-CHECK, and it is not optional. For a static target (v_S = 0) with the ego at
    speed v and lever arm L, this reduces to

        rho_dot = -v*cos(a) - omega*L*sin(a)

    reproducing BOTH facts verified on the 2026-08-25 bag: the -v_ego*cos(az) term (98.6%
    agreement over 16219 observations) and the odd-in-azimuth lever-arm term that settled the
    azimuth sign. An implementation that disagrees with this expression has a sign bug.

    range_rate is POSITIVE = RECEDING, matching EsrTrack.msg.
    """
    x = np.asarray(x, dtype=np.float64).reshape(4)
    R_sl = np.asarray(R_sl, dtype=np.float64).reshape(2, 2)
    t_sl = np.asarray(t_sl, dtype=np.float64).reshape(2)
    v_ego_s = np.asarray(v_ego_s, dtype=np.float64).reshape(2)

    p_l, v_l = x[:2], x[2:]
    p_s = R_sl @ p_l + t_sl
    v_s = R_sl @ v_l
    rho = float(math.hypot(p_s[0], p_s[1]))
    if rho < 1e-6:
        rho = 1e-6
    u = p_s / rho
    n = np.array([-u[1], u[0]])
    v_rel = v_s - v_ego_s

    z = np.array([rho, float(u @ v_rel), math.degrees(math.atan2(p_s[1], p_s[0]))])

    P_perp = (np.eye(2) - np.outer(u, u)) / rho          # d(u)/d(p_s)
    H = np.zeros((3, 4))
    H[0, :2] = u @ R_sl
    H[1, :2] = (v_rel @ P_perp) @ R_sl
    H[1, 2:] = u @ R_sl
    H[2, :2] = (math.degrees(1.0) * n / rho) @ R_sl
    return z, H


def radar_R(sigma_range=RADAR_SIGMA_RANGE, sigma_rate=RADAR_SIGMA_RANGE_RATE,
            sigma_az_deg=RADAR_SIGMA_AZIMUTH_DEG):
    """Measurement covariance for :func:`radar_h_and_H`. Genuinely diagonal in polar."""
    return np.diag([sigma_range ** 2, sigma_rate ** 2, sigma_az_deg ** 2])


def lidar_measurement(p_pred_l, z_xy, *, sigma_a=None, sigma_c=None, drop_range=False):
    """Camera+LiDAR position measurement. Returns ``(y, H, R)`` for :func:`kalman_update`.

    When ``drop_range`` is set the measurement collapses to its CROSS-RAY component only: a
    1-D lateral observation, with range left entirely to the radar.

    That is a drop, not an inflation, and the distinction is the point. The far-field failure
    is a BIAS -- median -7.59 m at 80-100 m, -32.10 m at 100-175 m -- and a Kalman filter has
    no defence against bias. A 14 m offset sustained over several frames is partly absorbed
    even at 10x inflated R, and the filter then reads the entry and exit as velocity spikes,
    which is worse than the original static error because a planner reacts to velocity.

    A per-frame node cannot make this decision; it is available only because the state is
    filtered. Where radar is ABSENT this failure is unmitigated, and the honest response is to
    keep sigma_along large and let the position stay uncertain rather than wrong-and-confident.
    """
    p = np.asarray(p_pred_l, dtype=np.float64).reshape(2)
    z = np.asarray(z_xy, dtype=np.float64).reshape(2)
    r = float(np.linalg.norm(p))
    if r < 1e-6:
        u = np.array([1.0, 0.0])
    else:
        u = p / r
    n = np.array([-u[1], u[0]])
    sa = sigma_along(r) if sigma_a is None else float(sigma_a)
    sc = sigma_cross(r) if sigma_c is None else float(sigma_c)

    if drop_range:
        H = np.zeros((1, 4))
        H[0, :2] = n
        return np.array([float(n @ (z - p))]), H, np.array([[sc ** 2]])

    B = np.column_stack((u, n))
    R = B @ np.diag([sa ** 2, sc ** 2]) @ B.T
    H = np.zeros((2, 4))
    H[:2, :2] = np.eye(2)
    return z - p, H, R


def init_from_radar(rho, az_deg, range_rate, v_ego_s, R_ls, t_ls,
                    sigma_rho=RADAR_SIGMA_RANGE, sigma_az_deg=RADAR_SIGMA_AZIMUTH_DEG,
                    sigma_rate=RADAR_SIGMA_RANGE_RATE, sigma_v_cross=20.0):
    """Birth a track from one radar detection. Returns ``(x, P)`` in the LiDAR frame.

    ``R_ls``/``t_ls`` map a point S -> L (the inverse direction from
    :func:`radar_h_and_H`'s arguments).

    Two things this does that a naive birth does not:

    * The RADIAL velocity component is initialised from the range rate. That is the whole
      point of track-level radar fusion -- a camera birth knows nothing about velocity, while
      the radar has already measured its most useful component.
    * The covariance is built IN POLAR and mapped through, giving the correct banana shape
      (thin in range, wide in azimuth) rather than an isotropic blob. This matters for the
      very first association, and it is also what keeps the EKF linearisation honest at birth:
      the worst second-order term in the filter is the range-rate one, ~sigma_p*sigma_v/rho,
      which at birth with an isotropic 20 m/s blob at 80 m would be ~0.25 m/s.

    ``sigma_v_cross`` = 20 m/s is INVENTED and deliberately wide -- it covers a target closing
    at 30 m/s against a 15 m/s ego. A tight first-frame velocity prior is what makes new
    tracks lag.
    """
    a = math.radians(float(az_deg))
    u = np.array([math.cos(a), math.sin(a)])
    n = np.array([-u[1], u[0]])
    B = np.column_stack((u, n))

    p_s = float(rho) * u
    # rr = u . (v_target - v_ego)  =>  u . v_target = rr + u . v_ego
    v_s = (float(range_rate) + float(np.asarray(v_ego_s).reshape(2) @ u)) * u

    C_p_s = B @ np.diag([sigma_rho ** 2, (float(rho) * math.radians(sigma_az_deg)) ** 2]) @ B.T
    C_v_s = B @ np.diag([sigma_rate ** 2, float(sigma_v_cross) ** 2]) @ B.T

    R_ls = np.asarray(R_ls, dtype=np.float64).reshape(2, 2)
    t_ls = np.asarray(t_ls, dtype=np.float64).reshape(2)
    x = np.concatenate([R_ls @ p_s + t_ls, R_ls @ v_s])
    P = np.zeros((4, 4))
    P[:2, :2] = R_ls @ C_p_s @ R_ls.T
    P[2:, 2:] = R_ls @ C_v_s @ R_ls.T
    return x, 0.5 * (P + P.T)


def camera_radar_range_cap(camera_range_m, k=3.0, floor_m=3.0):
    """How far a radar return may sit from the SAME track's recent camera range, or None.

    k * sigma_along(r), floored at 3 m, inside RANGE_TRUST_MAX_M; None beyond, where the camera
    range is biased and radar must stay free to own it.

    Why against the CAMERA and not the track: at 40-80 m, 38.9% of radar updates on vehicle tracks
    put the object more than 3 m behind its camera range, and the LiDAR on that bearing
    (scripts/radar_vehicle_truth.py) shows the vehicle at the CAMERA range with nothing at the
    radar range in 83% of them -- a multipath ghost -- and a different object behind it in 12.6%.
    A cap against the track's own predicted range does nothing (tried: 29239 -> 29227 updates),
    because once radar has pulled the track back, the next ghost agrees with the track.
    """
    r = float(camera_range_m)
    if r >= RANGE_TRUST_MAX_M:
        return None
    return max(float(floor_m), float(k) * float(sigma_along(r)))


def compensated_range_rate(range_rate, azimuth_deg, v_ego_s):
    """Range rate with the ego contribution removed: ~0 for a target stationary over ground.

    Follows directly from the verified convention ``rr = u . (v_target - v_ego)``: a target
    not moving over the ground has ``u . v_target = 0``, so ``rr + u . v_ego = 0``.

    This is the discriminant the radar-only birth gate rests on. Guardrails, manhole covers
    and overhead signs -- every member of the dominant clutter class -- are stationary, and
    nothing else separates them: the ESR has no elevation channel, amplitude bottoms out at
    -10 with ~30% of real tracks sitting on the floor, and update_count is inert on this
    driver.
    """
    a = np.radians(np.asarray(azimuth_deg, dtype=np.float64))
    u = np.stack([np.cos(a), np.sin(a)], axis=-1)
    return np.asarray(range_rate, dtype=np.float64) + u @ np.asarray(v_ego_s, dtype=np.float64)
