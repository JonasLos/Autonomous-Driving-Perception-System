"""Measurement-to-track association: native-space gating, then one global assignment.

Free of rclpy and of message types, like the rest of this package's core.

THE LESSON THIS MODULE EXISTS TO ENCODE. The shipped radar node gates association on RANGE
at +/-3 m. Measured on ``selfcal_loc_2026-09-08_11-47-43``, that gate does not merely
truncate the error distribution -- it preferentially keeps the objects whose range is already
correct and discards the ones that are 15-30 m wrong. In the 80-100 m band it keeps 78 of 146
real pairs and reports a median error of -0.29 m where the truth is -7.59 m; in 100-175 m it
keeps 9 of 57 and reports -1.04 m against -32.10 m. On one earlier bag it matched ZERO pairs.

So the node's own shadow statistics describe only the already-good subset, and this is
compounded by a second defect: ``range_disputed`` requires |disagreement| > 5.0 m while
association requires <= 3.0 m, which are mutually exclusive -- the flag documented as the
detector for exactly this failure can never fire at the shipped defaults.

**To detect a range error of magnitude X you must gate on AZIMUTH and open the range gate
well past X.** A 3 m range gate structurally cannot see a 14 m range error. That is why
:func:`associate_radar` takes a wide ``max_range_err`` by default and leans on azimuth, and
why the range residual is REPORTED rather than gated.

Gate widths were swept: from +/-0.7 deg to +/-2.0 deg the binned medians move by less than
0.2 m while the pair count grows 1475 -> 2124, so the result does not depend on the choice.
The loose end is contamination -- at +/-2.0 deg a symmetric +20..+33 m tail appears, which is
objects matched to unrelated tracks -- so 1.0 deg is the default.
"""

from __future__ import annotations

import numpy as np

from object_fusion.tracker import wrap_deg

__all__ = [
    "BLOCKED_COST", "solve_assignment", "mahalanobis_sq",
    "associate_radar", "apply_sticky_ids",
    "CHI2_2DOF_99", "CHI2_3DOF_99", "CHI2_3DOF_999",
]

#: A forbidden pair must cost more than any tour of allowed ones, or the optimiser will take
#: one to complete a larger assignment. Same convention and value as
#: ``radar_geometry.associate``, which this module deliberately mirrors rather than edits.
BLOCKED_COST = 1e6

CHI2_2DOF_99 = 9.210
CHI2_3DOF_99 = 11.345
#: The looser default. The LiDAR range residual has a heavy one-sided tail that is a BIAS
#: rather than noise, so a 99% gate rejects real measurements from real objects; the hard
#: outer caps below are what actually bound the association.
CHI2_3DOF_999 = 16.266


def solve_assignment(cost, allowed):
    """Global one-to-one assignment over ``cost`` restricted to ``allowed``.

    Returns ``[(row, col), ...]`` sorted. Never returns a pair outside ``allowed``, so an
    empty result is a legitimate outcome rather than a failure.

    Global, not greedy per row: at these counts the difference only shows up in genuinely
    ambiguous scenes, but "nearest for each row in turn" can leave a later row with nothing
    when a small swap would have satisfied both.

    This reimplements the optimiser core rather than importing it from
    ``radar_ros.radar_geometry.associate``, because that function bundles the optimiser with
    ESR-specific polar gating and refactoring it would mean editing a shipped module. The
    convention (blocked cost, scipy with a greedy fallback) is kept identical on purpose.
    """
    cost = np.atleast_2d(np.asarray(cost, dtype=np.float64))
    allowed = np.atleast_2d(np.asarray(allowed, dtype=bool))
    if cost.size == 0 or not allowed.any():
        return []
    c = np.where(allowed, cost, BLOCKED_COST)
    try:
        from scipy.optimize import linear_sum_assignment

        rows, cols = linear_sum_assignment(c)
        pairs = [(int(r), int(col)) for r, col in zip(rows, cols) if allowed[r, col]]
    except ImportError:
        # Greedy nearest-first. Acceptable fallback at these counts rather than a hard
        # failure, and it is flagged here so a silent downgrade is at least documented.
        pairs, used_r, used_c = [], set(), set()
        order = np.dstack(np.unravel_index(np.argsort(c, axis=None), c.shape))[0]
        for r, col in order:
            r, col = int(r), int(col)
            if not allowed[r, col]:
                break
            if r in used_r or col in used_c:
                continue
            used_r.add(r)
            used_c.add(col)
            pairs.append((r, col))
    return sorted(pairs)


def mahalanobis_sq(y, S):
    """``y^T S^-1 y`` for a single innovation, guarding a singular S."""
    y = np.atleast_1d(np.asarray(y, dtype=np.float64))
    S = np.atleast_2d(np.asarray(S, dtype=np.float64))
    try:
        return float(y @ np.linalg.solve(S, y))
    except np.linalg.LinAlgError:
        return float("inf")


def associate_radar(tracks, sweep, R_sl, t_sl, v_ego_s, *,
                    chi2_gate=CHI2_3DOF_999,
                    max_azimuth_err_deg=1.0,
                    max_range_err=60.0,
                    radar_R=None,
                    camera_ref=None):
    """Associate radar detections to predicted tracks, in the radar's own polar space.

    ``tracks`` is a sequence with ``.x`` (4,) and ``.P`` (4,4). ``sweep`` exposes ``range``,
    ``azimuth`` and ``range_rate`` arrays. Returns ``(pairs, info)`` where ``pairs`` is
    ``[(track_index, detection_index), ...]`` and ``info`` carries the per-pair residuals the
    caller needs for diagnostics -- crucially the RANGE residual, which is reported rather
    than gated (see the module docstring).

    Three bounds, and the order matters:

    1. statistical -- Mahalanobis over [range, range_rate, azimuth] against S = HPH' + R;
    2. a hard AZIMUTH cap, which is the discriminating one;
    3. a hard RANGE cap, deliberately wide, present only to stop a diverged track with a huge
       P from swallowing the whole sweep.

    Keeping hard caps alongside the statistical gate is necessary, not belt-and-braces: a
    diverged track has a huge S, and a purely statistical gate would admit anything.

    ``camera_ref``: optional per-track sequence of the track's RECENT camera range, expressed from
    the radar origin (None where there is none). Where given and inside the camera's range-trust
    bound, a return further from it than ``tracker.camera_radar_range_cap`` is refused -- see that
    function for the measurement behind it.
    """
    from object_fusion.tracker import radar_h_and_H, radar_R as default_radar_R

    R_meas = default_radar_R() if radar_R is None else np.asarray(radar_R, dtype=np.float64)
    rng = np.asarray(sweep.range, dtype=np.float64)
    az = np.asarray(sweep.azimuth, dtype=np.float64)
    rr = np.asarray(sweep.range_rate, dtype=np.float64)
    n, m = len(tracks), rng.size
    info = {"d_range": {}, "d_azimuth": {}, "nis": {}}
    if n == 0 or m == 0:
        return [], info

    cost = np.full((n, m), np.inf)
    allowed = np.zeros((n, m), dtype=bool)
    resid_r = np.zeros((n, m))
    resid_a = np.zeros((n, m))

    from object_fusion.tracker import camera_radar_range_cap

    for i, tr in enumerate(tracks):
        z_pred, H = radar_h_and_H(tr.x, R_sl, t_sl, v_ego_s)
        S = H @ np.asarray(tr.P, dtype=np.float64) @ H.T + R_meas
        try:
            Sinv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            continue
        y = np.stack([rng - z_pred[0], rr - z_pred[1], wrap_deg(az - z_pred[2])], axis=1)
        d2 = np.einsum("mi,ij,mj->m", y, Sinv, y)
        resid_r[i] = y[:, 0]
        resid_a[i] = y[:, 2]
        allowed[i] = ((d2 <= chi2_gate)
                      & (np.abs(y[:, 2]) <= max_azimuth_err_deg)
                      & (np.abs(y[:, 0]) <= max_range_err))
        if camera_ref is not None and camera_ref[i] is not None:
            cap = camera_radar_range_cap(camera_ref[i])
            if cap is not None:
                allowed[i] &= np.abs(rng - float(camera_ref[i])) <= cap
        cost[i] = d2

    pairs = solve_assignment(cost, allowed)
    for ti, di in pairs:
        info["d_range"][(ti, di)] = float(resid_r[ti, di])
        info["d_azimuth"][(ti, di)] = float(resid_a[ti, di])
        info["nis"][(ti, di)] = float(cost[ti, di])
    return pairs, info


def apply_sticky_ids(tracks, detections, pairs, *, sanity_ok, id_of_track, id_of_detection):
    """Override the geometric assignment where a visual tracker id already owns a track.

    ``tracking_node`` has already solved association in the image, with appearance and IoU
    evidence the 3-D filter does not have. Ignoring it is what breaks tracks at long range:
    when the fused position jumps 14 m between frames because the road is adopted on one frame
    and not the next, a position-only associator breaks the track and respawns it every time,
    so existence never accumulates, nothing ever confirms, and velocity never converges. The
    ByteTrack id is stable across that jump, because in the image nothing moved.

    The override is HARD but not unconditional: ``sanity_ok(track, detection)`` is an outer
    bound (a very loose chi-square, or a fixed metric cap) that stops a recycled id from
    teleporting a track across the scene. ByteTrack does swap and reuse ids, so the caller is
    expected to expire stickiness after a timeout -- ids are per-camera and this stack runs
    only ``/camera_fl``.

    Returns a new pair list. Pairs displaced by a sticky claim are dropped rather than
    reassigned: re-running the assignment on the remainder would let one id override cascade
    into unrelated swaps, which is harder to reason about than a missed update.
    """
    by_track = dict(pairs)
    claimed_t, claimed_d = set(), set()
    forced = []
    for di, det in enumerate(detections):
        did = id_of_detection(det)
        if not did:
            continue
        for ti, tr in enumerate(tracks):
            if id_of_track(tr) != did:
                continue
            if sanity_ok(tr, det):
                forced.append((ti, di))
                claimed_t.add(ti)
                claimed_d.add(di)
            break

    out = list(forced)
    for ti, di in by_track.items():
        if ti in claimed_t or di in claimed_d:
            continue
        out.append((ti, di))
    return sorted(out)
