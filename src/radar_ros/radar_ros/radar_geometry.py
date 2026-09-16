"""Pure geometry and association for the Delphi ESR radar. No ROS imports.

Kept at module scope and free of rclpy for the same reason ``perception_common.stamp_sync``
is: the offline A/B harness (``scripts/radar_ab.py``) imports these functions directly, so
what it scores is literally the production rule rather than a copy that drifts. The unit
tests in ``src/perception_common/test/test_radar_geometry.py`` import them the same way.
"""

import numpy as np

# Delphi ESR 2.5 track slots. The driver always emits a fixed-size array; empty slots come
# through as zero-range entries rather than being omitted.
ESR_TRACK_SLOTS = 64

# track_status is a 3-bit field. 0 means "no target"; everything else is some flavour of
# valid target and the driver does not decode the individual meanings. Measured on the
# 2026-08-25 bag the driver never emits status 0 at all -- it drops empty slots itself and
# sends only live tracks (~11 per sweep, not the full 64) -- so this check is a cheap
# belt-and-braces against a future driver that does forward them.
STATUS_NO_TARGET = 0

# amplitude is the track motion-power estimate. Measured range on the 2026-08-25 bag is
# -10..18, and -10 is the single most common value (~30% of tracks), so it reads as the
# sensor's floor rather than as a genuinely weak return. Anything above -10 is a usable
# default, which is why min_amplitude defaults to effectively-off in the node: a naive
# threshold of 0.0 discards about half of everything the radar reports.
AMPLITUDE_FLOOR = -10.0


def polar_to_cartesian(range_m, angle_deg):
    """ESR polar measurement -> Cartesian in the RADAR frame (x forward, y left).

    ``angle`` is positive to the LEFT of boresight, matching REP-103's right-handed z-up
    convention once x is forward. Both this and the ``range_rate`` sign are VERIFIED on the
    2026-08-25 bag rather than taken from the .msg comments:

    * ``range_rate`` positive = receding. For a static target it must equal
      ``-v_ego * cos(azimuth)``: 98.6% of 16219 track-observations fall within 3 m/s of that
      prediction, median residual -0.06 m/s, p90 |residual| 0.45 m/s. Flipping the sign, or
      dropping the ego-motion term, each drops the agreement to 0.0%.

    * The azimuth sign CANNOT be settled that way -- ``cos`` is even, so a flipped azimuth
      scores identically (98.6%). It is settled instead by the lever-arm term, which is odd
      in azimuth: while the vehicle yaws at w, the sensor origin translates laterally, adding
      ``-w * L * sin(azimuth)`` to the range rate. Regressing the residual recovers
      ``L = +2.82 +/- 0.41 m`` (6.8 sigma from zero) against a surveyed radar mounting of
      +2.915 m forward of ``lidar_tc``. Positive L means positive azimuth is to the left; a
      right-positive convention would have fitted a negative lever arm.

    Re-run both with ``scripts/radar_ab.py --check-conventions`` if the driver's decoding
    ever changes.
    """
    a = np.radians(np.asarray(angle_deg, dtype=np.float64))
    r = np.asarray(range_m, dtype=np.float64)
    return r * np.cos(a), r * np.sin(a)


def cartesian_to_polar(x, y):
    """Inverse of :func:`polar_to_cartesian`. Returns ``(range_m, angle_deg)``."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    return np.hypot(x, y), np.degrees(np.arctan2(y, x))


def gate_tracks(
    ranges,
    angles,
    amplitudes,
    statuses,
    update_counts,
    *,
    min_range,
    max_range,
    min_amplitude,
    min_update_count,
):
    """Boolean mask of tracks worth associating.

    Drops empty slots first. This driver already omits them, but an unfilled ESR slot would
    report range 0.0 with status 0, and a zero-range track sits at the sensor origin and
    wins every near-field association -- cheap enough to keep guarding against.

    ``min_update_count`` is INERT with the current driver. EsrTrack.msg documents the field
    as "rolling_count bit from track frame", and it is 0 on every track in every sweep of
    the 2026-08-25 bag -- a third stub alongside is_cipv and range_rate_ambiguous. It is
    kept as a parameter so a future driver that populates it needs no code change, but it
    defaults to 0 and MUST NOT be raised on the current driver: any positive threshold
    discards 100% of tracks. It was specced as a persistence-based clutter filter; that
    filter does not exist yet and would have to be built from track_id continuity instead.

    ``min_amplitude`` defaults effectively-off for the reason given at AMPLITUDE_FLOOR.
    """
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = np.asarray(angles, dtype=np.float64)
    amplitudes = np.asarray(amplitudes, dtype=np.float64)
    statuses = np.asarray(statuses)
    update_counts = np.asarray(update_counts)

    return (
        (statuses != STATUS_NO_TARGET)
        & (ranges >= max(min_range, 1e-3))
        & (ranges <= max_range)
        & (amplitudes >= min_amplitude)
        & (update_counts >= min_update_count)
        & np.isfinite(ranges)
        & np.isfinite(angles)
    )


def associate(
    obj_range,
    obj_azimuth_deg,
    radar_range,
    radar_azimuth_deg,
    *,
    max_range_err,
    max_azimuth_err_deg,
):
    """Match fused objects to radar tracks in the radar's own polar frame.

    Both inputs must already be expressed as (range, azimuth) about the radar origin --
    the caller transforms the fused positions out of ``lidar_tc`` first. Comparing in the
    sensor's native measurement space is the whole point: the ESR resolves range to ~0.1 m
    out to 175 m but its azimuth is good to only ~0.5 deg, which is ~0.9 m of lateral error
    at 100 m. A single Euclidean radius has to be either too tight to match anything in the
    far field or so loose it accepts unrelated objects in the near field. Separate gates
    give one bound that stays physical at every range.

    Returns ``[(obj_index, radar_index), ...]``. Never returns a pair outside the gates,
    so an empty result is a legitimate outcome rather than a failure.
    """
    obj_range = np.asarray(obj_range, dtype=np.float64)
    obj_azimuth_deg = np.asarray(obj_azimuth_deg, dtype=np.float64)
    radar_range = np.asarray(radar_range, dtype=np.float64)
    radar_azimuth_deg = np.asarray(radar_azimuth_deg, dtype=np.float64)

    if obj_range.size == 0 or radar_range.size == 0:
        return []
    if max_range_err <= 0.0 or max_azimuth_err_deg <= 0.0:
        return []

    d_range = np.abs(obj_range[:, None] - radar_range[None, :])
    d_az = np.abs(obj_azimuth_deg[:, None] - radar_azimuth_deg[None, :])

    allowed = (d_range <= max_range_err) & (d_az <= max_azimuth_err_deg)
    if not np.any(allowed):
        return []

    # Normalised so the two axes are commensurate: at the gate boundary each contributes
    # exactly 1.0, so neither dimension silently dominates the assignment.
    cost = (d_range / max_range_err) ** 2 + (d_az / max_azimuth_err_deg) ** 2

    # Forbidden pairs must be more expensive than any tour of allowed ones, or the optimiser
    # will take one to complete a larger assignment. 2.0 is the worst allowed cost.
    blocked = 1e6
    cost = np.where(allowed, cost, blocked)

    try:
        from scipy.optimize import linear_sum_assignment

        rows, cols = linear_sum_assignment(cost)
        pairs = [(int(r), int(c)) for r, c in zip(rows, cols) if allowed[r, c]]
    except ImportError:
        # Greedy nearest-first. At these object counts (<20 objects, <=64 tracks) it differs
        # from the optimum only in genuinely ambiguous scenes, so it is an acceptable
        # fallback rather than a silent downgrade worth failing over.
        pairs = []
        used_r, used_c = set(), set()
        order = np.dstack(np.unravel_index(np.argsort(cost, axis=None), cost.shape))[0]
        for r, c in order:
            r, c = int(r), int(c)
            if not allowed[r, c]:
                break
            if r in used_r or c in used_c:
                continue
            used_r.add(r)
            used_c.add(c)
            pairs.append((r, c))

    return sorted(pairs)


def radial_velocity_vector(range_rate, azimuth_deg):
    """Range rate -> a velocity vector in the RADAR frame.

    The ESR measures closing speed along the sensor-to-target ray and nothing else, so the
    result is that scalar placed back on the ray. A target crossing the beam at 20 m/s
    yields approximately zero. ``lat_rate`` exists in the message but is quantised to
    0.25 m/s, which is coarser than most of the values it would carry, so it is deliberately
    not used to synthesise a second component.

    Sign: ``range_rate`` is positive when receding, so a closing target gives a vector
    pointing back towards the sensor.
    """
    a = np.radians(np.asarray(azimuth_deg, dtype=np.float64))
    rr = np.asarray(range_rate, dtype=np.float64)
    return rr * np.cos(a), rr * np.sin(a)
