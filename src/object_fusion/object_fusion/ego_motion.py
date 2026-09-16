"""Ego motion from the INS, reduced to the SE(2) increment the filter actually needs.

The tracker is EGO-ANCHORED: state lives in the sensor frame at the current epoch and the
frame is moved forward each predict. So the only thing needed from odometry is, over a time
step, how far the sensor frame translated and how far it rotated -- never a global pose.

USE THE TWIST, NEVER THE POSE. Two measured reasons, both on
``selfcal_loc_2026-09-08_11-47-43``:

1. ``pose.position`` is refreshed at roughly a THIRD of the message rate. The median
   per-message pose step is 0.000 m, punctuated by ~0.67 m jumps, while the twist updates
   every message. Differencing the pose at message rate therefore yields zero two times out
   of three and then a spike -- pure artefact velocity injected into every track. Integrated
   over the bag the pose is sound (660.7 m of summed steps against 661.4 m from integrating
   the twist, 0.1% apart), so this is staleness rather than drift, but it is fatal to
   differencing.
2. The pose inherits RTK fix-transition jumps; the INS-smoothed twist does not.

VERIFIED PROPERTIES of the stream (same bag), which this module assumes:

* One clock domain with camera/LiDAR/radar -- ``header.stamp - log_time`` is -0.17 ms for
  odom against -0.10 ms radar, -2.4 ms LiDAR, -4.9 ms camera. No GPS/UTC offset.
* Twist is BODY-referenced (``child_frame_id='base_link'``), median |vy|/|v| = 0.0032 with
  |vx|/|v| = 1.0000. This is what makes ``/odom_grid`` and ``/odom`` interchangeable here:
  they are BIT-IDENTICAL in all four twist components and differ only in
  ``pose.orientation`` yaw, by a constant +1.2870 deg of UTM grid convergence. Prefer
  ``/odom_grid`` anyway -- it costs nothing and protects anything that later reads the pose.
* 105 Hz, worst gap 17.2 ms -- ten times the LiDAR rate, so no interpolation subtlety.
* Independently corroborated: odom speed agrees with the radar's own ``EsrStatus.host_speed``
  to a median 0.024 m/s (p90 0.051).

LEVER ARM. The twist is reported at the IMU, not at ``base_link``: fitting the longitudinal
lever arm against radar range rate recovers 3.63 +/- 0.02 m, which matches radar-minus-IMU
(3.573 m, confirmed independently by jeep_selfcal_loc's extrinsics) and not radar-minus-base_link
(2.915 m).

The LATERAL term is NOT IDENTIFIABLE from radar Doppler, which is why it never fit. Re-run on
turning data only (`scripts/radar_ab.py --lever-arm`, 59 484 observations with |w| > 0.03 rad/s,
2026-09-15): the pooled fit gives ly = -0.105 +/- 0.002 m against a surveyed -0.809, but split by
turn direction it is **-0.248 m on left turns and +0.040 m on right** -- a mounting offset cannot
change sign with the turn, so the estimate is absorbing something that does. Sideslip at the
sensor has exactly the ``w * ly`` signature and flips with the turn; a radar boresight term does
not absorb it either (it fits separately, +0.692 +/- 0.018 deg). The longitudinal arm, by
contrast, is stable across turn directions (3.453 / 3.642).

So the surveyed -0.809 m stands unrefuted and ``lever_arm_xy`` stays at zero -- today's behaviour.
Resolving it needs a reference that separates sideslip from geometry (a stationary yaw test, or
the INS's own sideslip estimate), not more radar data. It only matters in turns, where an
unmodelled arm aliases yaw rate into apparent lateral motion.
"""

from __future__ import annotations

import math
from bisect import bisect_left
from collections import deque

import numpy as np

__all__ = ["EgoTwist", "TwistBuffer", "frame_increment"]

#: Beyond this, a held twist is invention rather than interpolation. 0.5 s at 15 m/s is
#: 7.5 m applied to every track at once. INVENTED; the measured worst odom gap is 17 ms, so
#: this is a fault bound, not a working value.
DEFAULT_MAX_HOLD_S = 0.2


class EgoTwist:
    """Body-frame twist at one instant. ``omega`` is rad/s, positive counter-clockwise."""

    __slots__ = ("stamp", "vx", "vy", "omega")

    def __init__(self, stamp: float, vx: float, vy: float, omega: float):
        self.stamp = float(stamp)
        self.vx = float(vx)
        self.vy = float(vy)
        self.omega = float(omega)

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    def __repr__(self):  # pragma: no cover - diagnostics only
        return (f"EgoTwist(t={self.stamp:.3f}, vx={self.vx:+.2f}, vy={self.vy:+.2f}, "
                f"omega={self.omega:+.4f})")


def frame_increment(twist: EgoTwist, dt: float, lever_arm_xy=(0.0, 0.0)):
    """How the SENSOR frame moved over ``dt``. Returns ``(dpsi, d_xy)``.

    Both are expressed in the frame as it was at the START of the step, which is what
    :func:`object_fusion.tracker.predict` consumes.

    The sensor sits at ``lever_arm_xy`` from the twist's reference point, so its velocity
    picks up the tangential term ``omega x r``:

        v_sensor = (vx - omega*Ly,  vy + omega*Lx)

    Note the rotation is applied about the sensor, not about the reference point: the
    tracker's state is expressed in the sensor frame, so that is the origin the frame change
    must be built around. Getting this backwards was a real error in an earlier radar
    calibration here -- a first lever-arm model that rotated about the sensor origin made the
    fit WORSE (77.8% against 100%) before it was corrected.
    """
    lx, ly = float(lever_arm_xy[0]), float(lever_arm_xy[1])
    vsx = twist.vx - twist.omega * ly
    vsy = twist.vy + twist.omega * lx
    return twist.omega * dt, np.array([vsx * dt, vsy * dt], dtype=np.float64)


class TwistBuffer:
    """Time-ordered twist history with bounded hold, for querying increments between stamps.

    Stamps are only ever compared to each other, never to a clock, so this is valid under bag
    replay regardless of ``use_sim_time`` -- the same contract
    ``perception_common.stamp_sync`` keeps.
    """

    def __init__(self, duration: float = 5.0, max_hold: float = DEFAULT_MAX_HOLD_S):
        self.duration = float(duration)
        self.max_hold = float(max_hold)
        self._q: deque[EgoTwist] = deque()
        #: Counts of increments served from a held (stale) sample, and refusals past max_hold.
        self.held = 0
        self.starved = 0

    def add(self, twist: EgoTwist) -> None:
        # Out-of-order arrivals are dropped rather than sorted in: at 105 Hz a reordering is
        # a stream fault, and silently repairing it would hide that.
        if self._q and twist.stamp <= self._q[-1].stamp:
            return
        self._q.append(twist)
        cutoff = twist.stamp - self.duration
        while self._q and self._q[0].stamp < cutoff:
            self._q.popleft()

    def newest(self):
        return self._q[-1] if self._q else None

    def at(self, stamp: float):
        """Twist interpolated to ``stamp``, or ``None`` if it cannot be served honestly.

        Past the newest sample the last one is HELD for up to ``max_hold`` and the ``held``
        counter increments; beyond that this returns ``None`` and the caller must stop
        predicting rather than keep extrapolating a stale velocity across every track.
        """
        if not self._q:
            # Counted, not silent. An empty buffer starves every consumer exactly as a stale
            # one does, and reporting 0 here made a total odometry outage look healthy.
            self.starved += 1
            return None
        stamps = [t.stamp for t in self._q]
        if stamp <= stamps[0]:
            # Symmetric with the "past the newest sample" case below, and for the same reason: a
            # stamp far BEFORE anything in the buffer cannot be served honestly either. Returning
            # the oldest sample there is how an offline harness that pre-loaded a whole drive
            # silently predicted every early measurement with a twist from 200 s later.
            if stamps[0] - stamp > self.max_hold:
                self.starved += 1
                return None
            if stamp < stamps[0]:
                self.held += 1
            return self._q[0]
        if stamp >= stamps[-1]:
            if stamp - stamps[-1] > self.max_hold:
                self.starved += 1
                return None
            if stamp > stamps[-1]:
                self.held += 1
            return self._q[-1]
        i = bisect_left(stamps, stamp)
        a, b = self._q[i - 1], self._q[i]
        span = b.stamp - a.stamp
        w = 0.0 if span <= 0.0 else (stamp - a.stamp) / span
        return EgoTwist(stamp, a.vx + w * (b.vx - a.vx), a.vy + w * (b.vy - a.vy),
                        a.omega + w * (b.omega - a.omega))

    def increment(self, t0: float, t1: float, lever_arm_xy=(0.0, 0.0)):
        """Frame increment from ``t0`` to ``t1``. ``None`` when odometry cannot cover it.

        Integrated at the buffer's own sample rate rather than with a single midpoint step,
        so a turn spanning several samples is not flattened. Returns ``(dpsi, d_xy)`` with
        ``d_xy`` accumulated in the t0 frame.
        """
        if t1 <= t0:
            return 0.0, np.zeros(2)
        edges = [t0] + [t.stamp for t in self._q if t0 < t.stamp < t1] + [t1]
        dpsi_total = 0.0
        d_total = np.zeros(2)
        for a, b in zip(edges[:-1], edges[1:]):
            mid = self.at(0.5 * (a + b))
            if mid is None:
                return None
            dpsi, d = frame_increment(mid, b - a, lever_arm_xy)
            # Accumulate in the ORIGINAL t0 frame: each sub-step's displacement is expressed
            # in the frame at its own start, so it must be rotated back by the yaw already
            # accumulated before it.
            c, s = math.cos(dpsi_total), math.sin(dpsi_total)
            d_total = d_total + np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
            dpsi_total += dpsi
        return dpsi_total, d_total
