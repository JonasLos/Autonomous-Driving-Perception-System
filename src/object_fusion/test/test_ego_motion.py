"""Ego-motion increments, and the failure modes that must not be papered over."""
import math

import numpy as np

from object_fusion.ego_motion import EgoTwist, TwistBuffer, frame_increment


def _straight(buf, v=10.0, hz=100, dur=2.0):
    for i in range(int(hz * dur) + 1):
        buf.add(EgoTwist(i / hz, v, 0.0, 0.0))
    return buf


def test_straight_line_increment():
    dpsi, d = _straight(TwistBuffer()).increment(0.0, 1.0)
    assert dpsi == 0.0
    assert np.allclose(d, [10.0, 0.0], atol=1e-6)


def test_turn_matches_the_closed_form_arc():
    """A constant-rate turn is an arc; the integrated increment must land on its chord."""
    buf = TwistBuffer()
    v, w = 10.0, 0.2
    for i in range(201):
        buf.add(EgoTwist(i / 200.0, v, 0.0, w))
    dpsi, d = buf.increment(0.0, 1.0)
    R = v / w
    assert abs(dpsi - w) < 1e-9
    # Discretisation error only; the module documents this bound as ~1.5 cm.
    assert abs(d[0] - R * math.sin(w)) < 0.02
    assert abs(d[1] - R * (1.0 - math.cos(w))) < 0.02


def test_lever_arm_adds_tangential_velocity_only_in_turns():
    straight = frame_increment(EgoTwist(0.0, 10.0, 0.0, 0.0), 1.0, (3.0, 0.0))
    assert np.allclose(straight[1], [10.0, 0.0])
    turning = frame_increment(EgoTwist(0.0, 10.0, 0.0, 0.2), 1.0, (3.0, 0.0))
    assert turning[1][1] == 0.2 * 3.0          # omega * Lx


def test_holds_briefly_then_refuses_rather_than_inventing():
    """A stale twist held too long is 7.5 m of invention applied to every track at once."""
    buf = TwistBuffer(max_hold=0.2)
    _straight(buf, dur=0.5)
    assert buf.at(0.6) is not None and buf.held >= 1
    assert buf.at(5.0) is None
    assert buf.starved >= 1
    assert buf.increment(0.0, 5.0) is None


def test_out_of_order_samples_are_dropped_not_silently_sorted():
    buf = TwistBuffer()
    buf.add(EgoTwist(1.0, 10.0, 0.0, 0.0))
    buf.add(EgoTwist(0.5, 99.0, 0.0, 0.0))
    assert buf.newest().stamp == 1.0
    assert buf.newest().vx == 10.0


def test_interpolates_between_samples():
    buf = TwistBuffer()
    buf.add(EgoTwist(0.0, 0.0, 0.0, 0.0))
    buf.add(EgoTwist(1.0, 10.0, 0.0, 0.0))
    assert abs(buf.at(0.5).vx - 5.0) < 1e-9


def test_an_empty_buffer_counts_as_starved():
    """An empty buffer starves consumers exactly as a stale one does.

    Returning None without counting made a TOTAL odometry outage report odom_starved=0 --
    the diagnostic said healthy while nothing was being processed at all. A lying instrument
    is worse than the fault it hides.
    """
    buf = TwistBuffer()
    assert buf.at(1.0) is None
    assert buf.starved == 1
    assert buf.increment(0.0, 1.0) is None


def test_a_stamp_far_before_the_buffer_is_refused_not_clamped():
    """The mirror of the stale-buffer case. An offline harness that pre-loaded a whole drive into
    an 8 s buffer kept only its last 8 s, and every earlier query was served the oldest sample --
    a twist from minutes later -- which looked like healthy odometry and was not."""
    from object_fusion.ego_motion import EgoTwist, TwistBuffer
    buf = TwistBuffer(duration=8.0, max_hold=0.2)
    for i in range(20):
        buf.add(EgoTwist(100.0 + 0.1 * i, 10.0, 0.0, 0.0))
    assert buf.at(99.95) is not None, "just before the oldest sample is a hold, not a refusal"
    before = buf.starved
    assert buf.at(50.0) is None, "50 s before the buffer must refuse"
    assert buf.starved == before + 1, "and it must be counted"
    assert buf.increment(50.0, 50.1) is None
