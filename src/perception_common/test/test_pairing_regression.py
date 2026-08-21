"""Characterisation test for the bounded-wait pairing, against the vehicle's measured timing.

The unit tests in test_stamp_sync.py check the state machine one transition at a time. This
one drives the buffer with the actual timing recorded on the vehicle on 2026-08-13 -- a 10Hz
LiDAR and a ~10Hz camera on separate oscillators, arriving 0.031s and 0.014s after their own
stamps -- and asserts the three properties the 2026-08-17 change was made to get:

  1. one-sided matching at max_skew=0.06 starves (this is the failure that forced 0.12);
  2. bounded-wait matching at 0.06 does not, and matches at essentially the full rate;
  3. it roughly halves mean |skew| while doing so.

If someone tightens max_skew, shortens wait_for_newer, or breaks the deferral resolution, the
first two assertions fail here rather than on the vehicle. Run it over a duration longer than
the ~60-100s camera/LiDAR beat, or the phase never sweeps and every configuration looks fine.
"""

import numpy as np
import pytest

from perception_common.stamp_sync import DEFERRED, StampMatchedBuffer

# Measured on the vehicle, 2026-08-13. See CHANGELOG Operational Findings.
LIDAR_HZ = 10.0
CAMERA_HZ = 10.017          # free-running, hence the ~60s beat against the LiDAR
LIDAR_ARRIVAL_DELAY = 0.031
TRACKING_ARRIVAL_DELAY = 0.014
DURATION = 240.0            # > two full beat periods
PUMP_HZ = 50.0              # matches deferral_pump_period=0.02


class _Stamp:
    __slots__ = ("sec", "nanosec")

    def __init__(self, t):
        self.sec = int(t)
        self.nanosec = int(round((t - int(t)) * 1e9))


class _Header:
    __slots__ = ("stamp", "frame_id")

    def __init__(self, t):
        self.stamp = _Stamp(t)
        self.frame_id = "lidar"


class _Msg:
    __slots__ = ("header", "capture")

    def __init__(self, t):
        self.header = _Header(t)
        self.capture = t


def _timeline():
    """(arrival_time, kind, capture_stamp) for both streams, in arrival order."""
    events = []
    t = 0.0
    while t < DURATION:
        events.append((t + LIDAR_ARRIVAL_DELAY, "lidar", t))
        t += 1.0 / LIDAR_HZ
    # Offset so the two streams do not start exactly in phase.
    t = 0.0005
    while t < DURATION:
        events.append((t + TRACKING_ARRIVAL_DELAY, "detection", t))
        t += 1.0 / CAMERA_HZ
    events.sort()
    return events


def _run(wait_for_newer, max_skew):
    buf = StampMatchedBuffer(
        "projection",
        buffer_duration=2.0,
        max_skew=max_skew,
        wait_for_newer=wait_for_newer,
        wrap=lambda m: m,
    )
    completed = []
    published_sample_stamps = []

    def complete(pairing):
        completed.append(pairing)
        if pairing.value is not None:
            published_sample_stamps.append(pairing.value.capture)

    next_pump = 0.0
    for arrival, kind, stamp in _timeline():
        if kind == "lidar":
            buf.add(_Msg(stamp))
            # The node pumps immediately after buffering; this resolves most deferrals.
            for p in buf.drain(arrival):
                complete(p)
        else:
            pairing = buf.match(_Header(stamp), now=arrival, payload=stamp)
            if pairing.outcome is not DEFERRED:
                complete(pairing)

        if arrival >= next_pump:
            next_pump = arrival + 1.0 / PUMP_HZ
            for p in buf.drain(arrival):
                complete(p)

    # Flush anything still deferred at the end of the run.
    for p in buf.drain(DURATION + 10.0):
        complete(p)

    matched = [p for p in completed if p.value is not None]
    skews = [abs(p.skew) for p in matched]
    return {
        "matched": len(matched),
        "unmatched": len(completed) - len(matched),
        "mean_skew": float(np.mean(skews)) if skews else float("nan"),
        "max_skew": float(np.max(skews)) if skews else float("nan"),
        "published": published_sample_stamps,
        "buffer": buf,
    }


@pytest.fixture(scope="module")
def one_sided_wide():
    """The pre-2026-08-17 production config: one-sided matching, widened bound."""
    return _run(wait_for_newer=0.0, max_skew=0.12)


@pytest.fixture(scope="module")
def one_sided_tight():
    """The config that starved on 2026-08-13: tight bound, still one-sided."""
    return _run(wait_for_newer=0.0, max_skew=0.06)


@pytest.fixture(scope="module")
def two_sided():
    """The current config: bounded wait makes the tight bound reachable."""
    return _run(wait_for_newer=0.06, max_skew=0.06)


def test_one_sided_matching_starves_at_the_tight_bound(one_sided_tight, one_sided_wide):
    """Reproduce the 2026-08-13 failure, so the reason for the deferral stays evidenced."""
    starved = one_sided_tight["matched"] / one_sided_wide["matched"]
    assert starved < 0.75, (
        "Expected one-sided matching at max_skew=0.06 to starve, but it matched "
        f"{starved:.0%} of the wide-bound run. If this ever passes cleanly, the timing "
        "constants here no longer reflect the vehicle."
    )
    assert one_sided_tight["unmatched"] > 500


def test_bounded_wait_restores_the_match_rate(two_sided, one_sided_wide):
    """The same tight bound, now reachable, must not cost throughput."""
    assert two_sided["matched"] >= 0.99 * one_sided_wide["matched"]
    # A handful of unmatched at the very start (empty buffer) is expected; sustained
    # unmatched is the starvation signature.
    assert two_sided["unmatched"] <= 5


def test_bounded_wait_at_least_halves_mean_skew(two_sided, one_sided_wide):
    """The actual claim: pairing error, not throughput."""
    assert two_sided["mean_skew"] <= 0.55 * one_sided_wide["mean_skew"]
    assert two_sided["max_skew"] <= 0.06 + 1e-9


@pytest.mark.parametrize("case", ["one_sided_wide", "one_sided_tight", "two_sided"])
def test_published_sample_stamps_never_go_backwards(case, request):
    stamps = request.getfixturevalue(case)["published"]
    backwards = [
        (i, a, b) for i, (a, b) in enumerate(zip(stamps, stamps[1:])) if b < a
    ]
    assert not backwards, f"output stamp went backwards at {backwards[:3]}"


def test_no_deferral_is_left_dangling(two_sided):
    assert two_sided["buffer"].pending_count() == 0
