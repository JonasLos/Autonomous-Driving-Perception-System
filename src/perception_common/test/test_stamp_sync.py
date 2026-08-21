"""Unit tests for the deferral state machine in :mod:`perception_common.stamp_sync`.

These run without a ROS context -- no rclpy, no executor, no spinning node -- which is the
whole reason the buffer takes ``now`` as an opaque float instead of owning a timer. Ordering
guarantees are asserted here rather than on the vehicle, because the input sequences that break
them are exactly the ones a bag replay will not reproduce on demand.
"""

import pytest

from perception_common.stamp_sync import (
    DEFERRED,
    MATCHED,
    UNMATCHED,
    StampMatchedBuffer,
)


class FakeStamp:
    __slots__ = ("sec", "nanosec")

    def __init__(self, seconds):
        self.sec = int(seconds)
        self.nanosec = int(round((seconds - int(seconds)) * 1e9))


class FakeHeader:
    __slots__ = ("stamp", "frame_id")

    def __init__(self, seconds, frame_id="lidar"):
        self.stamp = FakeStamp(seconds)
        self.frame_id = frame_id


class FakeMsg:
    """Minimal stand-in for any stamped message; ``wrap`` returns it unchanged in these tests."""

    __slots__ = ("header", "tag")

    def __init__(self, seconds, tag=None):
        self.header = FakeHeader(seconds)
        self.tag = tag if tag is not None else seconds


def make_buffer(**kwargs):
    kwargs.setdefault("max_skew", 0.06)
    kwargs.setdefault("buffer_duration", 2.0)
    return StampMatchedBuffer("sample", wrap=lambda m: m, **kwargs)


# --------------------------------------------------------------------------------------
# wait_for_newer = 0: the pre-deferral behaviour
# --------------------------------------------------------------------------------------


def test_zero_wait_resolves_immediately_and_never_defers():
    buf = make_buffer(wait_for_newer=0.0)
    buf.add(FakeMsg(100.00))

    # Reference is newer than everything buffered: one-sided matching reaches backwards.
    p = buf.match(FakeHeader(100.03), now=0.0)
    assert p.outcome is MATCHED
    assert p.value.tag == 100.00
    assert p.skew == pytest.approx(-0.03)
    assert buf.deferred_count == 0


def test_zero_wait_rejects_beyond_max_skew():
    buf = make_buffer(wait_for_newer=0.0, max_skew=0.06)
    buf.add(FakeMsg(100.00))

    p = buf.match(FakeHeader(100.50), now=0.0)
    assert p.outcome is UNMATCHED
    assert p.skew == pytest.approx(-0.50)
    assert "older than" in buf.describe_unmatched(p.skew, "detection", reason=p.reason)


def test_empty_buffer_is_unmatched_not_deferred_at_zero_wait():
    buf = make_buffer(wait_for_newer=0.0)
    p = buf.match(FakeHeader(100.0), now=0.0)
    assert p.outcome is UNMATCHED
    assert p.skew == float("inf")
    assert "no sample buffered" in buf.describe_unmatched(p.skew)


def test_zero_stamp_reference_is_unmatched_with_infinite_skew():
    buf = make_buffer(wait_for_newer=0.06)
    buf.add(FakeMsg(100.0))

    p = buf.match(FakeHeader(0.0), now=0.0)
    assert p.outcome is UNMATCHED
    assert p.skew == float("inf")
    assert buf.zero_stamp_count == 1


def test_zero_stamp_sample_is_rejected_by_add():
    buf = make_buffer()
    assert buf.add(FakeMsg(0.0)) is False
    assert buf.zero_stamp_count == 1
    assert buf.newest() is None


# --------------------------------------------------------------------------------------
# Deferral: the two-sided path
# --------------------------------------------------------------------------------------


def test_reference_ahead_of_buffer_defers_then_resolves_on_newer_sample():
    buf = make_buffer(wait_for_newer=0.06)
    buf.add(FakeMsg(100.00))

    # 100.02 is newer than anything buffered, so the sample it should pair with may still
    # be in flight. Deferring is the whole point.
    p = buf.match(FakeHeader(100.02), now=0.000)
    assert p.outcome is DEFERRED
    assert buf.pending_count() == 1

    # The awaited sample lands 10ms later and resolves it in the same tick. 100.03 is nearer
    # to the reference than the buffered 100.00 (an exactly equidistant sample would be a tie
    # that min() breaks towards the older one, which is not what this test is about).
    buf.add(FakeMsg(100.03))
    out = buf.drain(now=0.010)
    assert len(out) == 1
    assert out[0].outcome is MATCHED
    assert out[0].value.tag == 100.03, "should pair forwards, not backwards to 100.00"
    assert out[0].skew == pytest.approx(0.01)
    assert buf.pending_count() == 0


def test_deferral_halves_skew_versus_one_sided_matching():
    """The claim the whole change rests on, as a direct comparison."""
    one_sided = make_buffer(wait_for_newer=0.0)
    two_sided = make_buffer(wait_for_newer=0.06)

    for buf in (one_sided, two_sided):
        buf.add(FakeMsg(100.00))

    ref = FakeHeader(100.045)
    a = one_sided.match(ref, now=0.0)
    b = two_sided.match(ref, now=0.0)

    assert a.outcome is MATCHED
    assert abs(a.skew) == pytest.approx(0.045)  # forced back onto 100.00

    assert b.outcome is DEFERRED
    two_sided.add(FakeMsg(100.05))
    resolved = two_sided.drain(now=0.005)[0]
    assert resolved.outcome is MATCHED
    assert abs(resolved.skew) == pytest.approx(0.005)  # paired with the nearer, newer sample
    assert abs(resolved.skew) < abs(a.skew)


def test_deferral_expires_when_no_newer_sample_arrives():
    buf = make_buffer(wait_for_newer=0.06)
    buf.add(FakeMsg(100.00))

    p = buf.match(FakeHeader(100.02), now=1.000)
    assert p.outcome is DEFERRED

    # Not yet expired, and nothing newer buffered: still held.
    assert buf.drain(now=1.030) == []
    assert buf.pending_count() == 1

    # Past wait_for_newer: fall back to the best backwards match rather than dropping it.
    out = buf.drain(now=1.061)
    assert len(out) == 1
    assert out[0].outcome is MATCHED
    assert out[0].value.tag == 100.00
    assert buf.expired_count == 1


def test_expiry_beyond_max_skew_reports_the_expired_reason():
    buf = make_buffer(wait_for_newer=0.06, max_skew=0.01)
    buf.add(FakeMsg(100.00))

    buf.match(FakeHeader(100.50), now=0.0)
    out = buf.drain(now=0.100)
    assert out[0].outcome is UNMATCHED
    assert out[0].reason == "expired"
    assert "wait_for_newer" in buf.describe_unmatched(out[0].skew, reason=out[0].reason)


def test_backwards_clock_step_releases_rather_than_parks_forever():
    buf = make_buffer(wait_for_newer=0.06)
    buf.add(FakeMsg(100.00))
    buf.match(FakeHeader(100.02), now=5.000)

    # Sim time rewound (bag loop). now - deferred_at is negative; holding would be forever.
    out = buf.drain(now=0.100)
    assert len(out) == 1
    assert buf.expired_count == 1


def test_payload_round_trips_through_deferral():
    buf = make_buffer(wait_for_newer=0.06)
    buf.add(FakeMsg(100.00))
    sentinel = object()

    p = buf.match(FakeHeader(100.02), now=0.0, payload=sentinel)
    assert p.outcome is DEFERRED
    buf.add(FakeMsg(100.03))
    assert buf.drain(now=0.001)[0].payload is sentinel


# --------------------------------------------------------------------------------------
# Ordering guarantees
# --------------------------------------------------------------------------------------


def test_head_of_line_blocking_preserves_reference_order():
    buf = make_buffer(wait_for_newer=0.06)
    buf.add(FakeMsg(100.00))

    # Two references defer; the second could resolve first if it were allowed to overtake.
    assert buf.match(FakeHeader(100.02), now=0.0, payload="first").outcome is DEFERRED
    assert buf.match(FakeHeader(100.04), now=0.0, payload="second").outcome is DEFERRED

    buf.add(FakeMsg(100.05))
    out = buf.drain(now=0.010)
    assert [p.payload for p in out] == ["first", "second"]
    assert out[0].reference_stamp < out[1].reference_stamp


def test_output_sample_stamps_are_non_decreasing_under_inversion():
    """The case reference monotonicity alone does not cover.

    Both references here are strictly increasing, so the reference watermark never fires. But
    a sample that arrives late and is *older* than the one already published can still sit
    nearest to the later reference. That is reachable whenever t2 - t1 < 2 * max_skew, which
    at 10Hz with an 0.06 bound is the normal operating point. Without the sample watermark
    this publishes a backwards stamp.
    """
    buf = make_buffer(wait_for_newer=0.0, max_skew=0.06)
    # A realistic backlog, so the late sample below reads as out-of-order delivery rather
    # than as the bag-loop backwards jump that add() resets on.
    for t in (99.700, 99.800, 99.900, 100.000):
        buf.add(FakeMsg(t))
    buf.add(FakeMsg(100.100))

    # Nearest to 100.090 is 100.100 (skew +0.010).
    first = buf.match(FakeHeader(100.090), now=0.0)
    assert first.outcome is MATCHED
    assert first.value.tag == 100.100

    # A sample captured before 100.100 is buffered late, and is strictly nearer to the next
    # reference (0.003 away) than the already-published 100.100 is (0.005 away). Nearest-wins
    # would pick it and walk the output stamp backwards from 100.100 to 100.092.
    buf.add(FakeMsg(100.092))
    second = buf.match(FakeHeader(100.095), now=0.0)
    assert second.reference_stamp > first.reference_stamp, "reference order is fine"

    # The watermark excludes it, so the pairing degrades to the nearest *non-decreasing*
    # sample rather than dropping the frame -- output stays monotonic and still publishes.
    assert second.outcome is MATCHED
    assert second.value.tag == 100.100
    assert second.skew == pytest.approx(0.005)

    # A later reference that lands on something genuinely newer is served normally.
    buf.add(FakeMsg(100.140))
    third = buf.match(FakeHeader(100.135), now=0.0)
    assert third.outcome is MATCHED
    assert third.value.tag == 100.140

    published = [first.value.tag, second.value.tag, third.value.tag]
    assert published == sorted(published), "output sample stamps must be non-decreasing"


def test_reference_at_or_behind_the_last_emitted_is_rejected():
    buf = make_buffer(wait_for_newer=0.0)
    buf.add(FakeMsg(100.00))

    assert buf.match(FakeHeader(100.02), now=0.0).outcome is MATCHED

    replay = buf.match(FakeHeader(100.02), now=0.0)
    assert replay.outcome is UNMATCHED
    assert replay.reason == "out-of-order"

    older = buf.match(FakeHeader(100.01), now=0.0)
    assert older.outcome is UNMATCHED
    assert older.reason == "out-of-order"
    assert buf.stale_reference_count == 2


def test_rejected_reference_is_retired_not_reconsidered():
    """A reference rejected on skew must not sneak back in once the buffer changes."""
    buf = make_buffer(wait_for_newer=0.0, max_skew=0.01)
    buf.add(FakeMsg(100.00))

    assert buf.match(FakeHeader(100.50), now=0.0).outcome is UNMATCHED
    buf.add(FakeMsg(100.50))
    again = buf.match(FakeHeader(100.50), now=0.0)
    assert again.outcome is UNMATCHED
    assert again.reason == "out-of-order"


# --------------------------------------------------------------------------------------
# Per-key independence (SAM3's left/right sides)
# --------------------------------------------------------------------------------------


def test_a_silent_key_does_not_block_another():
    buf = make_buffer(wait_for_newer=0.06)
    buf.add(FakeMsg(100.00))

    assert buf.match(FakeHeader(100.02), now=0.0, key="left").outcome is DEFERRED
    assert buf.match(FakeHeader(100.02), now=0.0, key="right").outcome is DEFERRED

    buf.add(FakeMsg(100.04))
    out = buf.drain(now=0.001)
    assert {p.key for p in out} == {"left", "right"}
    assert all(p.outcome is MATCHED for p in out)
    # Both sides of one frame share a single buffered object, hence a single parse and tree.
    assert out[0].value is out[1].value


def test_draining_one_key_leaves_the_other_pending():
    buf = make_buffer(wait_for_newer=0.06)
    buf.add(FakeMsg(100.00))
    buf.match(FakeHeader(100.02), now=0.0, key="left")
    buf.match(FakeHeader(100.02), now=0.0, key="right")
    buf.add(FakeMsg(100.04))

    out = buf.drain(now=0.001, key="left")
    assert [p.key for p in out] == ["left"]
    assert buf.pending_count(key="right") == 1


def test_keys_have_independent_watermarks():
    buf = make_buffer(wait_for_newer=0.0)
    buf.add(FakeMsg(100.00))

    assert buf.match(FakeHeader(100.02), now=0.0, key="left").outcome is MATCHED
    # The same stamp on the other side is a different stream and must still be accepted.
    assert buf.match(FakeHeader(100.02), now=0.0, key="right").outcome is MATCHED


# --------------------------------------------------------------------------------------
# Bag replay: backwards jumps and eviction
# --------------------------------------------------------------------------------------


def test_backwards_stamp_jump_clears_buffer_pending_and_watermarks():
    buf = make_buffer(wait_for_newer=0.06)
    buf.add(FakeMsg(200.00))
    assert buf.match(FakeHeader(200.02), now=0.0).outcome is DEFERRED
    assert buf.pending_count() == 1

    # Bag loops back to the start.
    buf.add(FakeMsg(100.00))
    assert buf.reset_count == 1
    assert buf.pending_count() == 0
    assert buf.flushed_on_reset_count == 1

    # The stale watermark from the previous pass must not reject the new pass.
    p = buf.match(FakeHeader(100.01), now=1.0)
    assert p.outcome is not UNMATCHED


def test_out_of_order_sample_keeps_the_buffer_sorted():
    """A late sample must not leave _entries[-1] reading as something other than the newest."""
    buf = make_buffer(wait_for_newer=0.0)
    for t in (100.00, 100.10, 100.30):
        buf.add(FakeMsg(t))

    buf.add(FakeMsg(100.20))  # delivered out of order
    assert buf.out_of_order_sample_count == 1
    assert buf.reset_count == 0, "a few ms of reordering is not a bag loop"
    assert buf.newest().tag == 100.30

    stamps = [e.stamp for e in buf._entries]
    assert stamps == sorted(stamps)


def test_eviction_is_keyed_on_buffered_stamps_not_the_caller_clock():
    buf = make_buffer(wait_for_newer=0.0, buffer_duration=0.5)
    for t in (100.0, 100.2, 100.4, 100.6, 100.8):
        buf.add(FakeMsg(t))

    # 100.0 and 100.2 fall outside [100.8 - 0.5, 100.8]; a huge `now` changes nothing.
    p = buf.match(FakeHeader(100.05), now=1e9, key="probe")
    assert p.outcome is UNMATCHED

    assert buf.match(FakeHeader(100.79), now=1e9).outcome is MATCHED


def test_clear_resets_everything():
    buf = make_buffer(wait_for_newer=0.06)
    buf.add(FakeMsg(100.00))
    buf.match(FakeHeader(100.02), now=0.0)
    buf.clear()

    assert buf.newest() is None
    assert buf.pending_count() == 0

    # The watermark is gone too, so a reference behind the one just cleared is accepted again.
    p = buf.match(FakeHeader(100.01), now=0.0)
    assert p.outcome is DEFERRED
    assert buf.drain(now=1.0)[0].outcome is UNMATCHED


def test_empty_buffer_defers_at_startup_rather_than_dropping():
    """Detections that arrive before the first sample are worth holding briefly."""
    buf = make_buffer(wait_for_newer=0.06)

    p = buf.match(FakeHeader(100.02), now=0.0)
    assert p.outcome is DEFERRED

    buf.add(FakeMsg(100.03))
    out = buf.drain(now=0.005)
    assert out[0].outcome is MATCHED


# --------------------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------------------


def test_status_reports_skew_statistics_once_matches_exist():
    buf = make_buffer(wait_for_newer=0.0)
    assert "mean|skew|=n/a" in buf.status()

    buf.add(FakeMsg(100.00))
    buf.match(FakeHeader(100.02), now=0.0)

    status = buf.status()
    assert "matched=1" in status
    assert "mean|skew|=0.0200s" in status
    assert "wait_for_newer=0.000s" in status


def test_stamp_offset_shifts_buffered_stamps():
    buf = make_buffer(wait_for_newer=0.0, stamp_offset=-0.05)
    buf.add(FakeMsg(100.10))  # treated as captured at 100.05

    p = buf.match(FakeHeader(100.05), now=0.0)
    assert p.outcome is MATCHED
    assert p.skew == pytest.approx(0.0)
