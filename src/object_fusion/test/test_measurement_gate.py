"""The depth-jump gate: withhold a one-frame depth outlier, never two in a row."""
import pytest

from object_fusion.measurement_gate import DepthJumpGate


def feed(gate, tid, samples):
    return [gate.admit(tid, t, x, 0.0) for t, x in samples]


def approaching(t0=0.0, x0=40.0, v=-12.0, n=6, dt=0.1):
    return [(t0 + i * dt, x0 + v * i * dt) for i in range(n)]


def test_steady_approach_is_never_gated():
    g = DepthJumpGate()
    assert all(feed(g, "7", approaching(n=30)))
    assert g.dropped == 0


def test_one_frame_spike_is_withheld_and_the_track_continues():
    g = DepthJumpGate()
    s = approaching(n=5)                      # 40.0, 38.8, ... 35.2 at t = 0.4
    assert all(feed(g, "7", s))
    assert not g.admit("7", 0.5, 34.0 - 6.0, 0.0), "a road ring 6 m nearer is dropped"
    # The next genuine measurement extrapolates across the gap from the last two ADMITTED ones.
    assert g.admit("7", 0.6, 40.0 - 12.0 * 0.6, 0.0)
    assert g.dropped == 1


def test_spike_in_either_direction():
    for jump in (-5.0, +5.0):
        g = DepthJumpGate()
        feed(g, "7", approaching(n=5))
        assert not g.admit("7", 0.5, 34.0 + jump, 0.0)


def test_never_two_in_a_row_a_real_change_is_accepted():
    """ByteTrack hands the id to another object: the second disagreement restarts the history."""
    g = DepthJumpGate()
    feed(g, "7", approaching(n=5))
    assert not g.admit("7", 0.5, 60.0, 0.0)
    assert g.admit("7", 0.6, 60.0, 0.0)
    assert g.admit("7", 0.7, 60.0, 0.0), "history restarted at the new object; not gated yet"
    assert g.admit("7", 0.8, 60.0, 0.0)


def test_gate_widens_with_range():
    g = DepthJumpGate(gate_min_m=1.5, gate_range_frac=0.05)
    feed(g, "far", [(0.0, 100.0), (0.1, 100.0)])
    assert g.admit("far", 0.2, 104.5, 0.0), "5% of 100 m is 5 m"
    g2 = DepthJumpGate(gate_min_m=1.5, gate_range_frac=0.05)
    feed(g2, "near", [(0.0, 20.0), (0.1, 20.0)])
    assert not g2.admit("near", 0.2, 22.0, 0.0), "at 20 m the 1.5 m floor applies"


def test_no_prediction_without_two_recent_measurements():
    g = DepthJumpGate()
    assert g.admit("7", 0.0, 30.0, 0.0)
    assert g.admit("7", 0.1, 45.0, 0.0), "one sample cannot predict anything"
    g2 = DepthJumpGate()
    feed(g2, "7", [(0.0, 30.0), (0.1, 30.0)])
    assert g2.admit("7", 0.5, 45.0, 0.0), "a 0.4 s gap is too old to extrapolate across"


def test_empty_tracker_id_is_never_gated_and_ids_are_independent():
    g = DepthJumpGate()
    assert all(g.admit("", 0.1 * i, 30.0 + 10.0 * (i % 2), 0.0) for i in range(10))
    feed(g, "a", approaching(n=5))
    feed(g, "b", [(0.0, 80.0), (0.1, 80.0), (0.2, 80.0)])
    assert g.admit("b", 0.3, 80.0, 0.0)
    assert not g.admit("a", 0.5, 80.0, 0.0)


def test_bag_rewind_resets():
    g = DepthJumpGate()
    feed(g, "7", approaching(t0=100.0, n=5))
    assert g.admit("7", 10.0, 80.0, 0.0), "stamps jumped back 90 s: nothing to predict from"


def test_stale_ids_are_forgotten():
    g = DepthJumpGate(forget_after_s=2.0)
    feed(g, "old", approaching(n=3))
    feed(g, "new", approaching(t0=5.0, n=3))
    assert g.predict("old", 5.3) is None and "old" not in g._hist


def test_prediction_is_constant_velocity():
    g = DepthJumpGate()
    feed(g, "7", [(0.0, 50.0), (0.1, 48.0)])
    assert g.predict("7", 0.2) == pytest.approx((46.0, 0.0))
