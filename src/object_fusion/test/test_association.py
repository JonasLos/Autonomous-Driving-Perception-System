"""Association: gate ordering, global assignment, and the Phase 0 regression."""
import math

import numpy as np
import pytest

from object_fusion.association import (
    BLOCKED_COST, apply_sticky_ids, associate_radar, mahalanobis_sq, solve_assignment,
)


class Trk:
    def __init__(self, x, P=None, bid=""):
        self.x = np.asarray(x, dtype=np.float64)
        self.P = np.eye(4) * 1.0 if P is None else np.asarray(P, dtype=np.float64)
        self.bytetrack_id = bid


class Sweep:
    def __init__(self, rng, az, rr=None):
        self.range = np.asarray(rng, dtype=np.float64)
        self.azimuth = np.asarray(az, dtype=np.float64)
        self.range_rate = np.zeros_like(self.range) if rr is None else np.asarray(rr, float)


I2, Z2 = np.eye(2), np.zeros(2)


# ------------------------------------------------------------------- the assignment
def test_assignment_is_global_not_greedy_per_row():
    """Row 0's nearest is column 0, but taking it strands row 1 with nothing cheap."""
    cost = np.array([[1.0, 2.0], [1.1, 50.0]])
    allowed = np.ones((2, 2), dtype=bool)
    assert solve_assignment(cost, allowed) == [(0, 1), (1, 0)]


def test_never_returns_a_forbidden_pair():
    cost = np.array([[1.0, 2.0], [3.0, 4.0]])
    allowed = np.zeros((2, 2), dtype=bool)
    assert solve_assignment(cost, allowed) == []


def test_blocked_cost_exceeds_any_tour_of_allowed_pairs():
    # The optimiser must never buy a blocked cell to complete a larger assignment.
    assert BLOCKED_COST > 1e5
    cost = np.array([[1.0, 1.0], [1.0, 1.0]])
    allowed = np.array([[True, False], [False, False]])
    assert solve_assignment(cost, allowed) == [(0, 0)]


def test_one_detection_cannot_serve_two_tracks():
    tracks = [Trk([60.0, 0.0, 0.0, 0.0]), Trk([60.2, 0.0, 0.0, 0.0])]
    pairs, _ = associate_radar(tracks, Sweep([60.0], [0.0]), I2, Z2, Z2)
    assert len(pairs) <= 1


def test_mahalanobis_handles_a_singular_covariance():
    assert mahalanobis_sq(np.array([1.0, 1.0]), np.zeros((2, 2))) == float("inf")


# ------------------------------------------------------- gating in the native space
def test_azimuth_is_the_discriminating_gate():
    """Same range, 5 deg apart -- far outside the 1 deg azimuth cap."""
    tracks = [Trk([60.0, 0.0, 0.0, 0.0])]
    off = 60.0 * math.tan(math.radians(5.0))
    pairs, _ = associate_radar(tracks, Sweep([60.0], [5.0]), I2, Z2, Z2,
                               max_azimuth_err_deg=1.0)
    assert pairs == []
    assert off > 5.0          # sanity: that is metres of lateral, not a rounding error


def test_a_large_range_disagreement_is_reported_not_gated_away():
    """THE PHASE 0 REGRESSION.

    The shipped node gates on range at +/-3 m, which structurally cannot see the 14-32 m
    road-return failure: it keeps the objects that are already right and drops the broken
    ones, then reports the far field as its best-behaved band. Association here must MATCH a
    badly-ranged object -- on azimuth -- and hand the range residual back as a number.
    """
    tracks = [Trk([100.0, 0.0, 0.0, 0.0], P=np.diag([100.0, 4.0, 100.0, 100.0]))]
    sweep = Sweep([70.0], [0.0])              # radar says 70 m; the track believes 100 m
    pairs, info = associate_radar(tracks, sweep, I2, Z2, Z2,
                                  max_azimuth_err_deg=1.0, max_range_err=60.0)
    assert pairs == [(0, 0)], "a 30 m range error must still associate"
    assert info["d_range"][(0, 0)] == pytest.approx(-30.0, abs=1e-6)


def test_a_three_metre_range_gate_would_have_missed_it():
    """The control for the test above: reproduce the shipped behaviour and watch it fail."""
    tracks = [Trk([100.0, 0.0, 0.0, 0.0], P=np.diag([100.0, 4.0, 100.0, 100.0]))]
    pairs, _ = associate_radar(tracks, Sweep([70.0], [0.0]), I2, Z2, Z2,
                               max_azimuth_err_deg=3.0, max_range_err=3.0)
    assert pairs == [], "this is the defect being regression-tested, not a passing case"


def test_hard_caps_bound_a_diverged_track():
    """A huge P makes S huge, so a purely statistical gate would swallow the sweep."""
    tracks = [Trk([60.0, 0.0, 0.0, 0.0], P=np.eye(4) * 1e6)]
    sweep = Sweep([60.0, 61.0], [0.0, 30.0])
    pairs, _ = associate_radar(tracks, sweep, I2, Z2, Z2, max_azimuth_err_deg=1.0)
    assert all(di == 0 for _, di in pairs), "the 30 deg detection must stay out"


def test_empty_inputs_are_legitimate_not_errors():
    assert associate_radar([], Sweep([], []), I2, Z2, Z2)[0] == []
    assert associate_radar([Trk([60.0, 0.0, 0.0, 0.0])], Sweep([], []), I2, Z2, Z2)[0] == []


# ------------------------------------------------------------------- sticky ids
def _sane(_t, _d):
    return True


def test_sticky_id_overrides_a_geometric_mismatch():
    """The 14 m frame-to-frame jump that breaks a position-only associator.

    ByteTrack's id is stable across it because in the image nothing moved.
    """
    tracks = [Trk([100.0, 0.0, 0.0, 0.0], bid="a"), Trk([86.0, 0.0, 0.0, 0.0], bid="b")]
    dets = [{"id": "a"}, {"id": "b"}]
    geometric = [(0, 1), (1, 0)]              # geometry paired them the wrong way round
    out = apply_sticky_ids(tracks, dets, geometric, sanity_ok=_sane,
                           id_of_track=lambda t: t.bytetrack_id,
                           id_of_detection=lambda d: d["id"])
    assert out == [(0, 0), (1, 1)]


def test_sticky_id_is_refused_when_the_sanity_bound_fails():
    """A recycled id must not teleport a track across the scene."""
    tracks = [Trk([10.0, 0.0, 0.0, 0.0], bid="a")]
    dets = [{"id": "a"}]
    out = apply_sticky_ids(tracks, dets, [], sanity_ok=lambda t, d: False,
                           id_of_track=lambda t: t.bytetrack_id,
                           id_of_detection=lambda d: d["id"])
    assert out == []


def test_detections_without_an_id_fall_through_to_geometry():
    tracks = [Trk([60.0, 0.0, 0.0, 0.0], bid="")]
    out = apply_sticky_ids(tracks, [{"id": ""}], [(0, 0)], sanity_ok=_sane,
                           id_of_track=lambda t: t.bytetrack_id,
                           id_of_detection=lambda d: d["id"])
    assert out == [(0, 0)]
