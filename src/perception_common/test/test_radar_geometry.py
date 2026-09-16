"""Unit tests for the radar polar geometry and association rule.

Pure pytest, no ROS context -- radar_ros.radar_geometry imports nothing but numpy for
exactly this reason. The association behaviour asserted here (separate range and azimuth
gates rather than one Euclidean radius) is the part that is hard to check on a bag, because
the far-field cases that distinguish the two rules are rare in any single recording.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "radar_ros"))

from radar_ros.radar_geometry import (  # noqa: E402
    STATUS_NO_TARGET,
    associate,
    cartesian_to_polar,
    gate_tracks,
    polar_to_cartesian,
    radial_velocity_vector,
)


# --------------------------------------------------------------------- polar <-> cartesian


def test_boresight_is_pure_x():
    x, y = polar_to_cartesian([25.0], [0.0])
    assert x[0] == pytest.approx(25.0)
    assert y[0] == pytest.approx(0.0)


def test_positive_angle_is_to_the_left():
    """+angle must put the target at +y, i.e. left, per REP-103 with x forward.

    This is the convention the whole association depends on: get it backwards and every
    object lands mirrored about the boresight, which at small angles looks like a plausible
    association error rather than a sign bug.
    """
    _, y = polar_to_cartesian([10.0], [30.0])
    assert y[0] > 0.0


def test_roundtrip_through_cartesian():
    rng_in, az_in = np.array([5.0, 60.0, 174.0]), np.array([-40.0, 0.0, 12.25])
    rng, az = cartesian_to_polar(*polar_to_cartesian(rng_in, az_in))
    assert rng == pytest.approx(rng_in)
    assert az == pytest.approx(az_in)


def test_esr_angle_extremes_survive_roundtrip():
    """+/-51.2 deg is the full encodable range of the ESR angle field."""
    rng, az = cartesian_to_polar(*polar_to_cartesian([100.0, 100.0], [-51.2, 51.2]))
    assert az == pytest.approx([-51.2, 51.2])
    assert rng == pytest.approx([100.0, 100.0])


# ----------------------------------------------------------------------------- gating

def _tracks(n=4, **over):
    d = dict(
        ranges=np.full(n, 50.0),
        angles=np.zeros(n),
        amplitudes=np.full(n, 10.0),
        statuses=np.ones(n, dtype=int),
        update_counts=np.full(n, 5, dtype=int),
    )
    d.update(over)
    return d


_LIMITS = dict(min_range=1.0, max_range=175.0, min_amplitude=0.0, min_update_count=1)


def test_empty_slots_are_dropped():
    """An unfilled ESR slot reports range 0 with status 0.

    It must not survive: a zero-range track sits at the sensor origin and would win the
    nearest-object association on every frame.
    """
    t = _tracks(4)
    t["ranges"][:2] = 0.0
    t["statuses"][:2] = STATUS_NO_TARGET
    keep = gate_tracks(**t, **_LIMITS)
    assert keep.tolist() == [False, False, True, True]


def test_zero_range_dropped_even_with_valid_status():
    t = _tracks(2, ranges=np.array([0.0, 50.0]))
    assert gate_tracks(**t, **_LIMITS).tolist() == [False, True]


def test_range_bounds_are_inclusive():
    t = _tracks(2, ranges=np.array([1.0, 175.0]))
    assert gate_tracks(**t, **_LIMITS).tolist() == [True, True]


def test_amplitude_and_update_count_filters():
    """The filter mechanism itself works -- see below for why neither is used by default."""
    t = _tracks(3, amplitudes=np.array([-5.0, 10.0, 10.0]),
                update_counts=np.array([5, 0, 5]))
    keep = gate_tracks(**t, **{**_LIMITS, "min_amplitude": 0.0, "min_update_count": 1})
    assert keep.tolist() == [False, False, True]


def test_positive_min_update_count_drops_everything_on_this_driver():
    """update_count is a stub: 0 on every track in every sweep of the 2026-08-25 bag.

    This is the regression guard for a real bug -- min_update_count originally defaulted to
    1 as a "cheapest clutter filter", which silently discarded 100% of radar tracks. The
    field is documented upstream as a rolling-count bit and this driver never decodes it.
    """
    t = _tracks(4, update_counts=np.zeros(4, dtype=int))
    assert gate_tracks(**t, **{**_LIMITS, "min_update_count": 1}).sum() == 0
    assert gate_tracks(**t, **{**_LIMITS, "min_update_count": 0}).sum() == 4


def test_amplitude_floor_is_not_a_weak_return():
    """-10 is the sensor's amplitude floor and ~30% of real tracks sit exactly on it.

    A min_amplitude of 0.0 looks harmless but drops ~72% of tracks (10.32 -> 2.86 per sweep
    measured). The default must stay below the floor.
    """
    t = _tracks(4, amplitudes=np.array([-10.0, -10.0, 3.0, 12.0]))
    assert gate_tracks(**t, **{**_LIMITS, "min_amplitude": -1e9}).sum() == 4
    assert gate_tracks(**t, **{**_LIMITS, "min_amplitude": 0.0}).sum() == 2


def test_non_finite_is_dropped():
    t = _tracks(2, ranges=np.array([np.nan, 50.0]))
    assert gate_tracks(**t, **_LIMITS).tolist() == [False, True]


# ------------------------------------------------------------------------ association

GATES = dict(max_range_err=3.0, max_azimuth_err_deg=3.0)


def test_exact_match_pairs_up():
    assert associate([50.0], [10.0], [50.0], [10.0], **GATES) == [(0, 0)]


def test_empty_inputs_are_not_an_error():
    assert associate([], [], [50.0], [0.0], **GATES) == []
    assert associate([50.0], [0.0], [], [], **GATES) == []


def test_out_of_gate_is_rejected_not_snapped():
    """Beyond the gate there must be NO pair, rather than the least-bad one."""
    assert associate([50.0], [0.0], [70.0], [0.0], **GATES) == []
    assert associate([50.0], [0.0], [50.0], [45.0], **GATES) == []


def test_assignment_is_global_not_greedy_per_row():
    """Two objects and two tracks, cross-matched: each must take the right one."""
    pairs = associate([50.0, 80.0], [0.0, 5.0], [80.1, 49.9], [5.1, 0.1], **GATES)
    assert pairs == [(0, 1), (1, 0)]


def test_one_track_cannot_serve_two_objects():
    pairs = associate([50.0, 51.0], [0.0, 0.5], [50.2], [0.1], **GATES)
    assert len(pairs) == 1


def test_far_field_lateral_offset_a_euclidean_gate_would_reject():
    """The case that motivates gating in polar rather than Cartesian.

    At 100 m a 2 deg azimuth error is ~3.5 m of lateral separation -- beyond any Euclidean
    radius tight enough to be safe in the near field, yet well inside the sensor's real
    angular uncertainty. It must match.
    """
    pairs = associate([100.0], [0.0], [100.0], [2.0], **GATES)
    assert pairs == [(0, 0)]

    x0, y0 = polar_to_cartesian([100.0], [0.0])
    x1, y1 = polar_to_cartesian([100.0], [2.0])
    assert math.hypot(x1[0] - x0[0], y1[0] - y0[0]) > 3.0


def test_near_field_pair_a_euclidean_gate_would_wrongly_accept():
    """The converse: 3.5 m apart at 10 m range is a different object, and 20 deg of
    azimuth says so even though the Cartesian distance is the same as the case above."""
    assert associate([10.0], [0.0], [10.0], [20.0], **GATES) == []


def test_degenerate_gates_match_nothing():
    assert associate([50.0], [0.0], [50.0], [0.0], max_range_err=0.0,
                     max_azimuth_err_deg=3.0) == []


# --------------------------------------------------------------------------- velocity


def test_radial_velocity_is_along_the_ray():
    vx, vy = radial_velocity_vector(10.0, 0.0)
    assert vx == pytest.approx(10.0)
    assert vy == pytest.approx(0.0)


def test_closing_target_points_back_at_the_sensor():
    """range_rate is positive when receding, so a closing target must be negative x."""
    vx, _ = radial_velocity_vector(-15.0, 0.0)
    assert vx < 0.0


def test_crossing_target_has_near_zero_radial_component():
    """A target at 90 deg contributes nothing to x -- the documented blind spot of a
    radial-only velocity, asserted so nobody later reads this as a full velocity vector."""
    vx, vy = radial_velocity_vector(20.0, 90.0)
    assert vx == pytest.approx(0.0, abs=1e-9)
    assert vy == pytest.approx(20.0)
