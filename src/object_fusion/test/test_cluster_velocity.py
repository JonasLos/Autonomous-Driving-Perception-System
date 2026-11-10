"""The windowed LiDAR cluster velocity and the ESR range-rate lag model (2026-11-09)."""
import numpy as np
import pytest

from object_fusion.cluster_velocity import (
    ClusterHistory, OdomFrame, median_slope_velocity, parse_classes, velocity_measurement,
    velocity_observed,
)
from object_fusion.tracker import (
    RADAR_RATE_LAG_SIGMA_FRACTION, kalman_update, radar_h_and_H, radar_rate_lag,
)


def _sweeps(v, n=11, dt=0.1, noise=0.0, seed=1):
    rng = np.random.default_rng(seed)
    t = np.arange(n) * dt
    return np.column_stack((t, v[0] * t + rng.normal(0.0, noise, n) if noise else v[0] * t,
                            v[1] * t + rng.normal(0.0, noise, n) if noise else v[1] * t))


# ------------------------------------------------------------------------------- the fit
def test_a_standing_object_fits_to_zero():
    v = median_slope_velocity(_sweeps((0.0, 0.0), noise=0.1), 1.0, 1.0)
    assert np.linalg.norm(v) < 0.15


def test_constant_velocity_is_recovered():
    assert median_slope_velocity(_sweeps((3.0, -1.0)), 1.0, 1.0) == pytest.approx([3.0, -1.0])


def test_a_single_switch_to_a_neighbouring_cluster_is_ignored():
    """The reason for a median: one sweep on another object must not read as motion."""
    s = _sweeps((0.0, 0.0))
    s[5, 1:] = (2.0, 2.0)
    assert np.linalg.norm(median_slope_velocity(s, 1.0, 1.0)) < 1e-9


def test_too_few_samples_give_no_answer():
    assert median_slope_velocity(_sweeps((0.0, 0.0), n=4), 0.3, 1.0) is None
    assert median_slope_velocity(np.empty((0, 3)), 0.0, 1.0) is None


def test_a_short_span_gives_no_answer():
    """Enough sweeps but bunched at the end of the window: too little baseline."""
    s = np.column_stack((np.linspace(0.5, 1.0, 8), np.zeros(8), np.zeros(8)))
    assert median_slope_velocity(s, 1.0, 1.0) is None


def test_only_the_trailing_window_counts():
    """Old motion must not leak into the present estimate."""
    moving = _sweeps((5.0, 0.0), n=11)
    still = np.column_stack((1.1 + np.arange(11) * 0.1, np.full(11, 5.0), np.zeros(11)))
    v = median_slope_velocity(np.vstack((moving, still)), 2.1, 1.0)
    assert np.linalg.norm(v) < 1e-9


def test_large_absolute_coordinates_do_not_create_a_slope():
    """Raw UTM and epoch stamps read 16 m/s on a parked car in a least-squares fit."""
    s = _sweeps((0.0, 0.0))
    s += [1.79e9, 3.4e6, 7.5e5]
    assert np.linalg.norm(median_slope_velocity(s, s[-1, 0], 1.0)) < 1e-6


# --------------------------------------------------------------------------- the fixed frame
def test_a_world_point_stays_put_while_the_car_drives_and_turns():
    frame, psi, pos = OdomFrame(), 0.0, np.zeros(2)
    world = np.array([20.0, 5.0])
    seen = []
    for k in range(20):
        t0, t1 = k * 0.1, (k + 1) * 0.1
        d_body, dpsi = np.array([1.0, 0.0]), 0.05
        frame.advance(t0, t1, dpsi, d_body)
        c, s = np.cos(psi), np.sin(psi)
        pos = pos + np.array([[c, -s], [s, c]]) @ d_body
        psi += dpsi
        c, s = np.cos(psi), np.sin(psi)
        seen.append(frame.to_fixed(t1, np.array([[c, s], [-s, c]]) @ (world - pos)))
    assert np.ptp(np.array(seen), axis=0) == pytest.approx([0.0, 0.0], abs=1e-9)


def test_a_late_cluster_uses_the_pose_at_its_own_stamp():
    frame = OdomFrame()
    frame.advance(0.0, 1.0, 0.0, (10.0, 0.0))
    assert frame.to_fixed(0.5, (0.0, 0.0)) == pytest.approx((5.0, 0.0))


def test_a_stamp_outside_the_history_is_refused():
    frame = OdomFrame()
    assert frame.to_fixed(0.0, (1.0, 1.0)) is None
    frame.advance(1.0, 2.0, 0.0, (1.0, 0.0))
    assert frame.to_fixed(0.5, (1.0, 1.0)) is None
    assert frame.to_fixed(3.0, (1.0, 1.0)) is None


def test_velocity_returns_to_the_current_sensor_axes():
    """Facing +y in the fixed frame, a fixed-frame +y velocity is straight ahead."""
    frame = OdomFrame()
    frame.advance(0.0, 1.0, np.pi / 2, (0.0, 0.0))
    assert frame.velocity_to_sensor((0.0, 2.0)) == pytest.approx([2.0, 0.0], abs=1e-12)


def test_history_is_time_ordered_and_bounded():
    h = ClusterHistory()
    for k in range(100):
        h.add(float(k), (0.0, 0.0))
    h.add(5.0, (9.0, 9.0))
    a = h.array()
    assert len(h) == 40 and np.all(np.diff(a[:, 0]) > 0) and a[-1, 1] == 0.0


# ----------------------------------------------------------------------- the filter update
def _state(v=(0.6, 0.0)):
    return np.array([10.0, 0.0, *v]), np.eye(4)


def test_velocity_only_leaves_position_alone():
    x, P = _state()
    P[0, 2] = P[2, 0] = 0.5                      # cross-covariance the update would otherwise use
    y, H, R = velocity_measurement(x, (0.0, 0.0), 0.3)
    x2, P2, _, applied = kalman_update(x, P, y, H, R, velocity_only=True)
    assert applied and x2[:2] == pytest.approx(x[:2]) and abs(x2[2]) < abs(x[2])
    assert P2 == pytest.approx(P2.T) and np.all(np.linalg.eigvalsh(P2) > 0)


def test_position_moves_without_the_flag():
    """The control for the test above: the cross-covariance really is there."""
    x, P = _state()
    P[0, 2] = P[2, 0] = 0.5
    y, H, R = velocity_measurement(x, (0.0, 0.0), 0.3)
    assert kalman_update(x, P, y, H, R)[0][0] != pytest.approx(x[0])


# ----------------------------------------------------------------------------- the observer
def test_velocity_observed_needs_a_recent_update():
    assert velocity_observed(10.0, (None, 9.8, None))
    assert not velocity_observed(10.0, (None, 9.0, None))
    assert not velocity_observed(10.0, (None, None, None))


def test_parse_classes():
    assert parse_classes(" person, cyclist ") == {"person", "cyclist"}
    assert parse_classes("") == frozenset()


# ---------------------------------------------------------------------------- the radar lag
R_ID, T0 = np.eye(2), np.zeros(2)


def test_no_lag_is_exactly_the_original_model():
    x = np.array([9.0, -4.0, 0.3, 0.1])
    v_ego = np.array([10.0, 0.0])
    z, H = radar_h_and_H(x, R_ID, T0, v_ego)
    z0, H0 = radar_h_and_H(x, R_ID, T0, v_ego, rate_lag_s=0.0)
    assert np.array_equal(z, z0) and np.array_equal(H, H0)
    assert radar_rate_lag(x, R_ID, T0, v_ego, 0.0) == (0.0, 0.0)


def test_straight_ahead_nothing_changes():
    bias, sigma = radar_rate_lag(np.array([50.0, 0.0, 0.0, 0.0]), R_ID, T0, np.array([10.0, 0.0]), 0.2)
    assert bias == pytest.approx(0.0) and sigma == pytest.approx(0.0)


def test_a_passed_static_object_is_predicted_to_read_as_approaching():
    """Matches the measurement: static target, -0.91 m/s at X 4-8, i.e. about 0.16 s of lag."""
    x = np.array([5.0, -5.0, 0.0, 0.0])
    v_ego = np.array([10.0, 0.0])
    bias, sigma = radar_rate_lag(x, R_ID, T0, v_ego, 0.2)
    az = np.arctan2(-5.0, 5.0)
    expected = 0.2 * 100.0 * np.sin(az) ** 2 / np.hypot(5.0, 5.0)
    assert bias == pytest.approx(-expected) and bias < 0.0
    assert sigma == pytest.approx(RADAR_RATE_LAG_SIGMA_FRACTION * expected)


def test_lag_moves_only_the_range_rate_prediction():
    x = np.array([5.0, -5.0, 0.0, 0.0])
    v_ego = np.array([10.0, 0.0])
    z0, H0 = radar_h_and_H(x, R_ID, T0, v_ego)
    z1, H1 = radar_h_and_H(x, R_ID, T0, v_ego, rate_lag_s=0.2)
    assert z1[1] < z0[1] and z1[[0, 2]] == pytest.approx(z0[[0, 2]])
    assert np.array_equal(H0, H1)


def test_lag_uses_relative_velocity_so_it_holds_for_a_moving_target():
    """A target pacing the car has no relative motion, so the range-rate is not changing."""
    bias, _ = radar_rate_lag(np.array([5.0, -5.0, 10.0, 0.0]), R_ID, T0, np.array([10.0, 0.0]), 0.2)
    assert bias == pytest.approx(0.0)
