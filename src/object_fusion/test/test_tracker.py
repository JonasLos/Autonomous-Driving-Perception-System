"""The estimator: sign conventions, Jacobians, numerics, and the two central claims."""
import math

import numpy as np
import pytest

from object_fusion.tracker import (
    RANGE_TRUST_MAX_M, compensated_range_rate, init_from_radar, kalman_update,
    lidar_measurement, predict, process_noise, radar_R, radar_h_and_H,
    range_is_trustworthy, sigma_along, sigma_cross, wrap_deg,
)


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


# --------------------------------------------------------------- sign conventions
@pytest.mark.parametrize("az_deg", [-45, -30, -10, 0, 10, 30, 45])
def test_static_target_range_rate_matches_the_verified_closed_form(az_deg):
    """rr = -v*cos(a) - omega*L*sin(a) -- BOTH terms verified on the 2026-08-25 bag.

    The first term is the 98.6%-agreement result; the second is the odd-in-azimuth lever-arm
    term that settled the azimuth sign, because cos is even and could not. An implementation
    that disagrees here has a sign bug, so this is the anchor test for the whole module.
    """
    v, w, L, rho = 15.0, 0.15, 2.915, 60.0
    a = math.radians(az_deg)
    x = np.array([rho * math.cos(a), rho * math.sin(a), 0.0, 0.0])   # static over ground
    z, _ = radar_h_and_H(x, np.eye(2), np.zeros(2), np.array([v, w * L]))
    assert z[1] == pytest.approx(-v * math.cos(a) - w * L * math.sin(a), abs=1e-12)


def test_positive_azimuth_is_left():
    z, _ = radar_h_and_H(np.array([10.0, 10.0, 0.0, 0.0]), np.eye(2), np.zeros(2), np.zeros(2))
    assert z[2] == pytest.approx(45.0)


def test_range_rate_positive_is_receding():
    # Target ahead moving away faster than a stationary ego.
    z, _ = radar_h_and_H(np.array([50.0, 0.0, 5.0, 0.0]), np.eye(2), np.zeros(2), np.zeros(2))
    assert z[1] > 0.0


def test_compensated_range_rate_is_zero_for_a_stationary_target():
    """The discriminant the whole radar-only clutter strategy rests on."""
    v_ego = np.array([15.0, 0.0])
    for az in (-40.0, -10.0, 0.0, 10.0, 40.0):
        rr = -15.0 * math.cos(math.radians(az))
        assert abs(float(compensated_range_rate(rr, az, v_ego))) < 1e-9


# ------------------------------------------------------------------------ Jacobian
def test_radar_jacobian_matches_finite_differences():
    rng = np.random.default_rng(11)
    R_sl, t_sl = rot(math.radians(5.443)), np.array([-2.964, 0.371])
    worst = 0.0
    for _ in range(120):
        x = np.array([rng.uniform(5, 120), rng.uniform(-30, 30),
                      rng.uniform(-25, 25), rng.uniform(-25, 25)])
        ve = np.array([rng.uniform(0, 20), rng.uniform(-1, 1)])
        _, H = radar_h_and_H(x, R_sl, t_sl, ve)
        for j in range(4):
            h = 1e-6 * max(1.0, abs(x[j]))
            e = np.zeros(4); e[j] = h
            zp, _ = radar_h_and_H(x + e, R_sl, t_sl, ve)
            zm, _ = radar_h_and_H(x - e, R_sl, t_sl, ve)
            worst = max(worst, float(np.max(np.abs((zp - zm) / (2 * h) - H[:, j]))))
    assert worst < 1e-5


# ----------------------------------------------------------------------- numerics
def test_joseph_form_stays_symmetric_and_positive_definite():
    """Thousands of small sequential updates is the regime that breaks (I-KH)P."""
    x = np.array([50.0, 2.0, 10.0, 0.0])
    P = np.diag([4.0, 4.0, 400.0, 400.0])
    Q = process_noise(0.033, 2.0, 1.0)
    R_sl, t_sl = rot(math.radians(5.443)), np.array([-2.964, 0.371])
    for _ in range(5000):
        x, P = predict(x, P, 0.033, 0.0, np.array([0.5, 0.0]), Q)
        _, H = radar_h_and_H(x, R_sl, t_sl, np.array([15.0, 0.0]))
        x, P, _, _ = kalman_update(x, P, np.array([0.05, -0.02, 0.1]), H, radar_R())
    assert np.max(np.abs(P - P.T)) < 1e-9
    assert np.linalg.eigvalsh(P).min() > 0.0


def test_gate_rejects_rather_than_clamps():
    x = np.array([50.0, 0.0, 0.0, 0.0])
    P = np.eye(4) * 0.01
    H = np.zeros((2, 4)); H[:2, :2] = np.eye(2)
    x2, P2, nis, applied = kalman_update(x, P, np.array([50.0, 50.0]), H, np.eye(2) * 0.01,
                                         gate_chi2=9.21)
    assert not applied and nis > 9.21
    assert np.array_equal(x, x2) and np.array_equal(P, P2)


# ------------------------------------------------------------------- the two claims
def test_radar_corrects_range_without_moving_lateral_position():
    """THE central claim of the polar measurement model.

    A track sitting 10 m short in range, with radar seeing the true range, must be pulled
    along the ray and essentially not sideways -- because the radar's azimuth is the weak
    channel and must not be allowed to act like a position fix.
    """
    truth_r = 80.0
    x = np.array([70.0, 0.0, 0.0, 0.0])            # 10 m short, dead ahead
    P = np.diag([25.0, 25.0, 100.0, 100.0])
    lateral_before = x[1]
    z_pred, H = radar_h_and_H(x, np.eye(2), np.zeros(2), np.zeros(2))
    y = np.array([truth_r - z_pred[0], 0.0, wrap_deg(0.0 - z_pred[2])])
    x2, _, _, applied = kalman_update(x, P, y, H, radar_R())
    assert applied
    assert x2[0] > 78.0, "range must be pulled most of the way to the radar's value"
    assert abs(x2[1] - lateral_before) < 0.05, "lateral position must barely move"


def test_track_survives_a_total_camera_dropout_on_radar_alone():
    """A constant-velocity target, camera gone, radar only -- range must stay locked."""
    dt, v_t = 0.05, 20.0
    pos = 60.0
    x, P = np.array([60.0, 0.0, v_t, 0.0]), np.diag([4.0, 4.0, 25.0, 25.0])
    Q = process_noise(dt, 2.0, 1.0)
    for _ in range(60):                       # 3 s with no camera at all
        pos += v_t * dt
        x, P = predict(x, P, dt, 0.0, np.zeros(2), Q)
        z_pred, H = radar_h_and_H(x, np.eye(2), np.zeros(2), np.zeros(2))
        y = np.array([pos - z_pred[0], v_t - z_pred[1], wrap_deg(0.0 - z_pred[2])])
        x, P, _, _ = kalman_update(x, P, y, H, radar_R())
    assert abs(x[0] - pos) < 0.5
    assert abs(x[2] - v_t) < 1.0


def test_a_world_stationary_object_reads_as_stationary_under_ego_motion():
    """With the ego moving, a fixed object's ground-referenced velocity must stay ~0.

    This is what the radar-only birth gate depends on. If ego motion leaks into track
    velocity, static clutter clears the 1.5 m/s threshold and the whole strategy fails open.
    """
    dt, v_ego = 0.05, 15.0
    x = np.array([80.0, 3.0, 0.0, 0.0])
    P = np.diag([1.0, 1.0, 4.0, 4.0])
    Q = process_noise(dt, 0.5, 0.5)
    world = np.array([80.0, 3.0])
    for _ in range(100):                      # 5 s of driving straight at the object
        world = world - np.array([v_ego * dt, 0.0])
        x, P = predict(x, P, dt, 0.0, np.array([v_ego * dt, 0.0]), Q)
        H = np.zeros((2, 4)); H[:2, :2] = np.eye(2)
        x, P, _, _ = kalman_update(x, P, world - x[:2], H, np.diag([0.04, 0.04]))
    assert np.hypot(x[2], x[3]) < 0.5, "a static object must not acquire ego's speed"


# ------------------------------------------------------- the far-field range policy
def test_range_trust_bound_matches_the_measurement():
    assert RANGE_TRUST_MAX_M == 80.0
    assert range_is_trustworthy(60.0) and not range_is_trustworthy(120.0)


def test_dropping_range_yields_a_one_dimensional_lateral_measurement():
    p = np.array([100.0, 0.0])
    y, H, R = lidar_measurement(p, np.array([70.0, 2.0]), drop_range=True)
    assert y.shape == (1,) and H.shape == (1, 4) and R.shape == (1, 1)
    # A 30 m range error must contribute NOTHING; only the 2 m lateral offset survives.
    assert abs(y[0] - 2.0) < 1e-9


def test_inflating_absorbs_a_sustained_bias_while_dropping_does_not():
    """Why the far-field rule is a DROP and not an inflation.

    A single inflated update barely moves (0.3 m here), which is exactly what makes this
    failure mode hard to spot. The problem is that the road-return error is a BIAS held over
    many frames, so those small pulls compound -- the filter walks toward the wrong range and
    then reads the entry and exit of the biased stretch as velocity spikes. Dropping the
    along-ray component absorbs none of it, at any duration.
    """
    z = np.array([70.0, 0.0])                  # 30 m short, sustained: the far-field failure
    Q = process_noise(0.1, 2.0, 1.0)

    x_d, P_d = np.array([100.0, 0.0, 0.0, 0.0]), np.diag([9.0, 9.0, 100.0, 100.0])
    x_i, P_i = x_d.copy(), P_d.copy()
    for _ in range(30):                        # 3 s at 10 Hz
        x_d, P_d = predict(x_d, P_d, 0.1, 0.0, np.zeros(2), Q)
        y, H, R = lidar_measurement(x_d[:2], z, drop_range=True)
        x_d, P_d, _, _ = kalman_update(x_d, P_d, y, H, R)

        x_i, P_i = predict(x_i, P_i, 0.1, 0.0, np.zeros(2), Q)
        y2, H2, R2 = lidar_measurement(x_i[:2], z, sigma_a=30.0, drop_range=False)
        x_i, P_i, _, _ = kalman_update(x_i, P_i, y2, H2, R2)

    assert abs(x_d[0] - 100.0) < 1e-6, "dropping must never move range, at any duration"
    assert x_i[0] < 90.0, "an inflated R walks several metres into the bias over 3 s"
    # And the spurious velocity that walk manufactures is the reason it is worse than the
    # static error it replaces: a planner reacts to velocity.
    assert abs(x_i[2]) > abs(x_d[2]) + 1.0


def test_sigma_along_is_interpolated_from_the_measured_table():
    assert sigma_along(30.0) == pytest.approx(0.64, abs=1e-9)      # re-measured 2026-09-15
    assert sigma_along(90.0) == pytest.approx(6.82, abs=1e-9)
    assert sigma_along(35.0) == pytest.approx(0.865, abs=1e-3)     # between 30 and 40
    assert sigma_along(1.0) >= 0.35                                # floor holds near field
    assert sigma_cross(80.0) > sigma_cross(20.0)


# ----------------------------------------------------------------------------- birth
def test_radar_birth_seeds_radial_velocity_and_a_banana_covariance():
    """Birth in polar: thin in range, wide across it. An isotropic blob would be wrong."""
    x, P = init_from_radar(80.0, 0.0, -20.0, np.zeros(2), np.eye(2), np.zeros(2))
    assert x[0] == pytest.approx(80.0)
    assert x[2] == pytest.approx(-20.0), "closing target -> negative x velocity"
    assert P[0, 0] < P[1, 1], "range better known than azimuth at 80 m"
    assert P[2, 2] < P[3, 3], "radial speed measured, cross-ray speed unknown"


def test_radar_birth_removes_ego_motion_from_the_seeded_velocity():
    """A static target seen from a moving ego must be born with ~zero ground velocity."""
    v_ego = np.array([15.0, 0.0])
    x, _ = init_from_radar(60.0, 0.0, -15.0, v_ego, np.eye(2), np.zeros(2))
    assert abs(x[2]) < 1e-9


# ------------------------------------------------------- measured tuning (Phase 2)
def test_camera_gate_rejects_the_outlier_population_rather_than_inflating():
    """Why CAMERA_GATE_CHI2 exists instead of a larger sigma_along.

    The along-ray error is a mixture, so no single Gaussian fits both the core and the tail.
    A road-adopted measurement must be REJECTED (leaving the good ones at full weight), not
    absorbed by an R inflated to swallow it.
    """
    from object_fusion.tracker import CAMERA_GATE_CHI2

    x = np.array([50.0, 0.0, 0.0, 0.0])
    P = np.diag([1.0, 1.0, 25.0, 25.0])
    good = np.array([50.3, 0.1])
    outlier = np.array([36.0, 0.0])          # 14 m short: the road-adoption signature

    y, H, R = lidar_measurement(x[:2], good)
    _, _, _, applied_good = kalman_update(x, P, y, H, R, gate_chi2=CAMERA_GATE_CHI2)
    y2, H2, R2 = lidar_measurement(x[:2], outlier)
    x_out, _, _, applied_bad = kalman_update(x, P, y2, H2, R2, gate_chi2=CAMERA_GATE_CHI2)

    assert applied_good, "a good measurement must pass the gate"
    assert not applied_bad, "a 14 m road-adopted measurement must be rejected"
    assert np.array_equal(x, x_out), "a rejected measurement must not move the state at all"


def test_radar_gate_is_looser_because_its_model_is_calibrated():
    """Measured: radar NIS sits at 94.5-96.9% under-gate in every band, 0-180 m."""
    from object_fusion.tracker import CAMERA_GATE_CHI2, RADAR_GATE_CHI2
    assert RADAR_GATE_CHI2 > CAMERA_GATE_CHI2


# ------------------------------------------------- the gate must not lock a track out
def test_the_gate_releases_after_consecutive_rejections():
    """Measured lockout: 719 rejections in only 181 runs, one of them 67 long.

    A rejected measurement does not update the track, so the state drifts further and the next
    innovation is larger -- self-reinforcing. Below the threshold the gate behaves normally;
    at it, the update is forced through regardless of how large the innovation is.
    """
    from object_fusion.tracker import MAX_CONSECUTIVE_REJECTS, gated_update

    x = np.array([50.0, 0.0, 0.0, 0.0])
    P = np.diag([1.0, 1.0, 25.0, 25.0])
    far = np.array([70.0, 0.0])                    # 20 m out: far outside any sane gate
    y, H, R = lidar_measurement(x[:2], far)

    _, _, _, applied, forced = gated_update(x, P, y, H, R, gate_chi2=9.21,
                                            consecutive_rejects=0)
    assert not applied and not forced, "an ordinary outlier is still rejected"

    x2, P2, _, applied2, forced2 = gated_update(
        x, P, y, H, R, gate_chi2=9.21, consecutive_rejects=MAX_CONSECUTIVE_REJECTS)
    assert applied2 and forced2, "at the threshold the escape must fire"
    assert x2[0] > x[0] + 5.0, "and it must actually move the state, not nudge it"


def test_the_forced_update_inflates_covariance_so_it_can_reconverge():
    """Without inflation the gain is tiny on a confident-but-wrong track, and it re-locks."""
    from object_fusion.tracker import MAX_CONSECUTIVE_REJECTS, gated_update

    x = np.array([50.0, 0.0, 0.0, 0.0])
    P = np.diag([0.01, 0.01, 25.0, 25.0])          # very confident, and wrong
    y, H, R = lidar_measurement(x[:2], np.array([70.0, 0.0]))
    x_no, _, _, _ = kalman_update(x, P, y, H, R)   # ungated, no inflation
    x_esc, _, _, _, _ = gated_update(x, P, y, H, R, gate_chi2=9.21,
                                     consecutive_rejects=MAX_CONSECUTIVE_REJECTS)
    assert x_esc[0] > x_no[0], "the escape must pull harder than a plain ungated update"
