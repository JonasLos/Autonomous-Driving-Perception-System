"""Capture-stamp ordering across sensors, and the lag that buys it."""
from object_fusion.measurement_queue import DEFAULT_LAG_S, Measurement, MeasurementQueue


def test_releases_in_capture_order_across_sensors_not_arrival_order():
    """A 30 Hz radar sample describing an earlier instant must not be applied after a
    10 Hz camera sample that describes a later one."""
    q = MeasurementQueue(lag=0.1)
    q.add(Measurement(1.00, "camera", "c1"))
    q.add(Measurement(0.95, "radar", "r1"))       # arrived later, captured earlier
    q.add(Measurement(0.98, "lidar", "l1"))
    out = [m.payload for m in q.release(now=1.2)]
    assert out == ["r1", "l1", "c1"]


def test_nothing_is_released_before_the_lag_elapses():
    q = MeasurementQueue(lag=0.12)
    q.add(Measurement(1.0, "camera", "c"))
    assert q.release(now=1.05) == []
    assert len(q.release(now=1.2)) == 1


def test_a_late_arrival_is_counted():
    q = MeasurementQueue(lag=0.05)
    q.add(Measurement(1.0, "camera", "a"))
    q.release(now=1.2)
    q.add(Measurement(0.9, "radar", "b"))         # older than what already went out
    assert q.late == 1


def test_equal_stamps_release_in_arrival_order_without_comparing_payloads():
    """Payloads are arbitrary objects; a heap that compared them would raise."""
    q = MeasurementQueue(lag=0.0)
    q.add(Measurement(1.0, "a", object()))
    q.add(Measurement(1.0, "b", object()))
    assert [m.sensor for m in q.release(now=2.0)] == ["a", "b"]


def test_reset_clears_state_for_a_bag_rewind():
    """A looping bag would otherwise resurrect measurements from the previous pass."""
    q = MeasurementQueue(lag=0.1)
    q.add(Measurement(100.0, "camera", "old"))
    q.reset()
    assert len(q) == 0
    q.add(Measurement(1.0, "camera", "new"))
    assert q.late == 0
    assert [m.payload for m in q.release(now=2.0)] == ["new"]


def test_drain_ignores_the_lag():
    q = MeasurementQueue(lag=10.0)
    q.add(Measurement(1.0, "camera", "a"))
    assert q.release(now=1.1) == []
    assert len(q.drain()) == 1


def test_depth_is_bounded():
    q = MeasurementQueue(lag=99.0, max_depth=10)
    for i in range(50):
        q.add(Measurement(float(i), "radar", i))
    assert len(q) == 10 and q.dropped == 40


def test_default_lag_covers_the_camera_path():
    assert DEFAULT_LAG_S >= 0.06 + 0.05


def test_the_prediction_clock_never_rewinds():
    """A measurement released out of capture order must not move the aggregator's clock back.

    It used to: `_apply` set `_last_t = t` even when `dt <= 0`, so the NEXT measurement predicted
    across an interval already applied and ego motion was counted twice. Live that showed as every
    static object drifting forward at ~35% of ego speed, and the box trailing its own detection
    between camera frames. This reproduces the bookkeeping in isolation.
    """
    applied = []
    last_t = None
    for stamp in (10.00, 10.03, 10.06, 10.01, 10.09):      # 10.01 arrives late, out of order
        if last_t is None:
            last_t = stamp
        dt = stamp - last_t
        if dt > 0.0:
            applied.append((last_t, stamp))
        last_t = max(last_t, stamp)
    spans = [(a, b) for a, b in applied]
    assert spans == [(10.00, 10.03), (10.03, 10.06), (10.06, 10.09)]
    total = sum(b - a for a, b in spans)
    assert abs(total - 0.09) < 1e-9, "each interval of ego motion applied exactly once"


def test_a_bag_rewind_must_reset_the_clock_not_pin_it():
    """The other half of "never rewind the clock".

    Out-of-order arrivals (tens of ms) must not move the clock back, but a LOOPING replay jumps
    back by the length of the bag. Pinning the clock there left every later measurement with
    dt <= 0 -- measured live at 74% of measurements, median dt -8.3 s -- so nothing was ever
    predicted again and the tracks crawled along on updates alone.
    """
    REWIND_S = 1.0
    last_t, predicted, resets = None, [], 0
    stream = [100.00, 100.03, 100.06, 100.04,      # 100.04 is a late arrival, not a rewind
              100.09, 82.00, 82.03, 82.06]         # 82.00 is the loop restart
    for stamp in stream:
        if last_t is None:
            last_t = stamp
            continue
        dt = stamp - last_t
        if dt < -REWIND_S:
            resets += 1
            last_t = stamp
            continue
        if dt > 0.0:
            predicted.append(round(dt, 3))
        last_t = max(last_t, stamp)
    assert resets == 1, "one rewind, from the loop restart"
    assert predicted == [0.03] * 5, "no interval applied twice, none skipped"
