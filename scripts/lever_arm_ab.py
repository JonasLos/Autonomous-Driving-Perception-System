#!/usr/bin/env python3
"""Where is the LiDAR relative to the odometry position, and is the range biased? (to-do 13)

Both errors move a static object's computed world position, and on a single pass they look the
same, which is why the first attempt at this (opposed-heading pairs in planner_objects_ab.py)
could not separate them. They differ in BEARING:

    a lever-arm error   shifts every object by the same vector in the VEHICLE frame
    a radial range bias shifts each object along ITS OWN ray from the sensor
    an ego-yaw error    shifts each object across its ray, by yaw_error * range

A static object watched from far ahead to close beside the car spans a wide bearing range, so all
three are separable from one drive. For observation i of static track k:

    p_world_i = p_k + R(psi_i) @ ( -de + b * u_i + dpsi * r_i * perp(u_i) )

with p_k the object's true position (a nuisance parameter, eliminated by demeaning per track),
psi_i the vehicle heading, u_i the unit ray from the LiDAR to the object in vehicle axes, r_i the
range, de the lever-arm error (what to add to lidar_offset), b the radial bias (positive = the
stack places objects too far away) and dpsi the residual ego-yaw error. Linear in all four, so it
is one least-squares solve.

    python3 scripts/lever_arm_ab.py ~/fusion_data/recordings/planner_ab

Reads /perception/objects (ego frame, capture-stamped) and /novatel/oem7/odom_grid, so it needs a
recording that has both -- scripts/run_planner_ab_nodes.sh makes one.
"""

import argparse
import bisect
import glob
import math
import os
from collections import defaultdict

import numpy as np
from mcap_ros2.reader import read_ros2_messages

OBJECTS = "/perception/objects"
ODOM = "/novatel/oem7/odom_grid"

#: What fusion_bridge_core uses today, and what this measures a correction to.
NOMINAL_OFFSET = (1.5055 + 0.887406, 0.206310)


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def load(bag):
    files = sorted(glob.glob(os.path.join(bag, "*.mcap"))) if os.path.isdir(bag) else [bag]
    odom, objs = [], []
    for f in files:
        for m in read_ros2_messages(f, topics=[ODOM, OBJECTS]):
            msg = m.ros_msg
            if m.channel.topic == ODOM:
                q = msg.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                odom.append((stamp(msg.header), msg.pose.pose.position.x,
                             msg.pose.pose.position.y, yaw))
            else:
                rows = [(o.track_id, o.pose.position.x, o.pose.position.y,
                         math.hypot(o.velocity.x, o.velocity.y) if o.velocity_valid else 0.0,
                         o.track_status) for o in msg.objects]
                if rows:
                    objs.append((stamp(msg.header), rows))
    odom.sort()
    objs.sort(key=lambda r: r[0])
    return odom, objs


def pose_at(ts, poses, t, max_gap=0.05):
    i = bisect.bisect_left(ts, t)
    if i == 0 or i >= len(ts):
        return None
    t0, t1 = ts[i - 1], ts[i]
    if t1 - t0 > 0.2 or not (t0 - max_gap <= t <= t1 + max_gap):
        return None
    w = (t - t0) / (t1 - t0)
    x0, y0, a0 = poses[i - 1]
    x1, y1, a1 = poses[i]
    da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
    return x0 + w * (x1 - x0), y0 + w * (y1 - y0), a0 + w * da


def fit(rows_A, rows_y, used):
    A = np.concatenate(rows_A)
    y = np.concatenate(rows_y)
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = y - A @ sol
    return sol, A, y, resid


def bootstrap(rows_A, rows_y, n=200, seed=0):
    """Resample whole TRACKS. Observations inside a track are the same object over consecutive
    frames -- treating them as independent understates the error by an order of magnitude."""
    rng = np.random.default_rng(seed)
    k = len(rows_A)
    out = []
    for _ in range(n):
        idx = rng.integers(0, k, k)
        try:
            sol, *_ = fit([rows_A[i] for i in idx], [rows_y[i] for i in idx], k)
        except np.linalg.LinAlgError:
            continue
        out.append(sol)
    return np.array(out)


def selftest():
    """Recover known values from synthetic passes, so a null result on real data can be trusted
    as being about the data rather than the algebra."""
    rng = np.random.default_rng(7)
    truth = dict(de=np.array([-1.70, 0.30]), b=0.25, dpsi=math.radians(0.2))
    for span_deg, label in ((20.0, "narrow (like the replay)"), (70.0, "wide (a close pass)")):
        rows_A, rows_y = [], []
        for k in range(40):
            p_true = rng.uniform(-100, 100, 2) + np.array([500.0, 500.0])
            A_k, y_k = [], []
            n = 30
            for i in range(n):
                psi = rng.uniform(-math.pi, math.pi)
                bearing = math.radians(rng.uniform(0, span_deg) - span_deg / 2)
                r = rng.uniform(10, 60)
                u = np.array([math.cos(bearing), math.sin(bearing)])
                perp = np.array([-u[1], u[0]])
                c, s_ = math.cos(psi), math.sin(psi)
                R = np.array([[c, -s_], [s_, c]])
                # what the stack would compute, given the truth plus noise
                err = -truth["de"] + truth["b"] * u + truth["dpsi"] * r * perp
                p_world = p_true + R @ err + rng.normal(0, 0.25, 2)
                A_k.append(np.column_stack([-R, (R @ u).reshape(2, 1),
                                            (R @ perp * r).reshape(2, 1)]))
                y_k.append(p_world)
            A_k, y_k = np.concatenate(A_k), np.concatenate(y_k)
            for start in (0, 1):
                idx = np.arange(start, 2 * n, 2)
                A_k[idx] -= A_k[idx].mean(axis=0)
                y_k[idx] -= y_k[idx].mean()
            rows_A.append(A_k)
            rows_y.append(y_k)
        sol, A, _, _ = fit(rows_A, rows_y, len(rows_A))
        cond = np.linalg.cond(A.T @ A)
        print(f"  {label:26s} de=({sol[0]:+.2f}, {sol[1]:+.2f})  b={sol[2]:+.2f}  "
              f"dpsi={math.degrees(sol[3]):+.3f} deg   cond={cond:.0f}")
    print(f"  truth                      de=({truth['de'][0]:+.2f}, {truth['de'][1]:+.2f})  "
          f"b={truth['b']:+.2f}  dpsi={math.degrees(truth['dpsi']):+.3f} deg")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--max-speed", type=float, default=0.5,
                    help="a track above this is not static and is skipped")
    ap.add_argument("--min-obs", type=int, default=15)
    ap.add_argument("--min-bearing-span", type=float, default=15.0,
                    help="degrees of bearing a track must span for it to separate the terms")
    ap.add_argument("--max-range", type=float, default=80.0,
                    help="past the range-trust bound the range is biased in its own way")
    ap.add_argument("--selftest", action="store_true",
                    help="recover known values from synthetic data and exit")
    ap.add_argument("--profile", action="store_true",
                    help="sweep lidar_offset_x alone and print how well each value explains the "
                         "observed positions (model-free, and shows whether the drive can tell)")
    args = ap.parse_args()

    if args.selftest:
        print("=== SELF-TEST: can this estimator recover a known lever arm? ===")
        selftest()
        return

    odom, objs = load(args.bag)
    ts = [o[0] for o in odom]
    poses = [(o[1], o[2], o[3]) for o in odom]
    print(f"[load] odom {len(odom)}  object messages {len(objs)}")

    tracks = defaultdict(list)
    seg, last_t = 0, None
    for t, rows in objs:
        if last_t is not None and t < last_t - 1.0:
            seg += 1                                     # bag loop: ids restart
        last_t = t
        pose = pose_at(ts, poses, t)
        if pose is None:
            continue
        for tid, ex, ey, speed, status in rows:
            if speed > args.max_speed or status == 0:    # moving, or tentative
                continue
            r = math.hypot(ex, ey)
            if not (3.0 < r < args.max_range):
                continue
            tracks[(seg, tid)].append((ex, ey, r, pose))

    rows_A, rows_y, used, spans = [], [], 0, []
    for key, obs in tracks.items():
        if len(obs) < args.min_obs:
            continue
        bearings = np.array([math.atan2(o[1], o[0]) for o in obs])
        span = math.degrees(bearings.max() - bearings.min())
        if span < args.min_bearing_span:
            continue
        spans.append(span)
        A_k, y_k = [], []
        for ex, ey, r, (px, py, psi) in obs:
            c, s = math.cos(psi), math.sin(psi)
            R = np.array([[c, -s], [s, c]])
            u = np.array([ex, ey]) / r
            perp = np.array([-u[1], u[0]])
            # columns: -de_x, -de_y, b, dpsi      (de enters negative, see the docstring)
            A_k.append(np.column_stack([-R, (R @ u).reshape(2, 1), (R @ perp * r).reshape(2, 1)]))
            y_k.append(np.array([px, py]) + R @ np.array([ex, ey]) + R @ np.array(NOMINAL_OFFSET))
        A_k = np.concatenate(A_k)                        # (2n, 4)
        y_k = np.concatenate(y_k)                        # (2n,)
        n = len(obs)
        # Eliminate the track's unknown true position: demean x and y rows separately.
        for start in (0, 1):
            idx = np.arange(start, 2 * n, 2)
            A_k[idx] -= A_k[idx].mean(axis=0)
            y_k[idx] -= y_k[idx].mean()
        rows_A.append(A_k)
        rows_y.append(y_k)
        used += 1

    if used < 5:
        print(f"only {used} usable static tracks -- need more, or relax --min-bearing-span")
        return
    if args.profile:
        # One parameter, everything else untouched: for each candidate offset, how much does a
        # static track's world position still wander? The per-track mean is removed, so a
        # constant shift costs nothing -- only the part that changes as the vehicle's heading
        # and the object's bearing change. A flat curve means this drive cannot tell.
        print(f"  {'offset_x':>9s} {'scatter':>9s}   (m, per-track RMS about each track's mean)")
        best = None
        for cand in np.arange(-1.0, 3.25, 0.25):
            tot, n = 0.0, 0
            for key, obs in tracks.items():
                if len(obs) < args.min_obs:
                    continue
                pts = []
                for ex, ey, r, (px, py, psi) in obs:
                    c, s_ = math.cos(psi), math.sin(psi)
                    R = np.array([[c, -s_], [s_, c]])
                    pts.append(np.array([px, py]) + R @ (np.array([ex, ey])
                                                         + np.array([cand, NOMINAL_OFFSET[1]])))
                pts = np.array(pts)
                d = pts - pts.mean(axis=0)
                tot += float((d * d).sum())
                n += len(pts)
            rms = math.sqrt(tot / max(n, 1))
            mark = ""
            if abs(cand - NOMINAL_OFFSET[0]) < 0.13:
                mark = "  <- in use today"
            elif abs(cand - 0.67) < 0.13:
                mark = "  <- vendor IMU extrinsic"
            print(f"  {cand:9.2f} {rms:9.3f}{mark}")
            if best is None or rms < best[1]:
                best = (cand, rms)
        print(f"\n  best {best[0]:+.2f} m at {best[1]:.3f} m scatter")
        return

    sol, A, y, resid = fit(rows_A, rows_y, used)
    boot = bootstrap(rows_A, rows_y)
    lo, hi = np.percentile(boot, [2.5, 97.5], axis=0)
    cond = np.linalg.cond(A.T @ A)
    sigma = math.sqrt(float(resid @ resid) / len(resid))
    de_x, de_y, b, dpsi = sol

    print(f"[fit] {used} static tracks, {len(y) // 2} observations, "
          f"bearing span median {np.median(spans):.0f} deg, residual sd {sigma:.2f} m")
    print(f"      design condition number {cond:.0f} "
          f"({'well posed' if cond < 100 else 'ILL-CONDITIONED: the terms trade off'})\n")
    print("  95% intervals are bootstrapped over TRACKS, not observations.\n")
    print(f"  lever-arm correction   de_x = {de_x:+.3f} [{lo[0]:+.3f}, {hi[0]:+.3f}] m")
    print(f"                         de_y = {de_y:+.3f} [{lo[1]:+.3f}, {hi[1]:+.3f}] m")
    print(f"     -> lidar_offset would become ({NOMINAL_OFFSET[0] + de_x:.3f}, "
          f"{NOMINAL_OFFSET[1] + de_y:.3f}) m, from ({NOMINAL_OFFSET[0]:.3f}, "
          f"{NOMINAL_OFFSET[1]:.3f})")
    print(f"  radial range bias      b    = {b:+.3f} [{lo[2]:+.3f}, {hi[2]:+.3f}] m "
          f"(positive: too far away)")
    print(f"  residual ego-yaw       dpsi = {math.degrees(dpsi):+.3f} "
          f"[{math.degrees(lo[3]):+.3f}, {math.degrees(hi[3]):+.3f}] deg (0 = -5.35 is right)")

    base = float(np.sqrt((y @ y) / len(y)))
    after = float(np.sqrt((resid @ resid) / len(resid)))
    print(f"\n  per-track position scatter: {base:.3f} m before the fit, {after:.3f} m after "
          f"({100 * (1 - after / base):.0f}% removed)")
    print("  A term is only real if its own standard error is well under its value AND it "
          "reduces the scatter; a lever arm that does not shrink the scatter is a fitting "
          "artefact.")


if __name__ == "__main__":
    main()
