#!/usr/bin/env python3
"""Offline validation and A/B for the radar association, straight from an mcap bag.

No ROS graph, no containers, no GPU. Reads the bag directly, so the sweep is cheap to repeat
and works with the stack shut down.

Two modes:

  --check-conventions   Verifies the two ESR sign conventions against ego odometry. This is
                        the acceptance test that has to pass before any association result
                        means anything: if range_rate or azimuth is decoded with the wrong
                        sign, every downstream number is confidently wrong.

  --sweep               Scores association gate settings against each other on the same
                        frames, reporting match rate and the range/azimuth residuals.

``gate_tracks`` and ``associate`` are imported from radar_ros.radar_geometry rather than
copied, so what is scored here is literally the production rule -- the same reason
scripts/patch_ab.py imports reject_ground from fusion_node.

Why the conventions need two different tests
--------------------------------------------
For a static target the range rate must equal ``-v_ego * cos(azimuth)``. That pins the
range_rate sign hard, but it says NOTHING about the azimuth sign, because cos is even: a
left-positive and a right-positive convention score identically. The azimuth sign is settled
separately by the lever-arm term, which is odd in azimuth -- while the vehicle yaws at w the
sensor origin translates laterally, adding ``-w * L * sin(azimuth)``. Fitting L and checking
it against the surveyed mounting distance resolves the sign.
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "radar_ros"))
from radar_ros.radar_geometry import gate_tracks  # noqa: E402

RADAR_TOPIC = "/delphi_esr_interface/radar/tracks"
ODOM_TOPIC = "/novatel/oem7/odom"

# Surveyed in ros_drivers/src/vehicle_platform/config/extrinsics.yaml: the radar sits
# +2.915 m forward of lidar_tc. The lever-arm fit should land near this.
SURVEYED_LEVER_ARM_M = 2.915

#: IMU -> radar, the reference point the INS twist is actually at (HANDOFF: the twist is at the
#: IMU, not base_link). Forward 3.573 m; the lateral figure is the one the old 1-D fit could not
#: see, because its term vanishes on straight driving.
SURVEYED_LEVER_FORWARD_M = 3.573
SURVEYED_LEVER_LATERAL_M = -0.809


def _resolve_bag(path):
    p = Path(path)
    if p.is_dir():
        files = sorted(p.glob("*.mcap"))
        if not files:
            sys.exit(f"no .mcap file inside {p}")
        return files[0]
    return p


def _load(bag):
    try:
        from mcap_ros2.reader import read_ros2_messages
    except ImportError:
        sys.exit("pip install mcap-ros2-support")

    # Every file in the bag, not just the first: a 398 s drive is split across 6 mcaps here, and
    # reading one of them left the lever-arm fit with 384 turning observations instead of ~9000.
    files = sorted(Path(bag).parent.glob("*.mcap")) if Path(bag).is_file() else \
        sorted(Path(bag).glob("*.mcap"))
    read_all = lambda topics: (m for f in files for m in read_ros2_messages(str(f), topics=topics))

    odom = []
    for m in read_all([ODOM_TOPIC]):
        tw = m.ros_msg.twist.twist
        odom.append(
            (m.log_time_ns * 1e-9, math.hypot(tw.linear.x, tw.linear.y), tw.angular.z,
             tw.linear.x, tw.linear.y)
        )
    if not odom:
        sys.exit(f"{bag} has no {ODOM_TOPIC}; the convention checks need ego motion")

    sweeps = []
    for m in read_all([RADAR_TOPIC]):
        tr = m.ros_msg.tracks
        if not tr:
            continue
        sweeps.append(
            (
                m.log_time_ns * 1e-9,
                np.array([[t.range, t.angle, t.range_rate, t.amplitude,
                           t.track_status, t.update_count, t.track_id] for t in tr],
                         dtype=np.float64),
            )
        )
    if not sweeps:
        sys.exit(f"{bag} has no {RADAR_TOPIC}")
    return np.array(odom), sweeps


def _flatten(odom, sweeps, min_speed):
    """All valid track observations while the vehicle is moving, with ego state attached."""
    rows = []
    for t, a in sweeps:
        v = float(np.interp(t, odom[:, 0], odom[:, 1]))
        w = float(np.interp(t, odom[:, 0], odom[:, 2]))
        if v < min_speed:
            continue
        ok = (a[:, 4] != 0) & (a[:, 0] > 1.0)
        for rng, ang, rr in a[ok][:, :3]:
            rows.append((v, w, rng, ang, rr))
    return np.array(rows)


def lever_arm_2d(odom, sweeps, min_speed, min_yaw_rate=0.03):
    """Fit BOTH components of the IMU->radar lever arm, on TURNING data.

    The 1-D fit above uses straight driving, where the lateral arm cancels exactly (its term is
    proportional to the yaw rate) -- which is why it returned -0.111 m against a surveyed -0.809 m
    and why that gap was never explained. With the vehicle yawing at w, a target static over
    ground gives, in the sensor frame:

        range_rate = -[(vx - w*ly)*cos(az) + (vy + w*lx)*sin(az)]

    so, moving the known ego terms to the left,

        y = range_rate + vx*cos(az) + vy*sin(az) = ly*(w*cos(az)) + lx*(-w*sin(az))

    which is linear in (ly, lx). Static targets are selected by the residual of the simple model
    and then trimmed robustly, so a moving vehicle in the scene cannot drag the fit.
    """
    rows = []
    for t, a in sweeps:
        v = float(np.interp(t, odom[:, 0], odom[:, 1]))
        w = float(np.interp(t, odom[:, 0], odom[:, 2]))
        vx = float(np.interp(t, odom[:, 0], odom[:, 3]))
        vy = float(np.interp(t, odom[:, 0], odom[:, 4]))
        if v < min_speed or abs(w) < min_yaw_rate:
            continue
        ok = (a[:, 4] != 0) & (a[:, 0] > 1.0)
        for rng, ang, rr in a[ok][:, :3]:
            rows.append((vx, vy, w, ang, rr))
    r = np.array(rows)
    print(f"3. lever arm, both components, on TURNING data (|w| > {min_yaw_rate} rad/s)")
    if len(r) < 500:
        print(f"   only {len(r)} turning observations; need a bag with more yaw\n")
        return
    vx, vy, w, az, rr = r.T
    a = np.radians(az)
    y = rr + vx * np.cos(a) + vy * np.sin(a)

    def fit(X, mask, labels):
        keep = mask.copy()
        for _ in range(3):
            c, *_ = np.linalg.lstsq(X[keep], y[keep], rcond=None)
            resid = y - X @ c
            keep = keep & (np.abs(resid) <= 3.0 * 1.4826 * np.median(np.abs(resid[keep])))
        n = int(keep.sum())
        cov = np.linalg.inv(X[keep].T @ X[keep]) * float(
            np.sum((y - X @ c)[keep] ** 2) / max(n - X.shape[1], 1))
        se = np.sqrt(np.diag(cov))
        rms = float(np.sqrt(np.mean((y - X @ c)[keep] ** 2)))
        return c, se, n, rms, labels

    core = np.abs(y) < 2.0
    X2 = np.column_stack([w * np.cos(a), -w * np.sin(a)])           # (ly, lx)
    # A radar BORESIGHT error delta has its own signature: to first order it adds
    # delta * (vx*sin(az) - vy*cos(az)), which does not need the vehicle to be turning at all.
    # Fitting it alongside shows whether the lateral arm is real or is absorbing a boresight.
    X3 = np.column_stack([X2, vx * np.sin(a) - vy * np.cos(a)])     # (ly, lx, delta)
    print(f"   turning observations : {len(r)}")
    for X, labels in ((X2, ("ly", "lx")), (X3, ("ly", "lx", "delta"))):
        c, se, n, rms, _ = fit(X, core, labels)
        parts = []
        for name, val, err in zip(labels, c, se):
            unit = "deg" if name == "delta" else "m"
            v = math.degrees(val) if name == "delta" else val
            e = math.degrees(err) if name == "delta" else err
            parts.append(f"{name} = {v:+.3f} +/- {e:.3f} {unit}")
        print(f"   {'2-par' if X is X2 else '3-par'} fit (n={n:6d}, rms {rms:.3f} m/s): "
              + "   ".join(parts))
    print(f"   surveyed: ly {SURVEYED_LEVER_LATERAL_M:+.3f} m, lx {SURVEYED_LEVER_FORWARD_M:+.3f} m"
          f" (IMU -> radar)")
    for tag, m in (("left turns  (w > 0)", core & (w > 0)), ("right turns (w < 0)", core & (w < 0))):
        if m.sum() < 500:
            continue
        c, se, n, rms, _ = fit(X2, m, ("ly", "lx"))
        print(f"   {tag}: ly = {c[0]:+.3f} +/- {se[0]:.3f}   lx = {c[1]:+.3f} +/- {se[1]:.3f}"
              f"   (n={n})")
    print()
    return None


def check_conventions(odom, sweeps, min_speed):
    r = _flatten(odom, sweeps, min_speed)
    if len(r) < 500:
        sys.exit(f"only {len(r)} moving observations; need a bag with more ego motion")
    v, w, rng, az, rr = r.T
    a = np.radians(az)

    print(f"observations (moving > {min_speed} m/s): {len(r)}")
    print(f"ego speed {v.min():.1f}-{v.max():.1f} m/s, yaw rate "
          f"{w.min():+.3f}..{w.max():+.3f} rad/s\n")

    # ---- 1. range_rate sign, via the static-world model -----------------------------
    pred = -v * np.cos(a)
    res = rr - pred
    core = np.abs(res) < 3.0
    print("1. range_rate sign  (static target: range_rate == -v_ego*cos(az))")
    print(f"   within 3 m/s   : {core.mean()*100:5.1f}%   "
          f"median {np.median(res[core]):+.3f}  p90|res| "
          f"{np.percentile(np.abs(res[core]), 90):.3f} m/s")
    for label, alt in (("range_rate flipped", -rr - pred), ("no ego term", rr)):
        c = np.abs(alt) < 3.0
        print(f"   control {label:20s}: within 3 m/s {c.mean()*100:5.1f}%")
    rr_ok = core.mean() > 0.90
    print(f"   => range_rate positive = RECEDING: {'CONFIRMED' if rr_ok else 'FAILED'}\n")

    # ---- 2. azimuth sign, via the lever arm ------------------------------------------
    print("2. azimuth sign  (cos is even, so test 1 cannot see this)")
    off = np.abs(az) > 2.0
    base = (rr + v * np.cos(a))[off]      # == -w*L*sin(az) for a static point
    keep = np.abs(base) < 2.0
    x = (-w * np.sin(a))[off][keep]
    y = base[keep]
    if np.dot(x, x) <= 0:
        print("   no yaw-rate signal in this bag; azimuth sign UNRESOLVED\n")
        return rr_ok
    L = float(np.dot(x, y) / np.dot(x, x))
    resid = y - L * x
    se = float(np.sqrt(np.sum(resid ** 2) / max(len(x) - 1, 1) / np.sum(x ** 2)))
    print(f"   static-world core   : {keep.sum()} obs")
    print(f"   fitted lever arm L  : {L:+.3f} +/- {se:.3f} m  ({abs(L/se):.1f} sigma)")
    print(f"   surveyed mounting   : {SURVEYED_LEVER_ARM_M:+.3f} m forward of lidar_tc")
    az_ok = L > 0 and abs(L / se) > 3.0
    print(f"   => azimuth positive = {'LEFT' if L > 0 else 'RIGHT'}: "
          f"{'CONFIRMED' if az_ok else 'WEAK/FAILED'}\n")
    return rr_ok and az_ok


def sweep(odom, sweeps, min_speed, gates):
    """Score gate settings by how many tracks survive and how stable the set is.

    Without recorded /fused_bbox in the bag there is nothing to associate against, so this
    reports track yield rather than match rate. Point it at a probe bag that also carries
    /fused_bbox to get association numbers.
    """
    print(f"{'min_amp':>8} {'min_upd':>8} {'tracks/sweep':>14} {'p90':>6} {'sweeps':>8}")
    for min_amp, min_upd in gates:
        counts = []
        for t, a in sweeps:
            v = float(np.interp(t, odom[:, 0], odom[:, 1]))
            if v < min_speed:
                continue
            keep = gate_tracks(
                a[:, 0], a[:, 1], a[:, 3], a[:, 4].astype(int), a[:, 5].astype(int),
                min_range=1.0, max_range=175.0,
                min_amplitude=min_amp, min_update_count=min_upd,
            )
            counts.append(int(keep.sum()))
        c = np.array(counts)
        print(f"{min_amp:8.1f} {min_upd:8d} {c.mean():14.2f} "
              f"{np.percentile(c, 90):6.0f} {len(c):8d}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="mcap file or a bag directory containing one")
    ap.add_argument("--check-conventions", action="store_true",
                    help="verify the range_rate and azimuth sign conventions")
    ap.add_argument("--sweep", action="store_true", help="sweep the track gating settings")
    ap.add_argument("--lever-arm", action="store_true",
                    help="fit both lever-arm components on turning data")
    ap.add_argument("--min-speed", type=float, default=3.0,
                    help="ignore frames slower than this, m/s (default 3.0)")
    args = ap.parse_args()

    if not args.check_conventions and not args.sweep and not args.lever_arm:
        args.check_conventions = True

    bag = _resolve_bag(args.bag)
    print(f"bag: {bag}\n")
    odom, sweeps = _load(bag)

    ok = True
    if args.lever_arm:
        lever_arm_2d(odom, sweeps, args.min_speed)
    if args.check_conventions:
        ok = check_conventions(odom, sweeps, args.min_speed)
    if args.sweep:
        # Amplitude floor is -10 on this sensor, so the useful sweep is around it rather
        # than above zero. min_update_count stays 0: the driver never populates it, and any
        # positive value drops every track.
        sweep(odom, sweeps, args.min_speed,
              [(a, 0) for a in (-1e9, -10.0, -5.0, 0.0, 5.0, 10.0)])

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
