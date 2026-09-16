#!/usr/bin/env python3
"""Ground misclassification near the car, especially in curves: what causes it, what fixes it.

User report: in curves, road close to the vehicle is shown as NON-ground. Candidate causes:

  LEAN   body roll / road bank tilts the road plane in the sensor frame, and Patchwork++'s zone
         planes and seeds assume a roughly level road around the sensor.
  RNR    Patchwork++'s reflected-noise removal drops returns steeper than RNR_ver_angle_thr
         (-15 deg) whose intensity is under RNR_intensity_thr (0.2). Production passes intensity
         ZEROS, so every such return -- the road within ~2.46 / tan(15 deg) = 9.2 m -- is "noise".

THE REFERENCE IS INDEPENDENT OF EVERY VARIANT. An earlier version scored against the inliers of
a near-field RANSAC plane, which is the very plane the "level by road plane" variant levels by --
it was graded against its own answer. Here road truth is built locally: 0.4 m cells 3-20 m out
that are flat (z range <= 8 cm, >= 4 returns), smooth with their neighbours (<= 8 cm), and the
lowest surface around (within 25 cm of the 5x5 minimum). No global plane, no attitude, no
Patchwork++.

Two errors, both reported:
  ROAD->NON   reference road labelled non-ground (the user's complaint)
  OBJ->GROUND returns 0.25-2.5 m above the local reference road labelled ground (the cost:
              cones, curbs-plus, car bodies eaten by an over-eager ground)

Variants (each its own Patchwork++ instance -- its adaptive thresholds carry state):
  V0      production: GroundSegmenter(max_range=120), intensity zeros
  V1      real intensity (raw / 255)
  V2      V1 with RNR off
  V0 RNR off   production intensity (zeros) with RNR off
  each levelling (--levels) is applied on V0, V0 RNR off and V2, so it is never confounded
  +mount  constant levelling by the mean road-plane slope (static LiDAR mount tilt)
  +odom   levelling by slopes predicted from odometry roll/pitch, linear model FITTED on --fit
          and scored on every bag (so --fit vs the other bag is an out-of-sample test)
  +odomnm the same without the model's constant (static mount tilt) row
  +plane  levelling by this sweep's own road plane (ground_segmentation.road_plane_slopes),
          odom model when the plane fit refuses
  +blend  the average of the plane and odom slopes
  +clamp  the plane slopes, limited to within 1 deg of the odom prediction

    PYTHONPATH=<pypatchworkpp dir> python3 scripts/lean_ab.py --fit BAG [--bags BAG ...] [--step 3]
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import time

import numpy as np
from scipy import ndimage

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (os.path.join(REPO, "src", "object_fusion"), os.path.join(REPO, "src", "perception_common")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mcap_ros2.reader import read_ros2_messages                                   # noqa: E402
from object_fusion.ground_segmentation import (                                   # noqa: E402
    GroundSegmenter, levelling_rotation, quaternion_roll_pitch_deg, road_plane_slopes,
)

LIDAR, ODOM = "/lidar_tc/velodyne_points", "/novatel/oem7/odom"
CELL, R_MIN, R_MAX = 0.4, 3.0, 20.0
BANDS = [(3, 9), (9, 15), (15, 20)]
CURVE_ACC = 1.5                      # m/s^2 of v*omega; above this a sweep counts as a curve
CORRIDOR_HALF_WIDTH = 6.0            # m either side of the vehicle's current path arc
EGO_YAW_DEG = -5.35                  # vehicle axis in lidar_tc (see HANDOFF)


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def decode(c):
    offs = {f.name: f.offset for f in c.fields}
    a = np.frombuffer(bytes(c.data), dtype=np.uint8).reshape(-1, c.point_step)
    g = lambda n: a[:, offs[n]:offs[n] + 4].copy().view(np.float32).ravel()
    p = np.stack([g("x"), g("y"), g("z"), g("intensity")], 1).astype(np.float64)
    return p[np.isfinite(p).all(1)]


def load_odom(bag):
    rows = []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[ODOM]):
            o = m.ros_msg
            q = o.pose.pose.orientation
            roll, pitch = quaternion_roll_pitch_deg(q.x, q.y, q.z, q.w)
            rows.append((stamp(o.header), roll, pitch, o.twist.twist.linear.x, o.twist.twist.angular.z))
    return np.array(rows)


def sweeps(bag, step):
    n = 0
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[LIDAR]):
            n += 1
            if n % step == 0:
                yield stamp(m.ros_msg.header), decode(m.ros_msg)


def at(odom, t):
    return [float(np.interp(t, odom[:, 0], odom[:, k])) for k in range(1, 5)]


def reference(xyz, kappa=0.0):
    """(corridor road mask, off-road flat terrain mask, object-like mask, low side sign or 0).

    The flat-patch reference alone also accepts terrain BESIDE the road -- a drop-off below road
    level, a vegetated embankment -- which Patchwork++ reasonably calls non-ground; on
    selfcal_loc_2026-09-03 that terrain was most of the "error". So road truth is the flat
    reference inside a corridor of +-CORRIDOR_HALF_WIDTH around the vehicle's current path arc
    (curvature ``kappa`` = yaw rate / speed, in the vehicle frame); flat terrain outside it is
    reported separately.
    """
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    r = np.hypot(x, y)
    n_cells = int(math.ceil(2 * R_MAX / CELL))
    inside = (r >= R_MIN) & (r < R_MAX)
    ix = np.clip(((x + R_MAX) / CELL).astype(int), 0, n_cells - 1)
    iy = np.clip(((y + R_MAX) / CELL).astype(int), 0, n_cells - 1)

    low = inside & (z < -1.2)
    k = ix[low] * n_cells + iy[low]
    zl = z[low]
    cnt = np.bincount(k, minlength=n_cells * n_cells)
    zsum = np.bincount(k, weights=zl, minlength=n_cells * n_cells)
    zmin = np.full(n_cells * n_cells, np.inf)
    zmax = np.full(n_cells * n_cells, -np.inf)
    np.minimum.at(zmin, k, zl)
    np.maximum.at(zmax, k, zl)
    mean = np.where(cnt > 0, zsum / np.maximum(cnt, 1), np.nan).reshape(n_cells, n_cells)
    flat = ((cnt >= 4) & (zmax - zmin <= 0.08)).reshape(n_cells, n_cells)

    padded = np.pad(mean, 1, constant_values=np.nan)
    nbrs = np.stack([padded[1 + di:1 + di + n_cells, 1 + dj:1 + dj + n_cells]
                     for di in (-1, 0, 1) for dj in (-1, 0, 1) if (di, dj) != (0, 0)])
    with np.errstate(all="ignore"):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            nbr_med = np.nanmedian(nbrs, axis=0)
    min5 = ndimage.minimum_filter(np.where(np.isnan(mean), np.inf, mean), size=5, mode="constant",
                                  cval=np.inf)
    smooth = np.isnan(nbr_med) | (np.abs(mean - nbr_med) <= 0.08)
    road_cell = flat & smooth & (mean <= min5 + 0.25)

    road_z = np.where(road_cell, mean, np.nan)
    flat_pt = inside & road_cell[ix, iy] & (np.abs(z - np.nan_to_num(mean[ix, iy])) <= 0.06)
    c, s_ = math.cos(math.radians(-EGO_YAW_DEG)), math.sin(math.radians(-EGO_YAW_DEG))
    xe, ye = c * x - s_ * y, s_ * x + c * y                    # lidar_tc -> vehicle axes
    in_corridor = np.abs(ye - 0.5 * kappa * np.maximum(xe, 0.0) ** 2) <= CORRIDOR_HALF_WIDTH
    road_pt = flat_pt & in_corridor
    offroad_pt = flat_pt & ~in_corridor

    # local road height for every point: mean of road cells in the 3x3 around it
    s = ndimage.uniform_filter(np.nan_to_num(road_z), size=3, mode="constant")
    c = ndimage.uniform_filter((~np.isnan(road_z)).astype(float), size=3, mode="constant")
    local = np.where(c > 0, s / np.maximum(c, 1e-9), np.nan)
    h = z - local[ix, iy]
    obj_pt = inside & ~road_cell[ix, iy] & np.isfinite(h) & (h > 0.25) & (h < 2.5)

    side = 0
    ring = road_pt & (np.abs(x) < 15) & (np.abs(y) > 1.5)
    zl_, zr_ = z[ring & (y > 0)], z[ring & (y < 0)]
    if zl_.size > 30 and zr_.size > 30:
        d = float(np.median(zl_) - np.median(zr_))
        if abs(d) > 0.05:
            side = 1 if d < 0 else -1               # +1: left (y > 0) is the low side
    return road_pt, offroad_pt, obj_pt, side


def fit_attitude_model(bag, odom, step):
    X, Y = [], []
    for t, p in sweeps(bag, step):
        sl = road_plane_slopes(p[:, :3])
        if sl is None:
            continue
        roll, pitch, _, _ = at(odom, t)
        X.append((roll, pitch, 1.0))
        Y.append(sl)
    X, Y = np.array(X), np.array(Y)
    coef, *_ = np.linalg.lstsq(X, Y, rcond=None)
    for _ in range(2):                                      # trim the worst 5% and refit
        res = np.linalg.norm(Y - X @ coef, axis=1)
        keep = res <= np.percentile(res, 95)
        coef, *_ = np.linalg.lstsq(X[keep], Y[keep], rcond=None)
    return coef, X, Y                                       # slopes = [roll, pitch, 1] @ coef


def clamp_to(plane, prior, max_dev_deg=1.0):
    """The plane slopes, limited to within ``max_dev_deg`` of the odometry prediction."""
    d = np.asarray(plane) - np.asarray(prior)
    lim = math.tan(math.radians(max_dev_deg))
    n = float(np.linalg.norm(d))
    return tuple(np.asarray(prior) + (d if n <= lim else d * (lim / n)))


def run(args):
    t0 = time.perf_counter()
    fit_odom = load_odom(args.fit)
    coef, Xf, Yf = fit_attitude_model(args.fit, fit_odom, args.step)
    mount = Yf.mean(axis=0)                                  # mean road slope: static mount tilt
    print(f"[fit] {os.path.basename(args.fit)}: {len(Xf)} sweeps with a road plane")
    for j, axis in enumerate("xy"):
        print(f"  slope_{axis} = {coef[0, j]:+.5f}*roll {coef[1, j]:+.5f}*pitch {coef[2, j]:+.5f}"
              f"   (odom deg; a rigid tilt would be 0.01746 per deg)")

    for bag in [args.fit] + [b for b in args.bags if b != args.fit]:
        odom = fit_odom if bag == args.fit else load_odom(bag)
        tag = os.path.basename(bag.rstrip("/"))
        _, Xb, Yb = (None, Xf, Yf) if bag == args.fit else fit_attitude_model(bag, odom, args.step)
        rms = lambda e: float(np.sqrt(np.mean(np.sum(e ** 2, axis=1))))
        deg = lambda s: math.degrees(math.atan(s))
        print(f"\n===== {tag} {'(FIT bag: in-sample)' if bag == args.fit else '(OUT OF SAMPLE)'} =====")
        print(f"[attitude model] residual road tilt, rms over {len(Xb)} sweeps: none {deg(rms(Yb)):.2f} deg | "
              f"mean mount tilt {deg(rms(Yb - mount)):.2f} deg | odom model {deg(rms(Yb - Xb @ coef)):.2f} deg")

        variants = {"V0 production": dict(inten=False, rnr=True, lvl=None),
                    "V1 intensity": dict(inten=True, rnr=True, lvl=None),
                    "V2 intensity, RNR off": dict(inten=True, rnr=False, lvl=None),
                    "V0 RNR off": dict(inten=False, rnr=False, lvl=None)}
        for lvl in args.levels:
            variants[f"V0 +{lvl}"] = dict(inten=False, rnr=True, lvl=lvl)
            variants[f"V0 RNR off +{lvl}"] = dict(inten=False, rnr=False, lvl=lvl)
            variants[f"V2 +{lvl}"] = dict(inten=True, rnr=False, lvl=lvl)
        segs = {k: GroundSegmenter(backend="patchworkpp", max_range=120.0,
                                   sensor_height=args.sensor_height,
                                   patchwork_params=None if v["rnr"] else {"enable_RNR": False})
                for k, v in variants.items()}
        acc = {k: {} for k in variants}
        n_sweeps = n_plane_fail = n_curve = 0
        ms = {k: [] for k in variants}

        def add(name, key, bad, total):
            a, b = acc[name].get(key, (0, 0))
            acc[name][key] = (a + bad, b + total)

        for t, p in sweeps(bag, args.step):
            xyz, inten = p[:, :3], np.clip(p[:, 3] / 255.0, 0.0, 1.0)
            roll, pitch, v, w = at(odom, t)
            road, offroad, obj, side = reference(xyz, kappa=w / v if abs(v) > 1.0 else 0.0)
            if road.sum() < 200:
                continue
            n_sweeps += 1
            regime = "curve" if abs(v * w) > CURVE_ACC else "straight"
            n_curve += regime == "curve"
            slopes_odom = np.array([roll, pitch, 1.0]) @ coef
            plane = road_plane_slopes(xyz)
            if plane is None:
                n_plane_fail += 1
            r = np.hypot(xyz[:, 0], xyz[:, 1])
            for name, cfg in variants.items():
                lvl = None
                if cfg["lvl"] == "mount":
                    lvl = levelling_rotation(*mount)
                elif cfg["lvl"] == "odom":
                    lvl = levelling_rotation(*slopes_odom)
                elif cfg["lvl"] == "odomnm":
                    lvl = levelling_rotation(*(np.array([roll, pitch, 0.0]) @ coef))
                elif cfg["lvl"] == "plane":
                    lvl = levelling_rotation(*(plane if plane is not None else slopes_odom))
                elif cfg["lvl"] == "blend":
                    lvl = levelling_rotation(*(0.5 * (np.asarray(plane) + slopes_odom)
                                               if plane is not None else slopes_odom))
                elif cfg["lvl"] == "clamp":
                    lvl = levelling_rotation(*(clamp_to(plane, slopes_odom)
                                               if plane is not None else slopes_odom))
                ts = time.perf_counter()
                g = segs[name].segment(xyz, intensity=inten if cfg["inten"] else None, level=lvl)
                ms[name].append(1000 * (time.perf_counter() - ts))
                add(name, (regime, "obj"), int((g & obj).sum()), int(obj.sum()))
                add(name, (regime, "offroad"), int((~g & offroad).sum()), int(offroad.sum()))
                for lo, hi in BANDS:
                    band = road & (r >= lo) & (r < hi)
                    add(name, (regime, "all", lo), int((~g & band).sum()), int(band.sum()))
                    if regime == "curve" and side != 0:
                        low = band & (np.sign(xyz[:, 1]) == side)
                        high = band & (np.sign(xyz[:, 1]) == -side)
                        add(name, ("curve", "low", lo), int((~g & low).sum()), int(low.sum()))
                        add(name, ("curve", "high", lo), int((~g & high).sum()), int(high.sum()))

        pct = lambda name, key: (f"{100 * acc[name][key][0] / acc[name][key][1]:5.1f}%"
                                 if acc[name].get(key, (0, 0))[1] else "    -")
        def tot(name, regime, kind):
            cells = [acc[name].get((regime, kind, lo), (0, 0)) for lo, _ in BANDS]
            return sum(c[0] for c in cells), sum(c[1] for c in cells)
        print(f"[sweeps] {n_sweeps} scored (every {args.step}th), {n_curve} of them in curves; "
              f"plane fit refused on {n_plane_fail}")
        hdr = "".join(f"{f'{lo}-{hi}':>7s}" for lo, hi in BANDS)
        print(f"\nROAD -> NON-GROUND (% of flat reference inside the +-{CORRIDOR_HALF_WIDTH:.0f} m path corridor)")
        print(f"  {'variant':24s} | STRAIGHT {hdr}  all | CURVE low {hdr} | CURVE high {hdr} | curve all"
              f" | OBJ->GROUND straight / curve | OFF-ROAD->NON straight / curve | ms")
        for name in variants:
            s_all = tot(name, "straight", "all")
            c_all = tot(name, "curve", "all")
            print(f"  {name:24s} |         "
                  + "".join(f"{pct(name, ('straight', 'all', lo)):>7s}" for lo, _ in BANDS)
                  + f" {100 * s_all[0] / max(s_all[1], 1):4.1f}% |          "
                  + "".join(f"{pct(name, ('curve', 'low', lo)):>7s}" for lo, _ in BANDS) + " |           "
                  + "".join(f"{pct(name, ('curve', 'high', lo)):>7s}" for lo, _ in BANDS)
                  + f" |    {100 * c_all[0] / max(c_all[1], 1):4.1f}%"
                  + f" |       {pct(name, ('straight', 'obj'))} / {pct(name, ('curve', 'obj'))}"
                  + f" |          {pct(name, ('straight', 'offroad'))} / {pct(name, ('curve', 'offroad'))}"
                  + f" | {np.median(ms[name]):4.1f}")
    print(f"\n[done] {time.perf_counter() - t0:.0f} s")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fit", required=True, help="bag the odometry->tilt model is fitted on")
    ap.add_argument("--bags", nargs="*", default=[], help="further bags to score (out of sample)")
    ap.add_argument("--step", type=int, default=3)
    ap.add_argument("--sensor-height", type=float, default=2.46,
                    help="Patchwork++ sensor_height; the stack ships 2.46, but the LiDAR measures "
                         "2.37-2.39 m above the road (jeep_selfcal_loc: 2.366 +- 0.005)")
    ap.add_argument("--levels", nargs="*", default=["odom", "plane", "blend"],
                    choices=["mount", "odom", "odomnm", "plane", "blend", "clamp"])
    run(ap.parse_args())


if __name__ == "__main__":
    main()
