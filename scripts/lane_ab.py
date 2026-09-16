"""Offline A/B for CLRerNet lane-pair selection.

Replays a recorded probe bag (/clrernet/all_lanes + /lidar_2d_projection) with no GPU, no
containers and no ROS graph, and runs two arms on *identical* paired inputs:

  A = the algorithm at git HEAD (index-paired centreline, whole-polyline mean_y sort)
  B = perception_common.lane_geometry (shared longitudinal grid, tiers, hysteresis, ego ray)

Both arms publish the same way the node does by default -- the selected boundaries' own matched
returns -- so every metric below is computed by one formula on comparable arrays and only the
selection differs.

Every lateral quantity below is measured against the *ego ray*, not against y=0. lidar_tc is
yawed about 5.35 deg from the vehicle axis, so y=0 diverges from the ego path by 9.4 cm per metre
of range and "ego bracketed" against it is meaningless past ~19 m -- it reports the neighbouring
lane as the containing one. Both arms are scored against the same ray, so the comparison is fair;
arm A simply does not use it when selecting.

Arm A is checked against the recorded /clrernet/left_lane before any metric is printed: if it
does not reproduce what the node actually published, the harness is measuring itself rather than
the change, and it says so.
"""

import sys
import numpy as np
from scipy.spatial import KDTree

REPO = "/home/avalocal/Downloads/Autonomous-Driving-Perception-System"
sys.path.insert(0, REPO + "/src/perception_common")

from perception_common.lane_geometry import (  # noqa: E402
    DEFAULT_EGO_YAW_DEG, TIER_CONTAINED, TIER_NEAR_EMPTY, LanePairSelector, make_grid,
    resample_lane,
)
from perception_common.stamp_sync import DEFERRED, StampMatchedBuffer  # noqa: E402
from mcap_ros2.reader import read_ros2_messages  # noqa: E402

PIXEL_LIM = 10
GRID = make_grid(5.0, 100.0, 0.5)
EGO_SLOPE = np.tan(np.radians(DEFAULT_EGO_YAW_DEG))


def ego_y(x):
    """The ego path in the LiDAR frame at range ``x``. See the module docstring."""
    return EGO_SLOPE * np.asarray(x, dtype=float)


def matched_3d(lane_uv, tree, pc_arr):
    dist, idx = tree.query(lane_uv)
    return pc_arr[idx[dist < PIXEL_LIM]]


def arm_a(lanes, tree, pc_arr):
    """Verbatim reproduction of get_closest_lane_pair_3d at b0919bf."""
    lanes_3d = []
    for lane in lanes:
        l3 = matched_3d(lane, tree, pc_arr)
        if l3.shape[0] > 0:
            lanes_3d.append((lane, l3, np.mean(l3[:, 1])))
    if len(lanes_3d) < 2:
        return np.empty((0, 3)), np.empty((0, 3))

    lanes_3d.sort(key=lambda t: t[2])
    best, min_d = None, float("inf")
    for i in range(len(lanes_3d) - 1):
        u1, a1, _ = lanes_3d[i]
        u2, a2, _ = lanes_3d[i + 1]
        n = min(len(a1), len(a2))
        if n == 0:
            continue
        c = (a1[:n] + a2[:n]) / 2.0
        d = abs(c[:10].mean(axis=0)[1] - 0.0)
        if d < min_d:
            min_d = d
            best = (u1, u2) if np.mean(a1[:, 1]) > np.mean(a2[:, 1]) else (u2, u1)
    if best is None:
        return np.empty((0, 3)), np.empty((0, 3))
    # The node re-queries the tree with these uv arrays to publish.
    return matched_3d(best[0], tree, pc_arr), matched_3d(best[1], tree, pc_arr)


def arm_b(lanes, tree, pc_arr, selector, now):
    resampled, raw = [], []
    for lane_uv in lanes:
        m = matched_3d(lane_uv, tree, pc_arr)
        lane = resample_lane(m, GRID, step=0.5, max_interp_gap=5.0,
                             max_backtrack=0.25, min_samples=3)
        if lane.valid.any():
            resampled.append(lane)
            raw.append(m)
    if len(resampled) < 2:
        return np.empty((0, 3)), np.empty((0, 3)), None
    pair, _ = selector.select(resampled, now=now)
    if pair is None:
        return np.empty((0, 3)), np.empty((0, 3)), None
    by_id = {id(l): r for l, r in zip(resampled, raw)}
    return by_id[id(pair.left)], by_id[id(pair.right)], pair


def cloud(msg, names):
    off = {f.name: f.offset for f in msg.fields}
    if not all(k in off for k in names):
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    a = np.column_stack([raw[:, off[k]:off[k] + 4].copy().view(np.float32).ravel()
                         for k in names])
    return a[np.isfinite(a).all(axis=1)]


def stamp_ns(h):
    return h.stamp.sec * 10**9 + h.stamp.nanosec


def metrics(L, R):
    """One formula, both arms: window means over the published boundary clouds.

    ``ly``/``ry`` are each boundary's mean lateral distance from the ego ray, read at that
    boundary's own mean range inside the window -- the two arms drop different points, so they do
    not share one. ``cy`` is the centreline's deviation from the ego path, which is the quantity
    the selector is trying to minimise and the one whose frame-to-frame jumps are the defect.
    """
    if len(L) == 0 or len(R) == 0:
        return None
    w = lambda A: (A[:, 0] >= 6) & (A[:, 0] <= 20)
    if not w(L).any() or not w(R).any():
        return None
    lx, rx = L[w(L), 0].mean(), R[w(R), 0].mean()
    ly = L[w(L), 1].mean() - ego_y(lx)
    ry = R[w(R), 1].mean() - ego_y(rx)
    n = min(len(L), len(R))
    return dict(dx=float(np.median(np.abs(L[:n, 0] - R[:n, 0]))),
                ly=float(ly), ry=float(ry), width=float(ly - ry), cy=float(0.5 * (ly + ry)))


def report(name, rows, extra=""):
    if len(rows) < 3:
        print(f"\n{name}: too few frames ({len(rows)})")
        return
    cy = np.array([r["cy"] for r in rows])
    w = np.array([r["width"] for r in rows])
    dx = np.array([r["dx"] for r in rows])
    ly = np.array([r["ly"] for r in rows])
    ry = np.array([r["ry"] for r in rows])
    d = np.abs(np.diff(cy))
    print(f"\n{name}   (n={len(rows)} frames){extra}")
    print(f"  index mismatch |x_L[i]-x_R[i]| : median {np.median(dx):5.2f} m   >5 m in {100*np.mean(dx>5):4.1f}% of frames")
    print(f"  lane width p50                 : {np.median(w):5.2f} m   in[2.5,4.5] {100*np.mean((w>2.5)&(w<4.5)):5.1f}%   >5.5 m {100*np.mean(w>5.5):4.1f}%")
    print(f"  ego bracketed (about the ray)  : {100*np.mean((ly>0)&(ry<0)):5.1f}%")
    print(f"  centre_y off ego path          : p50 {np.median(cy):+.2f}  p5 {np.percentile(cy,5):+.2f}  p95 {np.percentile(cy,95):+.2f} m")
    print(f"  jump |d centre_y|              : p50 {np.median(d):.3f}  p90 {np.percentile(d,90):.3f}  max {d.max():.2f} m")
    for thr in (0.5, 1.0, 1.75):
        print(f"     > {thr:4.2f} m                   : {int(np.sum(d>thr)):3d}/{len(d)}  ({100*np.mean(d>thr):5.1f}%)")
    return d


def main(path):
    lanes_msgs, projections, published = [], [], {}
    for m in read_ros2_messages(path, topics=[
            "/clrernet/all_lanes", "/lidar_2d_projection", "/clrernet/left_lane"]):
        t, r = m.channel.topic, m.ros_msg
        if t == "/clrernet/all_lanes":
            lanes_msgs.append(r)
        elif t == "/lidar_2d_projection":
            projections.append(r)
        else:
            published[stamp_ns(r.header)] = cloud(r, ("x", "y", "z"))
    print(f"loaded: all_lanes={len(lanes_msgs)}  projections={len(projections)}  "
          f"published_left={len(published)}")
    if not lanes_msgs or not projections:
        print("nothing to do"); return

    buf = StampMatchedBuffer("projection", buffer_duration=2.0, max_skew=0.08,
                             wait_for_newer=0.06, wrap=lambda msg: msg)
    events = [(stamp_ns(p.header), 0, p) for p in projections]
    events += [(stamp_ns(l.header), 1, l) for l in lanes_msgs]
    events.sort(key=lambda e: (e[0], e[1]))

    paired = []
    for ts, kind, msg in events:
        if kind == 0:
            buf.add(msg)
            for pr in buf.drain(now=ts * 1e-9):
                if pr.value is not None:
                    paired.append((pr.value, pr.payload))
        else:
            pr = buf.match(msg.header, now=ts * 1e-9, payload=msg)
            if pr.outcome is not DEFERRED and pr.value is not None:
                paired.append((pr.value, pr.payload))
    print(f"paired detections: {len(paired)}   ({buf.status()})")

    # Mirrors the node's declared defaults; ego_yaw_deg comes from the module default.
    sel = LanePairSelector(GRID, score_min_x=6.0, score_max_x=30.0, min_window_nodes=8,
                           min_lane_width=2.2, max_lane_width=4.5, incumbent_tol=0.75,
                           hysteresis_margin=0.75, switch_debounce=2, memory_timeout=0.5)
    rows_a, rows_b, tiers, checks = [], [], [], []
    for proj, det in paired:
        arr = cloud(proj, ("x", "y", "z", "u", "v"))
        if arr is None or arr.shape[0] < 10:
            continue
        pc_arr, uv = arr[:, :3], arr[:, 3:5]
        tree = KDTree(uv)
        d = {}
        for pt in det.points:
            d.setdefault(pt.lane_id, []).append([pt.x, pt.y])
        lanes = [np.asarray(p, dtype=np.float64) for _, p in sorted(d.items())]
        if not lanes:
            continue

        La, Ra = arm_a(lanes, tree, pc_arr)
        Lb, Rb, pair = arm_b(lanes, tree, pc_arr, sel, stamp_ns(proj.header) * 1e-9)

        pub = published.get(stamp_ns(proj.header))
        if pub is not None and len(pub) and len(La):
            n = min(len(pub), len(La))
            checks.append(float(np.abs(pub[:n, :3] - La[:n, :3]).max()))

        ma, mb = metrics(La, Ra), metrics(Lb, Rb)
        if ma:
            rows_a.append(ma)
        if mb:
            rows_b.append(mb)
        if pair is not None:
            tiers.append(pair.tier)

    if checks:
        bad = int(np.sum(np.array(checks) > 1e-3))
        print(f"\nself-check: arm A vs recorded /clrernet/left_lane over {len(checks)} frames — "
              f"worst |delta| {max(checks):.2e} m, mismatched frames {bad}")
        print("  => arm A reproduces the node" if bad == 0 else "  => WARNING: arm A drifted")

    report("ARM A  (git HEAD)", rows_a)
    report("ARM B  (lane_geometry)", rows_b)
    if tiers:
        t = np.array(tiers)
        print(f"\n  arm B tiers: contained {100*np.mean(t==TIER_CONTAINED):.1f}%  "
              f"near-empty {100*np.mean(t==TIER_NEAR_EMPTY):.1f}%")
        print(f"  arm B selector: {sel.status()}")


if __name__ == "__main__":
    main(sys.argv[1])
