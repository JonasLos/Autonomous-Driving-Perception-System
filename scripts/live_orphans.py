"""Live orphan / duplicate audit from a marker recording.

Reads /perception/measurements/camera_lidar_markers (what the detector measured) and
/perception/objects_markers (what the aggregator published) and answers the user's question
directly: does every blue measurement box have a published track on it?

Every measurement is matched to the NEAREST published track at the closest published stamp,
so "orphan" here means no track within 3 m, and the distance histogram separates a dropped
object (no track anywhere) from a displaced one (a track 3-8 m away).
"""
import sys, math, bisect, argparse
from collections import defaultdict
import numpy as np
from mcap_ros2.reader import read_ros2_messages

MEAS = "/perception/measurements/camera_lidar_markers"
PUB = "/perception/objects_markers"

# The two streams are NOT in the same frame: measurements are published in lidar_tc, tracks in
# ego, which is yawed -5.35 deg from it (ego_frame_publisher logs "rotating a POINT the other
# way uses +5.350"). Comparing them unrotated charges the aggregator for the frame difference --
# 6.5 m of cross-offset at 70 m, which reads as a 98.8% orphan rate in the far bands.
EGO_YAW_DEG = 5.35

# Past object_fusion's RANGE_TRUST_MAX_M the filter deliberately stops taking the camera's range
# (it is biased -7.6 m at 80-100 m and worse beyond), so there the track and the measurement are
# SUPPOSED to disagree along the ray. A plain 3 m test counts that intended divergence as a lost
# object -- it read 30.5% "orphaned" past 80 m. Out there a measurement is covered when a track
# sits on its BEARING (within 3 m across the ray) at any plausible range.
RANGE_TRUST_MAX_M = 80.0
FAR_ALONG_FRAC = 0.35

# A measurement is applied by the aggregator's fixed-lag queue `measurement_lag` seconds after its
# capture stamp, so the published frame that CARRIES it is that much later. Matching a measurement
# to the nearest frame in time therefore reads a frame from before its own update landed. That is
# invisible for a long-lived track (it is in every frame either way) and dominant for a short-lived
# one, which is why it showed up as a far-band effect: at >80 m, 69% of "orphans" have their track
# on the bearing within 25 ms, and 92% within 100 ms.
#
# The default is the node's own `measurement_lag`, because that is the frame which answers the
# question this script asks. Pass `--lag 0` to reproduce any orphan number recorded before
# 2026-09-23 -- all of them were measured with the nearest-frame rule, and they are only
# comparable with each other when both arms published at the same RATE. They did not: the cluster
# path publishes at 49.6 Hz against 39.7, which is what made it look like a far-band regression.
MATCH_LAG_S = 0.12


def centres(msg):
    out = []
    for m in msg.markers:
        if getattr(m, "action", 0) == 2:            # DELETE / DELETEALL
            continue
        if m.type != 1:                             # CUBE only; skip text and arrows
            continue
        p = m.pose.position
        out.append((p.x, p.y))
    return out


def load(path):
    meas, pub = [], []
    for m in read_ros2_messages(path, topics=[MEAS, PUB]):
        t = m.log_time_ns * 1e-9
        c = centres(m.ros_msg)
        if m.channel.topic == MEAS:
            k = math.radians(EGO_YAW_DEG)
            co, si = math.cos(k), math.sin(k)
            c = [(x * co - y * si, x * si + y * co) for (x, y) in c]
            meas.append((t, c))
        else:
            pub.append((t, c))
    meas.sort(); pub.sort()
    return meas, pub


def main(path):
    meas, pub = load(path)
    pub_t = [t for t, _ in pub]
    print(f"{path}\n  camera frames {len(meas)}  published frames {len(pub)}")
    if not meas or not pub:
        sys.exit("nothing recorded")

    dists, bands, shares, dups, tdup = [], defaultdict(list), [], [], []
    along, cross = [], []
    for t, cs in meas:
        want = t + MATCH_LAG_S                      # the frame that CARRIES this measurement
        i = bisect.bisect_left(pub_t, want)
        cand = [j for j in (i - 1, i) if 0 <= j < len(pub)]
        if not cand:
            continue
        j = min(cand, key=lambda k: abs(pub_t[k] - want))
        if abs(pub_t[j] - want) > 0.15:             # no published frame near this measurement
            continue
        tracks = pub[j][1]
        claimed = defaultdict(int)
        for (x, y) in cs:
            r = math.hypot(x, y)
            if not tracks:
                dists.append(float("inf")); bands[band(r)].append(1.0)
                continue
            d = [math.hypot(x - qx, y - qy) for (qx, qy) in tracks]
            k = int(np.argmin(d))
            dists.append(d[k])
            covered = d[k] < 3.0
            if not covered and r >= RANGE_TRUST_MAX_M:
                ux, uy = x / r, y / r
                for (qx, qy) in tracks:
                    a_ = (qx - x) * ux + (qy - y) * uy
                    c_ = -(qx - x) * uy + (qy - y) * ux
                    if abs(c_) < 3.0 and abs(a_) < FAR_ALONG_FRAC * r:
                        covered = True
                        break
            bands[band(r)].append(0.0 if covered else 1.0)
            if d[k] < 3.0:
                claimed[k] += 1
                qx, qy = tracks[k]
                # split the offset into along-ray and cross-ray, in the measurement's frame
                ux, uy = x / max(r, 1e-6), y / max(r, 1e-6)
                along.append((qx - x) * ux + (qy - y) * uy)
                cross.append(-(qx - x) * uy + (qy - y) * ux)
        shares.append((sum(1 for v in claimed.values() if v >= 2), len(tracks)))
        dups.append((sum(1 for a in range(len(tracks))
                         if any(math.hypot(tracks[a][0] - tracks[b][0],
                                           tracks[a][1] - tracks[b][1]) < 2.0
                                for b in range(len(tracks)) if b != a)), len(tracks)))
        # "within 2 m of another track" counts two cones 2 m apart as a duplicate, which is
        # exactly the case merging must NOT collapse. A TRUE duplicate is two tracks whose
        # nearest measurement is the SAME measurement -- two boxes on one object.
        owner = {}
        for a in range(len(tracks)):
            qx, qy = tracks[a]
            if not cs:
                continue
            e = [math.hypot(px - qx, py - qy) for (px, py) in cs]
            k = int(np.argmin(e))
            if e[k] < 3.0:
                owner.setdefault(k, []).append(a)
        tdup.append((sum(len(v) for v in owner.values() if len(v) >= 2), len(tracks)))

    d = np.asarray(dists)
    n = d.size
    print(f"  measurements matched {n}")
    aware = np.concatenate([np.asarray(v) for v in bands.values()])
    print(f"  ORPHANED: {100 * aware.mean():.1f}%  (range-aware: past {RANGE_TRUST_MAX_M:.0f} m a "
          f"track on the bearing counts)   naive 3 m test: {100 * np.mean(d >= 3.0):.1f}%")
    print("  nearest published track:  "
          + "  ".join(f"<{c} m {100 * np.mean(d < c):5.1f}%" for c in (1, 2, 3, 5, 8))
          + f"   none at all {100 * np.mean(~np.isfinite(d)):.1f}%")
    print("  by range band: " + "  ".join(
        f"{k} {100 * np.mean(v):.1f}% (n={len(v)})" for k, v in sorted(bands.items())))
    a, c = np.abs(np.asarray(along)), np.abs(np.asarray(cross))
    if a.size:
        print(f"  offset of the track it DID match: |along| median {np.median(a):.2f} m  "
              f"|cross| median {np.median(c):.2f} m  (n={a.size})")
    s = np.asarray(shares, dtype=float); u = np.asarray(dups, dtype=float)
    print(f"  SHARED  (one track covering 2+ measurements): "
          f"{100 * s[:, 0].sum() / max(s[:, 1].sum(), 1):.1f}% of published tracks")
    print(f"  DUPLICATE (a track within 2 m of another):    "
          f"{100 * u[:, 0].sum() / max(u[:, 1].sum(), 1):.1f}% of published tracks")
    td = np.asarray(tdup, dtype=float)
    print(f"  TRUE DUP  (2+ tracks on the SAME measurement): "
          f"{100 * td[:, 0].sum() / max(td[:, 1].sum(), 1):.1f}% of published tracks")


def band(r):
    for lo, hi in ((0, 25), (25, 40), (40, 60), (60, 80), (80, 200)):
        if lo <= r < hi:
            return f"{lo}-{hi}"
    return "200+"


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recordings", nargs="+")
    ap.add_argument("--lag", type=float, default=MATCH_LAG_S,
                    help="seconds to look FORWARD for the frame carrying each measurement. "
                         "Default is the node's measurement_lag; --lag 0 reproduces the "
                         "nearest-frame rule every number before 2026-09-23 was measured with")
    args = ap.parse_args()
    MATCH_LAG_S = args.lag
    print(f"  matching each measurement to the frame at +{MATCH_LAG_S:.3f} s")
    for p in args.recordings:
        main(p)
        print()
