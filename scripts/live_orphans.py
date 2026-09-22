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

# MATCH ON HEADER STAMPS, not on the time a message was recorded. The aggregator holds every
# measurement in a fixed-lag queue and publishes the released state stamped with that
# measurement's own CAPTURE time, so a measurement and the frame carrying it share a stamp to
# within a millisecond -- while their recording times differ by the pipeline's wall-clock latency,
# which is not a constant: it measured +78 ms with the cluster path running and +40 ms without.
#
# Matching on recording time therefore compares a measurement against a frame from before its
# update landed, by an amount that DIFFERS PER ARM. That is invisible for a long-lived track and
# decisive for a short-lived one, so it reads as a far-band effect, and it silently favours
# whichever arm is faster. Every orphan number recorded before 2026-09-23 was measured that way;
# `--log-time` reproduces them.
STAMP_TOL_S = 0.05
USE_LOG_TIME = False


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


def _stamp(msg, log_time_ns):
    """Capture time from the header; the recording time only as a fallback (--log-time)."""
    if USE_LOG_TIME or not msg.markers:
        return log_time_ns * 1e-9
    h = msg.markers[0].header.stamp
    return h.sec + h.nanosec * 1e-9


def load(path):
    meas, pub = [], []
    for m in read_ros2_messages(path, topics=[MEAS, PUB]):
        t = _stamp(m.ros_msg, m.log_time_ns)
        c = centres(m.ros_msg)
        if m.channel.topic == MEAS:
            k = math.radians(EGO_YAW_DEG)
            co, si = math.cos(k), math.sin(k)
            c = [(x * co - y * si, x * si + y * co) for (x, y) in c]
            meas.append((t, c))
        else:
            pub.append((t, c))
    # Sort by stamp ONLY, and stably: several frames can share a stamp (the
    # camera detections and the LiDAR clusters are both stamped from the same
    # sweep), and a plain sort() then orders those by their CONTENTS, which
    # scrambles the one order that matters -- the order they were published in.
    # The last frame at a stamp is the one that has seen everything released at
    # that instant; picking any other reads a track out of existence.
    meas.sort(key=lambda r: r[0]); pub.sort(key=lambda r: r[0])
    return meas, pub


def main(path):
    meas, pub = load(path)
    pub_t = [t for t, _ in pub]
    print(f"{path}\n  camera frames {len(meas)}  published frames {len(pub)}")
    if not meas or not pub:
        sys.exit("nothing recorded")

    dists, bands, shares, dups, tdup = [], defaultdict(list), [], [], []
    along, cross = [], []
    tol = 0.15 if USE_LOG_TIME else STAMP_TOL_S
    for t, cs in meas:
        i = bisect.bisect_left(pub_t, t)
        cand = [j for j in (i - 1, i) if 0 <= j < len(pub)]
        if not cand:
            continue
        j = min(cand, key=lambda k: abs(pub_t[k] - t))
        if abs(pub_t[j] - t) > tol:                 # no published frame at this instant
            continue
        # The node can publish MORE THAN ONE frame at the same stamp -- with the cluster path on it
        # publishes one per measurement released in a tick, and the cluster's frame carries the
        # same release stamp as the camera's but not yet the camera's update. Take the LAST frame
        # at this stamp, which is the one that has seen everything released at that instant.
        # Reading the first instead cost 8 points of apparent far-band coverage, all of it
        # attributed to the cluster path, twice.
        while j + 1 < len(pub) and pub_t[j + 1] == pub_t[j]:
            j += 1
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
    ap.add_argument("--log-time", action="store_true",
                    help="match on RECORDING time instead of header stamps, which reproduces "
                         "every orphan number measured before 2026-09-23 -- and their bias")
    args = ap.parse_args()
    USE_LOG_TIME = args.log_time
    print("  matching on " + ("RECORDING time (the pre-2026-09-23 rule, biased by pipeline "
                              "latency)" if USE_LOG_TIME else "header stamps"))
    for p in args.recordings:
        main(p)
        print()
