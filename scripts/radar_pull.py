"""To-do item 12: is the track-vs-measurement offset the filter's error, or the radar pulling
the track toward the true surface?

The camera+LiDAR measurement sits on the near face of an object; the radar's scattering centre
sits deeper into it. So a track that has been updated by both SHOULD sit behind its camera
measurement, by up to about the radar's own offset. This scores that directly: for every camera
measurement with a published track on it, find the radar return on the same bearing and ask
where the track sits on the measurement->radar segment.

  fraction 0.0 = on the camera measurement,  1.0 = on the radar return.
Anything inside [0, 1] is the filter blending two disagreeing sensors, which is its job.
Outside it is the filter being somewhere neither sensor put it.
"""
import math, bisect, argparse
import numpy as np
from mcap_ros2.reader import read_ros2_messages

MEAS = "/perception/measurements/camera_lidar_markers"
PUB = "/perception/objects_markers"
RAD = "/perception/measurements/radar_markers"

# MATCH ON HEADER STAMPS (see live_orphans.py for the full account). The aggregator stamps each
# published frame with the capture time of the measurement it just released, so a measurement and
# the frame carrying it share a stamp -- while their RECORDING times differ by a pipeline latency
# that is not constant and not equal between two arms. Two further rules come with it: sort by
# stamp ONLY and stably, so frames sharing a stamp keep their publication order, and take the LAST
# frame at a stamp, which is the one that has seen everything released at that instant.
# `--log-time` reproduces the numbers recorded before 2026-09-23.
USE_LOG_TIME = False

EGO_YAW_DEG = 5.35                      # lidar_tc -> ego, for a POINT
R_SL_YAW_DEG = 5.443                    # lidar_tc -> delphi_esr_radar, from the node's own log
T_SL = np.array([-2.964, 0.371])


def rot(deg):
    k = math.radians(deg)
    return np.array([[math.cos(k), -math.sin(k)], [math.sin(k), math.cos(k)]])


def radar_to_ego(p):
    """radar frame -> lidar_tc -> ego."""
    R_sl = rot(R_SL_YAW_DEG)
    p_l = R_sl.T @ (np.asarray(p, dtype=float) - T_SL)
    return rot(EGO_YAW_DEG) @ p_l


def centres(msg, kind):
    out = []
    for m in msg.markers:
        if m.type != kind or getattr(m, "action", 0) in (2, 3):
            continue
        p = m.pose.position
        out.append((p.x, p.y))
    return out


def _stamp(msg, log_time_ns):
    if USE_LOG_TIME or not msg.markers:
        return log_time_ns * 1e-9
    h = msg.markers[0].header.stamp
    return h.sec + h.nanosec * 1e-9


def load(path):
    meas, pub, rad = [], [], []
    for m in read_ros2_messages(path, topics=[MEAS, PUB, RAD]):
        t = _stamp(m.ros_msg, m.log_time_ns)
        if m.channel.topic == MEAS:
            c = [tuple(rot(EGO_YAW_DEG) @ np.array(p)) for p in centres(m.ros_msg, 1)]
            meas.append((t, c))
        elif m.channel.topic == PUB:
            pub.append((t, centres(m.ros_msg, 1)))
        else:
            rad.append((t, [tuple(radar_to_ego(p)) for p in centres(m.ros_msg, 2)]))
    for a in (meas, pub, rad):
        a.sort(key=lambda r: r[0])          # stable, by stamp only: ties keep publication order
    return meas, pub, rad


def nearest_frame(stamps, arr, t, tol):
    i = bisect.bisect_left(stamps, t)
    cand = [j for j in (i - 1, i) if 0 <= j < len(arr)]
    if not cand:
        return None
    j = min(cand, key=lambda k: abs(stamps[k] - t))
    if abs(stamps[j] - t) > tol:
        return None
    while j + 1 < len(arr) and stamps[j + 1] == stamps[j]:
        j += 1                              # the last frame at this stamp has seen everything
    return arr[j][1]


def main(path):
    meas, pub, rad = load(path)
    pt = [t for t, _ in pub]
    rt = [t for t, _ in rad]
    print(f"{path}\n  camera {len(meas)}  published {len(pub)}  radar {len(rad)} frames")

    fracs, gaps, cam_only, alongs = [], [], [], []
    for t, cs in meas:
        tol = 0.15 if USE_LOG_TIME else 0.05
        tracks = nearest_frame(pt, pub, t, tol)
        returns = nearest_frame(rt, rad, t, tol)
        if not tracks:
            continue
        for (mx, my) in cs:
            d = [math.hypot(mx - qx, my - qy) for (qx, qy) in tracks]
            k = int(np.argmin(d))
            if d[k] >= 3.0:
                continue                                  # orphan; counted elsewhere
            tx, ty = tracks[k]
            r_m = math.hypot(mx, my)
            # The radar return on this bearing, if any. Picked by BEARING alone: picking the
            # one closest in range would force the measured range gap toward zero and answer
            # the question with its own assumption.
            best = None
            for (rx, ry) in (returns or []):
                db = abs(math.degrees(math.atan2(ry, rx) - math.atan2(my, mx)))
                if db < 1.5 and (best is None or db < best[0]):
                    best = (db, rx, ry)
            # signed along-ray displacement of the track from the measurement
            ux, uy = mx / max(r_m, 1e-6), my / max(r_m, 1e-6)
            along = (tx - mx) * ux + (ty - my) * uy
            if best is None:
                cam_only.append(along)
                continue
            _, rx, ry = best
            r_gap = (rx - mx) * ux + (ry - my) * uy       # radar's own offset from the camera
            gaps.append(r_gap)
            alongs.append(along)
            if abs(r_gap) > 0.3:
                fracs.append(along / r_gap)

    f = np.asarray(fracs); g = np.asarray(gaps); c = np.asarray(cam_only)
    al = np.asarray(alongs)
    print(f"  tracks with a radar return on the same bearing: {g.size}")
    if g.size:
        print(f"    radar sits {np.median(g):+.2f} m from the camera measurement along the ray"
              f"  (p25 {np.percentile(g, 25):+.2f}  p75 {np.percentile(g, 75):+.2f})")
    if al.size:
        print(f"    those tracks sit {np.median(al):+.2f} m from their camera measurement "
              f"along the ray (|offset| median {np.median(np.abs(al)):.2f} m)")
    if f.size:
        print(f"    track position on the camera->radar segment: median {np.median(f):.2f}"
              f"   inside [0,1]: {100 * np.mean((f >= 0) & (f <= 1)):.1f}%"
              f"   beyond the radar: {100 * np.mean(f > 1):.1f}%"
              f"   behind the camera: {100 * np.mean(f < 0):.1f}%")
    if c.size:
        print(f"  tracks with NO radar return (the filter is on its own): n={c.size}  "
              f"along-ray offset median {np.median(c):+.2f} m  "
              f"|offset| median {np.median(np.abs(c)):.2f} m")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recordings", nargs="+")
    ap.add_argument("--log-time", action="store_true",
                    help="match on RECORDING time instead of header stamps, reproducing the "
                         "numbers recorded before 2026-09-23 and their bias")
    args = ap.parse_args()
    USE_LOG_TIME = args.log_time
    print("  matching on " + ("RECORDING time (pre-2026-09-23)" if USE_LOG_TIME
                              else "header stamps"))
    for p in args.recordings:
        main(p)
        print()
