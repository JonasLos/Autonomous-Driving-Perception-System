"""Cross-ray (lateral) spread of the camera+LiDAR position, using radar azimuth as the ruler.

lateral error ~ r * sin(az_obj - az_radar). The ESR's own azimuth is ~0.5 deg (0.0087*r m), so the
measured spread bounds the object's cross-ray sigma: sigma_obj = sqrt(max(0, meas^2 - radar^2)).
"""
import pickle, math, sys
import numpy as np
sys.path.insert(0, "scripts"); sys.path.insert(0, "src/object_fusion")
import ground_ab as G
from object_fusion.tracker import sigma_cross

rows = pickle.load(open(sys.argv[1], "rb"))
arm = sys.argv[2] if len(sys.argv) > 2 else "DropNF"
print(f"{'band':>10s} {'n':>5s} {'median':>8s} {'meas sd':>8s} {'radar sd':>9s} {'-> object sd':>12s}"
      f" {'filter assumes':>14s}")
for lo, hi in ((15, 25), (25, 35), (35, 45), (45, 55), (55, 65), (65, 80), (80, 100)):
    lat = []
    for r in rows:
        if r[arm] is None or r.get("radar_az") is None:
            continue
        rr = math.hypot(*r["pub"])
        if not (lo <= rr < hi):
            continue
        lat.append(float(r["radar"]) * math.radians(r["obj_az"] - r["radar_az"]))
    a = np.asarray(lat)
    if a.size < 10:
        continue
    med = float(np.median(a))
    sd = 1.4826 * float(np.median(np.abs(a - med)))
    mid = 0.5 * (lo + hi)
    radar_sd = math.radians(0.5) * mid
    obj = math.sqrt(max(0.0, sd ** 2 - radar_sd ** 2))
    print(f"  {lo:3d}-{hi:<4d} {a.size:5d} {med:+8.2f} {sd:8.2f} {radar_sd:9.2f} {obj:12.2f}"
          f" {sigma_cross(mid):14.2f}")
