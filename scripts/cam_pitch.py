"""How far off is the camera ground-intercept, and is it ONE angle?

For a box whose bottom edge sits where the object meets the road, the ray's depression angle must
satisfy tan(theta) = h / r, with h the camera height above that patch of road. So per detection:

    delta = theta_ray - atan(h / r_lidar)

A constant delta over range means a single angle offset -- camera pitch mis-calibration and a
systematic box-bottom offset are the SAME parameter (an offset of dv pixels is dv/f radians), and
one number fixes both. A delta that drifts with range means the camera height (or the flat-road
assumption) is wrong instead.
"""
import pickle, math, sys
import numpy as np
sys.path.insert(0, "scripts"); sys.path.insert(0, "src/object_fusion"); sys.path.insert(0, "src/perception_common")
import ground_ab as G
from object_fusion.projection import pixel_ray

rows = pickle.load(open(sys.argv[1], "rb"))
H_CAM = 2.46 - 0.8425                      # LiDAR height above road minus camera z in lidar frame
FX = 3461.179

sel = [r for r in rows if r["DropNF"] is not None and r["cls"] in ("car", "truck", "bus")]
print(f"vehicles with a LiDAR position: {len(sel)}")
print(f"{'band':>10s} {'n':>5s} {'delta deg':>10s} {'sd':>6s} {'= px':>7s} {'implied err at band':>20s}")
for lo, hi in ((10, 20), (20, 30), (30, 40), (40, 50), (50, 65), (65, 80)):
    d = []
    for r in sel:
        x, y = r["DropNF"]
        rng = math.hypot(x, y)
        if not (lo <= rng < hi):
            continue
        bu, bv, bw, bh = r["box"]
        o, dirv = pixel_ray(bu, bv + bh / 2.0)
        if dirv[2] >= -1e-6:
            continue
        theta_ray = math.atan2(-dirv[2], math.hypot(dirv[0], dirv[1]))
        theta_true = math.atan(H_CAM / rng)
        d.append(math.degrees(theta_ray - theta_true))
    a = np.asarray(d)
    if a.size < 10:
        continue
    med = float(np.median(a))
    sd = 1.4826 * float(np.median(np.abs(a - med)))
    mid = 0.5 * (lo + hi)
    # a delta of this size moves the intercept by dr = r^2 * delta / h
    dr = mid * mid * math.radians(med) / H_CAM
    print(f"  {lo:3d}-{hi:<4d} {a.size:5d} {med:+10.3f} {sd:6.3f} {math.radians(med)*FX:+7.1f}"
          f" {dr:+19.2f} m")
