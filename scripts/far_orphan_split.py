"""Among far (>80 m) orphans: is there no track at all, or one that is off the ray?

The 2026-09-22 numbers from this script ("nothing on the bearing" 22 -> 31-33 of 292) were
measured before `live_orphans.load` matched frames correctly, and they are retracted: there
is no far-band difference between the cluster arms. See HANDOFF item 7 and rule 4.
"""
import sys, math, bisect
import numpy as np
sys.path.insert(0, "scripts")
from live_orphans import load, RANGE_TRUST_MAX_M, FAR_ALONG_FRAC

for path in sys.argv[1:]:
    meas, pub = load(path)
    pub_t = [t for t, _ in pub]
    no_track = off_ray = too_far_along = 0
    orph = n = 0
    for t, cs in meas:
        i = bisect.bisect_left(pub_t, t)
        cand = [j for j in (i - 1, i) if 0 <= j < len(pub)]
        if not cand:
            continue
        j = min(cand, key=lambda k: abs(pub_t[k] - t))
        if abs(pub_t[j] - t) > 0.05:
            continue
        while j + 1 < len(pub) and pub_t[j + 1] == pub_t[j]:
            j += 1                          # the last frame at a stamp; see live_orphans.py
        tracks = pub[j][1]
        for (x, y) in cs:
            r = math.hypot(x, y)
            if r < RANGE_TRUST_MAX_M:
                continue
            n += 1
            if not tracks:
                orph += 1; no_track += 1
                continue
            ux, uy = x / r, y / r
            d = min(math.hypot(x - qx, y - qy) for (qx, qy) in tracks)
            on_ray = [( (qx-x)*ux + (qy-y)*uy, -(qx-x)*uy + (qy-y)*ux ) for (qx, qy) in tracks]
            covered = d < 3.0 or any(abs(c) < 3.0 and abs(a) < FAR_ALONG_FRAC * r for a, c in on_ray)
            if covered:
                continue
            orph += 1
            if any(abs(c) < 3.0 for a, c in on_ray):
                too_far_along += 1          # on the bearing, but way off in range
            else:
                off_ray += 1                # nothing on this bearing at all
    print(f"{path.split('/')[-2]:26s} far n={n:4d}  orphans {orph:3d} ({100*orph/max(n,1):4.1f}%): "
          f"no published track at all {no_track:3d} | nothing on the bearing {off_ray:3d} | "
          f"on the bearing but out of range {too_far_along:3d}")
