"""Publish cadence and track count per published frame, per arm."""
import sys
import numpy as np
sys.path.insert(0, "scripts")
from live_orphans import load

for path in sys.argv[1:]:
    _, pub = load(path)
    t = np.asarray([p[0] for p in pub])
    n = np.asarray([len(p[1]) for p in pub])
    dt = np.diff(t)
    dt = dt[(dt > 0) & (dt < 5)]
    print(f"{path.split('/')[-2]:26s} frames {len(pub):6d} over {t[-1]-t[0]:6.1f} s "
          f"= {len(pub)/(t[-1]-t[0]):5.1f} Hz | dt median {np.median(dt)*1e3:5.1f} ms "
          f"p90 {np.percentile(dt,90)*1e3:6.1f} | tracks/frame mean {n.mean():4.2f} "
          f"median {np.median(n):.0f} max {n.max():3d} | empty frames {100*np.mean(n==0):4.1f}%")
