"""Past RANGE_TRUST_MAX_M the filter drops the camera's along-ray component so radar owns range.
So: how often is radar actually there? This counts provenance by range band from a recording of
/perception/objects (scripts/run_planner_ab_nodes.sh makes one, or record the topic directly).

    python3 scripts/far_field_support.py ~/fusion_data/recordings/planner_ab

Measured 2026-09-17: radar has NEVER touched 41% of tracks at 80-100 m and 57% at 100-150 m, so
for those the range is dead reckoning from their birth value. See HANDOFF item 6.
"""
import glob, os, sys, math
from collections import defaultdict
from mcap_ros2.reader import read_ros2_messages

CONTRIB_RADAR = 4
bands = [(0, 25), (25, 40), (40, 60), (60, 80), (80, 100), (100, 150), (150, 250)]
agg = defaultdict(lambda: dict(n=0, radar_ever=0, radar_now=0, dropped=0, coasting=0, ids=set()))
files = sorted(glob.glob(os.path.join(sys.argv[1], "*.mcap")))
for f in files:
    for m in read_ros2_messages(f, topics=["/perception/objects"]):
        for o in m.ros_msg.objects:
            r = math.hypot(o.pose.position.x, o.pose.position.y)
            b = next((b for b in bands if b[0] <= r < b[1]), None)
            if b is None:
                continue
            a = agg[b]
            a["n"] += 1
            a["ids"].add(o.track_id)
            a["radar_ever"] += bool(o.contributions & CONTRIB_RADAR)
            a["radar_now"] += bool(o.contributions_this_frame & CONTRIB_RADAR)
            a["dropped"] += bool(o.camera_range_dropped)
            a["coasting"] += (o.track_status == 2)
print(f"  {'band':>9s} {'obs':>7s} {'tracks':>7s} {'radar ever':>11s} {'radar now':>10s}"
      f" {'range dropped':>14s} {'coasting':>9s}")
for b in bands:
    a = agg.get(b)
    if not a or not a["n"]:
        continue
    n = a["n"]
    print(f"  {b[0]:4d}-{b[1]:<4d} {n:7d} {len(a['ids']):7d} {100*a['radar_ever']/n:10.1f}%"
          f" {100*a['radar_now']/n:9.1f}% {100*a['dropped']/n:13.1f}% {100*a['coasting']/n:8.1f}%")
