"""Read INSCONFIG out of a bag: the receiver's own lever arms, including the USER output point."""
import glob, sys, os
from mcap_ros2.reader import read_ros2_messages
bag = sys.argv[1]
files = sorted(glob.glob(os.path.join(bag, "*.mcap"))) if os.path.isdir(bag) else [bag]
for f in files:
    for m in read_ros2_messages(f, topics=["/novatel/oem7/insconfig"]):
        msg = m.ros_msg
        print(f"--- {f}")
        for field in ("imu_type", "mapping", "initial_alignment_mode", "gnss_seed_enabled",
                      "number_of_translations", "number_of_rotations", "alignment_status"):
            if hasattr(msg, field):
                print(f"  {field}: {getattr(msg, field)}")
        for tr in getattr(msg, "translations", []):
            print(f"  TRANSLATION frame={getattr(tr.frame, 'frame', tr.frame)} "
                  f"source={getattr(tr.translation_source, 'status', tr.translation_source)} "
                  f"xyz=({tr.x:+.3f}, {tr.y:+.3f}, {tr.z:+.3f}) "
                  f"sd=({tr.x_stdev:.3f}, {tr.y_stdev:.3f}, {tr.z_stdev:.3f})")
        for ro in getattr(msg, "rotations", []):
            print(f"  ROTATION frame={getattr(ro.frame, 'frame', ro.frame)} "
                  f"rpy=({ro.x:+.3f}, {ro.y:+.3f}, {ro.z:+.3f})")
        sys.exit(0)
print("no INSCONFIG message found")
