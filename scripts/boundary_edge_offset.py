#!/usr/bin/env python3
"""X3: where do SAM3 and SphereFormer each put the road edge, and why do they disagree?

Uses a recording of road_boundary's inputs (scripts/road_boundary_ab.py header, or the
boundary_eval category). For every SAM3 mask it lifts the road mask onto the ground-flagged
projection captured closest to it, and takes SphereFormer's road from the labelled sweep closest
in time; both edges are then the per-bin lateral extremes in the vehicle frame (the same rules
the upstream nodes use on this branch: road_mask_boundary.lift_mask_boundary and
sphereformer_boundary.fixed_bin_boundary).

  python3 scripts/boundary_edge_offset.py REC/REC_0.mcap [--bag ORIGINAL_BAG --overlay OUT.png]

Reading the offsets (SAM3 minus SphereFormer, per side, against range):
  * left negative AND right positive  -> SAM3's road is NARROWER on both sides: the two models
    disagree on what counts as road (a shoulder), or SAM3's mask edge is conservative.
  * both sides the same sign, growing with range -> an angular (yaw) camera-LiDAR error.
  * both sides the same sign, constant -> a lateral offset.
With --bag, the camera frame closest to the median frame is drawn with SAM3's edge (green) and
SphereFormer's edge (magenta) projected through the camera calibration, to look at directly.
"""

from __future__ import annotations

import argparse
import io
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
sys.path[:0] = [os.path.join(REPO, "src", "object_fusion"), os.path.join(REPO, "src", "perception_common"),
                os.path.join(REPO, "src", "sphereformer_ros"),
                os.path.join(REPO, "src", "SAM3_ROS_NODE", "src", "sam3_ros")]

from mcap_ros2.reader import read_ros2_messages  # noqa: E402
from PIL import Image  # noqa: E402

from object_fusion.flagged_cloud import decode_fields  # noqa: E402
from object_fusion.road_boundary import decode_semkitti_rgb  # noqa: E402
from road_mask_boundary import ego_component, lift_mask_boundary  # noqa: E402
from sphereformer_boundary import (  # noqa: E402
    OTHER_MIN_CLASS, ROAD, fixed_bin_boundary, lidar_to_vehicle,
)

YAW = -5.35
SEG, PROJ, MASK, INFO = ("/sphereformer_road_segmentation", "/perception/lidar_2d_projection_ground",
                         "/segmentation/mask_image/compressed", "/camera_fl/camera_info")
STATIONS = (10.0, 15.0, 20.0, 25.0, 30.0, 40.0)


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def station_y(pts, x0, half=1.5):
    if len(pts) == 0:
        return np.nan
    v = lidar_to_vehicle(pts[:, :2], YAW)
    k = np.abs(v[:, 0] - x0) <= half
    return float(np.median(v[k, 1])) if k.any() else np.nan


def sphereformer_edges(msg):
    f = decode_fields(msg, ("x", "y", "z", "rgb"))
    xyz = np.stack([f["x"], f["y"], f["z"]], 1)
    ok = np.isfinite(xyz).all(1)
    xyz, cls = xyz[ok], decode_semkitti_rgb(f["rgb"][ok])
    return fixed_bin_boundary(xyz[cls == ROAD], other_xy=xyz[cls >= OTHER_MIN_CLASS, :2],
                              ego_yaw_deg=YAW)


def sam3_edges(mask_msg, proj_msg, image_wh):
    mask = np.asarray(Image.open(io.BytesIO(bytes(mask_msg.data))).convert("L"))
    f = decode_fields(proj_msg, ("x", "y", "z", "u", "v", "ground"))
    xyz = np.stack([f["x"], f["y"], f["z"]], 1)
    ok = np.isfinite(xyz).all(1)
    g = None if f["ground"] is None else f["ground"][ok] > 0.5
    road = ego_component(mask >= 1)
    return lift_mask_boundary(xyz[ok], f["u"][ok], f["v"][ok], road, image_wh, ground=g,
                              ego_yaw_deg=YAW), mask


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap")
    ap.add_argument("--bag", help="original bag with /camera_fl/image, for the overlay")
    ap.add_argument("--overlay", default="edge_overlay.png")
    args = ap.parse_args()

    segs, projs, masks, image_wh = [], [], [], None
    for m in read_ros2_messages(args.mcap, topics=[SEG, PROJ, MASK, INFO]):
        t, msg = m.channel.topic, m.ros_msg
        if t == INFO and image_wh is None:
            image_wh = (int(msg.width), int(msg.height))
        elif t == SEG:
            segs.append((stamp(msg.header), msg))
        elif t == PROJ:
            projs.append((stamp(msg.header), msg))
        elif t == MASK:
            masks.append((stamp(msg.header), msg))
    st = np.array([s for s, _ in segs])
    pt = np.array([s for s, _ in projs])
    rows = []
    for t, mmsg in masks:
        i, j = int(np.argmin(np.abs(pt - t))), int(np.argmin(np.abs(st - t)))
        if abs(pt[i] - t) > 0.08 or abs(st[j] - t) > 0.1:
            continue
        (sl, sr), _ = sam3_edges(mmsg, projs[i][1], image_wh)
        fl, fr = sphereformer_edges(segs[j][1])
        rows.append((t, [(station_y(sl, x), station_y(sr, x), station_y(fl, x), station_y(fr, x))
                         for x in STATIONS]))
    a = np.array([r[1] for r in rows])                     # frames x stations x 4
    print(f"{os.path.basename(args.mcap)}: {len(rows)} paired frames\n")
    print(f"{'station':>8s} {'SAM3 L':>8s} {'SF L':>8s} {'dL':>7s}   {'SAM3 R':>8s} {'SF R':>8s} {'dR':>7s}   "
          f"{'width SAM3':>10s} {'width SF':>9s}")
    for k, x in enumerate(STATIONS):
        sl, sr, fl, fr = (np.nanmedian(a[:, k, c]) for c in range(4))
        dl = np.nanmedian(a[:, k, 0] - a[:, k, 2])
        dr = np.nanmedian(a[:, k, 1] - a[:, k, 3])
        print(f"{x:7.0f}m {sl:8.2f} {fl:8.2f} {dl:+7.2f}   {sr:8.2f} {fr:8.2f} {dr:+7.2f}   "
              f"{sl - sr:10.2f} {fl - fr:9.2f}")
    print("\n dL < 0 and dR > 0: SAM3 narrower on both sides (definition / conservative mask);"
          "\n same sign on both sides: a calibration shift (growing with range = yaw).")

    if args.bag:
        from object_fusion.projection import project_to_pixels
        t_mid = rows[len(rows) // 2][0]
        best = None
        for m in read_ros2_messages(args.bag, topics=["/camera_fl/image"]):
            d = abs(stamp(m.ros_msg.header) - t_mid)
            if best is None or d < best[0]:
                best = (d, m.ros_msg)
        img_msg = best[1]
        raw = np.frombuffer(bytes(img_msg.data), np.uint8).reshape(img_msg.height, img_msg.width, -1)
        if raw.shape[2] == 1:   # Bayer RGGB: a cheap half-resolution debayer is enough to look at
            r, g1, g2, b = raw[0::2, 0::2, 0], raw[0::2, 1::2, 0], raw[1::2, 0::2, 0], raw[1::2, 1::2, 0]
            rgb = np.stack([r, ((g1.astype(int) + g2) // 2).astype(np.uint8), b], -1)
            rgb = np.array(Image.fromarray(rgb).resize((img_msg.width, img_msg.height)))
        else:
            rgb = raw[:, :, :3][:, :, ::-1].copy()
        mi = min(range(len(masks)), key=lambda q: abs(masks[q][0] - t_mid))
        i = int(np.argmin(np.abs(pt - masks[mi][0])))
        j = int(np.argmin(np.abs(st - masks[mi][0])))
        (sl, sr), mask = sam3_edges(masks[mi][1], projs[i][1], image_wh)
        fl, fr = sphereformer_edges(segs[j][1])
        out = rgb.copy()
        mh, mw = mask.shape
        big = np.array(Image.fromarray(((mask >= 1) * 255).astype(np.uint8)).resize((img_msg.width, img_msg.height)))
        out[big > 0] = (0.65 * out[big > 0] + 0.35 * np.array([60, 140, 255])).astype(np.uint8)
        for pts, col in ((sl, (40, 230, 60)), (sr, (40, 230, 60)), (fl, (255, 40, 220)), (fr, (255, 40, 220))):
            if len(pts) == 0:
                continue
            _, u, v = project_to_pixels(np.asarray(pts, float), image_wh=(img_msg.width, img_msg.height))
            for uu, vv in zip(u.astype(int), v.astype(int)):
                out[max(0, vv - 5):vv + 6, max(0, uu - 5):uu + 6] = col
        Image.fromarray(out).save(args.overlay)
        print(f"\noverlay: {args.overlay}  (blue tint = SAM3 road mask, green = SAM3 edge, "
              f"magenta = SphereFormer edge)")


if __name__ == "__main__":
    main()
