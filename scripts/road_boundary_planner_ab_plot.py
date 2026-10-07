#!/usr/bin/env python3
"""Plots for the planner-level A/B (scripts/road_boundary_planner_ab.py).

  python3 scripts/road_boundary_planner_ab_plot.py REC1/REC1_0.mcap [REC2/REC2_0.mcap ...] --out DIR

Writes three PNGs per run:
  1_timeseries.png  lmp_v10c's left/right edge and centreline at 20 m over the drive, raw input vs
                    road_boundary input, with frames that have a vehicle on that side shaded.
  2_summary.png     share of obstacle frames pulled in > 0.5 m, and p95 centreline shift, per
                    station, per bag.
  3_snapshot.png    top-down view of the frame where raw input was pulled in furthest at 20 m:
                    both lmp_v10c outputs, the fused objects, and each arm's obstacle-free edge.
"""

from __future__ import annotations

import argparse
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import road_boundary_planner_ab as ab  # noqa: E402
from mcap_ros2.reader import read_ros2_messages  # noqa: E402

from object_fusion import frames  # noqa: E402

# Reference palette, light mode (slots 1 and 2 validate all-pairs; line style is the second cue).
SURFACE, INK, INK2, GRID, SHADE = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df", "#d9d8d3"
ARM_STYLE = {
    "raw": dict(color="#eb6834", ls="--", label="lmp_v10c on raw boundaries (today)"),
    "rb": dict(color="#2a78d6", ls="-", label="lmp_v10c on road_boundary"),
}

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10,
    "axes.titlesize": 11, "axes.titleweight": "bold", "legend.frameon": False,
})


def series_at(series, arm, station_idx):
    L, R = series[arm]["left"], series[arm]["right"]
    tl = np.array([t for t, _ in L])
    yl = np.array([y[station_idx] for _, y in L])
    rt = np.array([t for t, _ in R])
    yr_all = np.array([y[station_idx] for _, y in R])
    yr = np.array([yr_all[np.argmin(np.abs(rt - t))] if len(rt) and np.min(np.abs(rt - t)) < 0.05
                   else np.nan for t in tl])
    return tl, yl, yr


def shade_runs(ax, t, mask, t0):
    """Grey bands over contiguous runs of obstacle frames."""
    if not mask.any():
        return
    idx = np.flatnonzero(mask)
    runs = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
    for r in runs:
        ax.axvspan(t[r[0]] - t0, t[r[-1]] - t0 + 0.05, color=SHADE, alpha=0.6, lw=0, zorder=0)


def plot_timeseries(runs, out):
    si = ab.STATIONS.index(20.0)
    fig, axes = plt.subplots(3, len(runs), figsize=(7.2 * len(runs), 8.4), sharex="col",
                             squeeze=False)
    for c, (name, series, objs) in enumerate(runs):
        t0 = min(series[a]["left"][0][0] for a in ARM_STYLE if series[a]["left"])
        for arm, st in ARM_STYLE.items():
            t, yl, yr = series_at(series, arm, si)
            obs = np.array([ab.obstacle_side(x, objs) for x in t])
            ref_l = np.nanmedian(yl[~obs[:, 0]])
            ref_r = np.nanmedian(yr[~obs[:, 1]])
            ctr = 0.5 * (yl + yr)
            ref_c = 0.5 * (ref_l + ref_r)
            for r, (y, ref, m) in enumerate(((yl, ref_l, obs[:, 0]), (yr, ref_r, obs[:, 1]),
                                             (ctr, ref_c, obs[:, 0] | obs[:, 1]))):
                ax = axes[r, c]
                if arm == "raw":
                    shade_runs(ax, t, m, t0)
                ax.plot(t - t0, y, color=st["color"], ls=st["ls"], lw=2, label=st["label"])
                ax.axhline(ref, color=st["color"], lw=1, ls=":", alpha=0.8)
        axes[0, c].set_title(f"{name}\nLeft edge at 20 m")
        axes[1, c].set_title("Right edge at 20 m")
        axes[2, c].set_title("Centreline at 20 m (what the planner drives on)")
        for r in range(3):
            axes[r, c].set_ylabel("lateral y in ego frame [m]")
        axes[2, c].set_xlabel("time [s]")
    # One legend, plus a key for the shading and the reference lines.
    handles, labels = axes[0, 0].get_legend_handles_labels()
    handles += [plt.Rectangle((0, 0), 1, 1, color=SHADE, alpha=0.6),
                plt.Line2D([], [], color=INK2, ls=":", lw=1)]
    labels += ["vehicle on that side of the road (3-45 m ahead)",
               "that arm's median edge on obstacle-free frames"]
    fig.legend(handles, labels, loc="lower center", ncol=2, bbox_to_anchor=(0.5, -0.005))
    fig.suptitle("Stationary vehicle, traffic passing: does a passing car pull the planner's edge in?\n"
                 "Left edge dipping DOWN or right edge going UP inside a grey band = pulled toward "
                 "the centre.\nBlue's two-level flicker = lmp_v10c switching between SAM3 and "
                 "SphereFormer, which disagree by 0.4-0.75 m",
                 fontsize=12, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0.06, 1, 0.92))
    fig.savefig(out, dpi=130)
    plt.close(fig)


def plot_summary(results, out):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
    xs = np.arange(len(ab.STATIONS))
    width = 0.19
    bags = list(results)
    hatches = ["", "//"]
    for b, bag in enumerate(bags):
        for a, arm in enumerate(ARM_STYLE):
            rows = results[bag][arm]
            off = (b * 2 + a - 1.5) * (width + 0.02)
            pulled = [100 * rows[x]["gt05"] for x in ab.STATIONS]
            cshift = [rows[x]["centre_p95"] for x in ab.STATIONS]
            kw = dict(width=width, color=ARM_STYLE[arm]["color"], edgecolor=SURFACE, linewidth=2,
                      hatch=hatches[b], label=f"{bag}: {arm}")
            axes[0].bar(xs + off, pulled, **kw)
            axes[1].bar(xs + off, cshift, **kw)
    axes[0].set_title("Obstacle frames with the edge pulled in > 0.5 m")
    axes[0].set_ylabel("share of obstacle frames [%]")
    axes[1].set_title("Centreline shift in obstacle frames, p95")
    axes[1].set_ylabel("|centreline - obstacle-free centreline| [m]")
    for ax in axes:
        ax.set_xticks(xs, [f"{x:.0f} m" for x in ab.STATIONS])
        ax.set_xlabel("look-ahead station")
        ax.grid(axis="x", visible=False)
    axes[0].legend(loc="upper right", fontsize=9)
    fig.text(0.5, 0.005, "At 10 m, lmp_v10c on raw input has no boundary in 92-95% of frames "
             "(SphereFormer is used only beyond 15 m), so its 10 m bars rest on very few frames.",
             ha="center", color=INK2, fontsize=9)
    fig.suptitle("Lower is better; orange = raw input (today), blue = road_boundary input; "
                 "hatched = second bag", fontsize=11, color=INK)
    fig.tight_layout(rect=(0, 0.04, 1, 0.93))
    fig.savefig(out, dpi=130)
    plt.close(fig)


def plot_snapshot(path, name, series, objs, out):
    si = ab.STATIONS.index(20.0)
    t, yl, yr = series_at(series, "raw", si)
    obs = np.array([ab.obstacle_side(x, objs) for x in t])
    ref_l = np.nanmedian(yl[~obs[:, 0]])
    ref_r = np.nanmedian(yr[~obs[:, 1]])
    inward = np.where(obs[:, 0], ref_l - yl, -np.inf)
    inward = np.maximum(inward, np.where(obs[:, 1], yr - ref_r, -np.inf))
    k = int(np.nanargmax(inward))
    t_star = t[k]
    # Pull the full paths of both arms nearest to t_star.
    want = {f"/ab/{a}/road_{s}": None for a in ARM_STYLE for s in ("left", "right")}
    best = {k_: (np.inf, None) for k_ in want}
    for m in read_ros2_messages(path, topics=list(want)):
        d = abs(ab.stamp(m.ros_msg.header) - t_star)
        if d < best[m.channel.topic][0]:
            best[m.channel.topic] = (d, m.ros_msg)
    ts = np.array([o[0] for o in objs])
    obj_xy = objs[int(np.argmin(np.abs(ts - t_star)))][1]

    fig, ax = plt.subplots(figsize=(11, 4.6))
    for arm, st in ARM_STYLE.items():
        for side in ("left", "right"):
            msg = best[f"/ab/{arm}/road_{side}"][1]
            if msg is None or not msg.poses:
                continue
            e = frames.lidar_to_ego(np.array([(p.pose.position.x, p.pose.position.y)
                                              for p in msg.poses]))
            e = e[np.argsort(e[:, 0])]
            ax.plot(e[:, 0], e[:, 1], color=st["color"], ls=st["ls"], lw=2.4,
                    label=st["label"] if side == "left" else None)
    ax.axhline(ref_l, color=INK2, ls=":", lw=1)
    ax.axhline(ref_r, color=INK2, ls=":", lw=1, label="obstacle-free edge (raw arm's median)")
    ax.scatter(obj_xy[:, 0], obj_xy[:, 1], s=140, marker="s", color=INK, zorder=5,
               label="fused objects (vehicles)")
    ax.scatter([0], [0], s=160, marker=">", color=INK2, zorder=5)
    ax.annotate("ego", (0, 0), xytext=(1.2, -0.9), color=INK2)
    ax.axvline(20, color=GRID, lw=1.5)
    ax.annotate("20 m station", (20, ax.get_ylim()[1]), xytext=(20.5, ref_l + 2.2), color=INK2)
    ax.set_xlim(0, 60)
    ax.set_ylim(min(ref_r, -4) - 4, max(ref_l, 4) + 4)
    ax.set_aspect("equal")
    ax.set_xlabel("x ahead [m] (ego frame)")
    ax.set_ylabel("y left [m]")
    ax.set_title(f"{name}: the frame where raw input was pulled in furthest at 20 m "
                 f"({inward[k]:.2f} m), t = {t_star - t[0]:.1f} s")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2, fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mcap", nargs="+")
    ap.add_argument("--out", default=os.path.expanduser("~/fusion_data/recordings/planner_ab_plots"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    runs, results = [], {}
    for p in args.mcap:
        name = os.path.basename(os.path.dirname(os.path.abspath(p))).replace("planner_ab_", "")
        series, objs, _ = ab.load(p)
        runs.append((name, series, objs))
        results[name] = ab.analyse(series, objs)
    plot_timeseries(runs, os.path.join(args.out, "1_timeseries.png"))
    plot_summary(results, os.path.join(args.out, "2_summary.png"))
    for p, (name, series, objs) in zip(args.mcap, runs):
        plot_snapshot(p, name, series, objs, os.path.join(args.out, f"3_snapshot_{name}.png"))
    print(args.out)


if __name__ == "__main__":
    main()
