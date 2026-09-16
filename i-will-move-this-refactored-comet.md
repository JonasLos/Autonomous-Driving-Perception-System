# Center-patch fusion mode — A/B result and next steps

**Status: hypothesis tested and largely disproved as originally framed. Do not ship the
center-patch rule as specified.** The investigation found three separate errors in the
original proposal's premises, and one genuine bug elsewhere in the pipeline that mattered far
more than the rule being tested. A narrower version of the idea survives and is worth one
more look; see *Plan going forward*.

Nothing has been committed. All changes are working-tree only.

---

## 1. Original hypothesis

Object fusion placed objects well under the ROS 1 stack and places them systematically **too
near** under ROS 2. Calibration was already ruled out (`K` matches `PROJ`, `/tf_static
lidar_tc→camera_fl` matches `T1`, same 2064×1544 pixel space). The suspected cause was the
pixel→point association rule:

| | rule |
|---|---|
| ROS 1 `objects_transform.py:27,141-146` | `pixel_lim = 20`: ±20 px around the bbox **center**, one point, drop the detection when empty (`if idx.size > 0`) |
| ROS 2 `fusion_node.py:403-423` | **every** point in the full bbox → `reject_ground` → `foreground_points` → median |

The proposal was to restore the center-patch rule, on the argument that a 40 px window cannot
span two LiDAR ring bands and therefore "structurally cannot see the road".

## 2. Three premises that turned out to be wrong

### 2.1 The 78 px ring pitch does not exist

The whole structural argument rested on *"1.29° ring pitch at f = 3461 → 78 px constant image
spacing"*. Measured directly from the `ring` field of `/lidar_tc/velodyne_points`, the sensor
is a **VLP-32C with a deliberately non-uniform beam pattern**:

| rings | elevation | pitch | px at f = 3461 |
|---|---|---|---|
| 9–25 | −3.667° … +1.667° | **0.333°** | **20.1** |
| 6–8 | | 0.667° | 40.2 |
| 5 | | 1.106° | 66.8 |
| 0–4, 26–31 | | up to 9.36° | up to 570 |

Confirmed in a narrow image column (u ∈ [1500,1700]): adjacent-ring `v` gaps run 10–23 px near
the horizon. **1.29° is 40°/31 — the average over the full FOV, not the pitch anywhere a
distant vehicle is imaged.** Consequences:

- Near the horizon a ±20 px patch spans **~2 ring bands**, not one. There is no guarantee.
- The "39 px hard ceiling" has no basis and would span ~4 bands.
- **Risk 1 in the original document ("per-object output rate roughly halves") is unfounded**
  for the same reason — near the horizon the patch almost always catches a ring.

### 2.2 The mechanism is box height, not ring geometry

What a center window actually does is stay away from the box's **bottom rows**, where the road
returns were already measured to sit (`fusion_node.py` records them "pinned to the bottom 8% of
the box"). That is a box-height effect, so **protection weakens with range** rather than
staying constant: a 1.5 m car is 519 px tall at 10 m, 130 px at 40 m, 52 px at 100 m. The
original claim — *"tied to sensor geometry, not to object size — that is the whole reason it
works at 100 m as well as at 10 m"* — is backwards.

This is why a **fraction-of-box-height** arm (`F_q`: keep the top q of the box) was swept
alongside the fixed-pixel arms. It is the scale-free way to say the same thing.

### 2.3 The bag is 83% traffic cones

| class | detections | distinct tracks | median box height |
|---|---|---|---|
| cone | 598 | 12 | 112 px |
| **car** | **117 → 147** | **1** | 69 px |
| bus | 4 | 2 | 84 px |

The hypothesis is about *the road in front of a vehicle*. A cone stands ~0.5 m and sits **on**
the road, so its returns are legitimately at road height — `reject_ground`'s own docstring says
as much. The "height above local ground" metric therefore **penalises correct behaviour for 83%
of the sample**, and any pooled number is dominated by it. All results below are broken out by
class for this reason.

**The entire vehicle evidence base is one car track.**

---

## 3. The bug that actually mattered: the 100 m crop wall

`transform.py` cropped at `lim_x = [0, 100]`. The one car in this bag sits at **~105 m median
range**. The wall was therefore **cropping the car's own returns**, leaving only the nearest
sliver inside the 2D box.

Isolation test — start from the new 150 m cloud and re-apply each old stage on **identical
detections**:

| variant | car dets | pts in box | pts in ±20 px patch | patch empty | median range |
|---|---|---|---|---|---|
| new: 150 m, no voxel | 147 | 16 | 6 | **0%** | 105.7 m |
| + voxel 0.1 back on | 147 | 16 | 6 | 0% | 105.7 m |
| + 100 m wall back on | 121 | **7** | **0** | **56%** | 89.3 m |
| + both back (= old) | 121 | 7 | 0 | 56% | 89.3 m |

Two things follow:

1. **The voxel filter changes nothing for the car.** Measured retention is 98% at 25–50 m and
   **100% beyond 50 m** — the natural sampling out there (0.35 m azimuthal, 0.58 m between
   rings at 100 m) is already far coarser than a 10 cm leaf. It only thins the near field
   (31% kept at 0–10 m, 79% at 10–25 m).
2. **The 100 m wall was the whole effect.** The old "median car range 89.3 m" was never the
   car's range — it was the range of whatever slice of scene survived the wall inside the box.

**This invalidated the far-field conclusions of the first A/B run.** The 44–48% patch-arm yield
collapse originally attributed to the rule was an artifact of the wall.

### Other crop findings

- **z ceiling of +1 m** removes ~5,600 points/sweep. The LiDAR sits **2.46 m above the road**
  (measured from near-field ground returns), so +1 m is 3.46 m above the surface. Cones, cars,
  vans and a 3.2 m city bus survive; a **3.5 m box truck and anything taller has its upper body
  cropped** before fusion sees it. Plausibly why the 4 bus detections all sit at road height.
- **Lateral crop ±20 m is narrower than the camera FOV beyond ~67 m.** Horizontal half-FOV is
  16.6°, so the image spans ±0.298 × range: ±20 m at 67 m, ±45 m at 150 m. At 150 m the crop
  keeps only **45% of the image width**. Any range increase buys a progressively narrower cone
  unless `crop_lateral_limit` rises with it.
- **z floor of −3.5 m** is 1.04 m *below* the road. Harmless.
- **Latent:** `transform.py` divides by `uv1[2,:]` with no sign guard. The camera sits 1.21 m
  forward of the LiDAR, so points with lidar x ∈ [0, 1.2] are behind it. ROS 1 used
  `lim_x = [2.5, 100]` and was immune. Measured: **0 such points currently land inside the
  image**, so it is latent, not active.

---

## 4. What was built

| artifact | state |
|---|---|
| `fusion_node.py` refactor | working tree, **uncommitted** (submodule `PaavanBagla/Custom_YOLO_ROS` @ `221bd11`, branch `vehicle_deploy`) |
| `scripts/patch_ab.py` | new, untracked — offline scorer |
| `scripts/patch_ab_rviz.py` | new, untracked — RViz side-by-side replay |
| `config/patch_ab.rviz` | new, untracked — generated at runtime by the above |
| `src/transform.py` | modified — three new runtime parameters |
| `/home/avalocal/probe_bag` | 53 MB, recorded under the old 100 m + voxel settings |
| `/home/avalocal/probe_bag_150` | 64 MB, recorded under 150 m + no voxel |

**`fusion_node.py`** — `reject_ground(px, py, pz, *, min_range, margin, min_points)` and
`foreground_points` lifted to module level so the harness runs the node's own code rather than a
copy that can drift. `_reject_ground` is now a thin method passing the node's attributes. The
`1.29deg` paragraph in the docstring was corrected; its *conclusion* ("past ~67 m whether any
ring lands on the vehicle is luck") is ruled out by the corrected geometry, so the 16 no-return
detections it explained are flagged as **cause unidentified** rather than given a new story.

**`src/transform.py`** — three runtime parameters, following the existing
`apply_bounded_parameters` pattern:

| parameter | default | was |
|---|---|---|
| `crop_max_range` | **150.0** | hardcoded 100 |
| `crop_lateral_limit` | 20.0 | hardcoded 20 |
| `voxel_size` | **0.0 (off)** | hardcoded 0.1 |

Bounds are re-read per sweep, so `ros2 param set` takes effect on the next cloud. Rollback
needs no edit:
`ros2 param set /lidar_to_2d_projection crop_max_range 100.0` / `voxel_size 0.1`.

Measured live: **9.982 Hz** on `/lidar_2d_projection` (std dev 3.5 ms), ~2,880 points/sweep
(up from 2,159), and `process_pointcloud` **8.5 ms → 3.4–4.6 ms** — removing the voxel filter
made the node *faster*, since `np.unique` over an Nx3 int array cost more than it saved.

### Harness self-check (both runs)

```
projection-stamp mismatches : 0
position mismatches         : 0 of 717 / 748 compared (worst 0.00e+00 m)
=> arm A reproduces the recorded /fused_bbox
```

Arm A reproduces the node bit-for-bit, which is also the regression test for the refactor.
`pytest src/perception_common/test/` → 32 passed.

---

## 5. Results

Arms: **A** = the node today (full box + `reject_ground`). **box** = full box, ground rejection
off. **B_Npx** = ±N px center patch. **F_q** = top q of the box. `dmed` = paired range change
vs A. `jumps` = frame-to-frame range changes > 5 m per track.

### Car — run 1, with the 100 m wall (117 det, 1 track) — **now known to be corrupted by the wall**

```
   arm      yield    n    range  height  >0.4m  pts    dmed        jumps
     A     100.0%  117    89.29    0.72    98%    5     n/a      1/116
   box     100.0%  117    82.98    0.01     2%    4   +0.00      4/116
B_20px      45.3%   53    92.42    0.68    92%    4   +0.00       0/52
B_39px     100.0%  117    82.98    0.01     2%    4   +0.00      4/116
 F_0.4      44.4%   52    92.34    1.06   100%    5   +0.01       0/51
 F_0.6      45.3%   53    92.14    0.80   100%    6   +0.00       0/52
```

### Car — run 2, 150 m wall, voxel off (147 det, 1 track) — **the valid one**

```
   arm      yield    n    range  height  >0.4m  pts    dmed        jumps
     A     100.0%  147   102.49    0.67    97%    5     n/a      6/146
   box     100.0%  147    87.24    0.03    12%    3  -14.38     12/146
B_20px     100.0%  147   104.98    0.66    74%    4   -0.00      0/146
B_39px     100.0%  147    87.24    0.02    12%    3  -14.27     12/146
 F_0.4      91.2%  134   104.89    0.88   100%    5   +0.00      0/133
 F_0.6     100.0%  147   104.98    0.68    99%    5   +0.00      0/146
```

Readings:

- **The car is now tracked past 100 m.** 94 of 147 detections fall in the 100–130 m band — the
  majority of the track was previously invisible or mis-placed.
- **`reject_ground` is doing the heavy lifting**, and is worth **−14.4 m**: `box` and `B_39px`
  (both effectively "full box, no ground rejection") collapse to 0.03 m / 12% elevated, while A
  sits at 0.67 m / 97%.
- **Every patch/fraction arm now places identically to A** (`dmed` ±0.00) — the near-bias the
  proposal set out to remove **is not present in arm A's output**.
- **But they remove all 6 of A's track jumps.** `F_0.6` does it at 100% yield and 99% elevated;
  `B_20px` at 100% yield. That is the one genuine, reproducible improvement found.

### Cone — run 2 (598 det, 12 tracks). A cone at road height is CORRECT.

```
   arm      yield    n    range  height  >0.4m  pts    dmed        jumps
     A     100.0%  598    22.83    0.22    35%    4     n/a     22/586
   box     100.0%  598    22.83    0.21    32%    4   +0.00     24/586
B_20px      93.1%  557    26.95    0.20    31%    3   +0.04     46/545
 F_0.4      98.5%  589    25.20    0.56    83%    2   +0.06     82/578
 F_0.6      99.8%  597    25.25    0.56    76%    2   +0.04     58/586
```

**The `F_q` arms actively damage cones**: they lift 76–83% of them onto "elevated" returns —
i.e. they clip the cone body — and **2–4× the track jumps**. On the class that is 83% of the
data, the box-fraction rule is a regression.

### The metric that prevented a false positive

At 50–75 m in run 1, `F_0.4` reported **+11.77 m with 100% of detections moving outwards**,
which under range-only metrics reads as a triumph. The height metric showed the deciding points
never left road height — it had hopped to a *farther road ring*. This is why the RViz tool
exists and why range alone must not be the acceptance criterion.

---

## 6. What we now believe

1. **There is no residual near-bias on vehicles.** Arm A places the one car at 97% elevated
   returns, and every alternative agrees with it to ±0.00 m. `reject_ground` (shipped
   2026-08-20) already fixed the problem this proposal was written to fix.
2. **The center-patch rule as specified should not ship.** It is neutral on vehicles and a
   regression on cones, and its stated justification does not hold.
3. **The real far-field defect was the 100 m crop wall**, which was truncating the only distant
   object in the bag. That is now a parameter at 150 m.
4. **One narrow win survives**: on the car, `F_0.6` and `B_20px` match A's placement while
   removing all 6 frame-to-frame jumps. On one track.
5. **The evidence base is too thin to ship anything on.** One car track, two bus detections.

---

## 7. Plan going forward

**Priority 1 — get a bag with actual vehicle traffic.** Everything above rests on one car
track. Until there is a bag with several vehicles at 40–150 m, no fusion rule change should
ship. This is the single highest-value next step and it needs no code.

**Priority 2 — decide the transform crop settings on their own merits.** These are already
parameters and are independent of the association rule:

- Keep `crop_max_range = 150.0`. It recovered the majority of the car's track. Re-check CPU and
  `/lidar_2d_projection` rate on the vehicle, not just on replay.
- **Raise `crop_lateral_limit` with it.** At 150 m the ±20 m crop keeps only 45% of the image
  width. Sweep 20 → 45 m and measure cost.
- **Revisit the `lim_z = +1` ceiling** (still hardcoded). It is 3.46 m above the road and clips
  box trucks and taller. Consider raising to ~+2.5 m and measuring the point-count cost.
- Decide whether `voxel_size` stays at 0.0. It is free-to-slightly-faster and costs nothing at
  range, but it does densify the near field — see the `foreground_points` caveat below.

**Priority 3 — re-test the one surviving win, per class.** If a bag with vehicles arrives,
re-run the harness and check whether `F_0.6`'s "same placement, zero jumps" result on the car
holds across several tracks. If it does, the shippable form is **class-conditional** — applied
to vehicle classes only, never to cones — not the global parameter the original document
proposed. That is a materially different design and needs its own decision.

**Priority 4 — small, independent cleanups.**

- Add a sign guard on `uv1[2,:]` in `transform.py` (latent behind-camera projection).
- Wire the `ground_rejection_*` parameters into `yolo.launch.py`; they are declared in
  `fusion_node.py` but absent from the launch file.
- Watch `foreground_points`: it cuts at `max(median + 3·MAD, 0.05)` of inter-point depth gaps.
  Denser near-field returns shrink that median, so the cut sits on its 0.05 m floor more often
  and may split clusters more eagerly than before. Unverified.

**Explicitly not planned:** shipping `center_patch_px` / `center_patch_min_points` into
`fusion_node.py` and `yolo.launch.py` (Step 4 of the original document). The evidence does not
support it.

---

## 8. Reproducing any of this

```bash
source /opt/ros/jazzy/setup.bash
source ~/.local/opt/adps_custom_msgs/setup.bash

# offline scorer — seconds, no GPU, no containers
python3 scripts/patch_ab.py /home/avalocal/probe_bag_150 \
    --lidar-bag /home/avalocal/rosbag2_2026_08_20-13_11_07

# side-by-side in RViz; needs a running rmw_zenohd
ros2 run rmw_zenoh_cpp rmw_zenohd &
python3 scripts/patch_ab_rviz.py /home/avalocal/probe_bag_150 \
    --source-bag /home/avalocal/rosbag2_2026_08_20-13_11_07 \
    --arms A F_0.6 B_20px --rate 0.5 --skip-empty
rviz2 -d config/patch_ab.rviz      # written on startup; fixed frame lidar_tc
```

`--classes car bus` narrows to the vehicle classes. ENTER pauses, `n` steps, `q` quits.

To record a fresh probe bag (~3 min):

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd &
USE_SIM_TIME=true docker compose -f docker-compose.yml -f docker-compose.replay.yml \
    --profile runtime up -d transform_node yolo_node
ros2 bag record -o probe_bag_X /yolo/tracking /lidar_2d_projection /fused_bbox /tf_static &
scripts/play_rosbag.sh /home/avalocal/rosbag2_2026_08_20-13_11_07
```

The `docker-compose.replay.yml` overlay is required for `transform_node` to pick up edits to
`src/transform.py` without an image rebuild — it bakes source in with `COPY . .`, unlike
`yolo_node` which bind-mounts and colcon-builds at container start.

---

## 9. Caveats on everything above

- **One car track, two bus detections.** The single largest limitation.
- `dmed` values of ±0.00 with wide IQRs mean *the median detection does not move* while a
  minority move a lot. Read the IQR, not just the median.
- Cross-run differences in cone numbers (run 1 vs run 2) include YOLO/tracker nondeterminism,
  not only the transform change. Only the isolation test in §3 controls for that properly.
- The harness reproduces the node's pairing but drains deferrals at message boundaries rather
  than on a 50 Hz timer, so its `expired` counter runs high (276–322 vs the node's ~2). Since
  projection-stamp mismatches are **0** across all 1257/1261 arrays, this changed no selection.
- Radar is still absent. The association rule affects *placement*, not far-field range.
