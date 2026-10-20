# Road boundaries: where the work stands (2026-10-08)

The note to start from when this work is picked up again. Detail lives in
[`road_boundary.md`](road_boundary.md) (the downstream node and its A/B) and
[`road_boundary_upstream_plan.md`](road_boundary_upstream_plan.md) (the SphereFormer/SAM3
changes, measurements, speed work). This page says where everything is, what is decided, and
what is open.
[`road_boundary_obstacles.md`](road_boundary_obstacles.md) lists every change that handles
obstacles near the edge, layer by layer, with its parameter and off value.

## Where everything is

| what | where | state |
|---|---|---|
| Vehicle checkout | `~/Downloads/Autonomous-Driving-Perception-System`, branch `radar_integration_and_fusion` | `road_boundary` committed (`695a874`), **off by default**. Otherwise untouched: `git status` shows only the 3 pre-existing untracked files. SAM3 submodule on `vehicle_deploy`. |
| Upstream fixes | git **worktree** `~/Downloads/ADPS-upstream`, branch `upstream_boundary_fixes` | 7 commits on top of `radar_integration_and_fusion`, latest `36b5e22` |
| SAM3 changes | submodule branch `upstream_boundary_fixes` | 5 commits on top of `vehicle_deploy`, latest `6909864`. The branch ref is also in the vehicle checkout's submodule repo, so the worktree's submodule commit resolves there. |
| Docker images | `perception-sphereformer:upstream`, `perception-object-fusion:roadb` | `:latest` images untouched. SAM3 and YOLO have no branch image: they colcon-build whichever checkout the stack is started from. |
| Recordings and results | `~/fusion_data/recordings/` (`planner_ab_*`, `route2_*`, `roadb_inputs_*`, `planner_ab_plots/`) | persistent |
| Session harness and logs | `~/fusion_data/upstream_boundary_session_2026-10-07/` (see its `README.md`) | copied out of `/tmp` before shutdown |
| SAM3 compile cache | `~/.cache/huggingface/torchinductor_sam3` (~200 MB) | persistent; only used with `cuda_graphs` |

Nothing is pushed to GitHub. The worktree symlinks `.env`, `scripts/lib`, `rosbags` and the
weights from the vehicle checkout; never `git add` them.

## What is done

- **Downstream `road_boundary` node** (your original options 1–4): occlusion rejection, extent
  extraction, robust fit with a width prior, temporal tracking. On the stationary Sep 22 road
  bags, through the planner's own `lmp_v10c`, frames where the edge is pulled in by more than
  0.5 m at 20 m drop from 27–31% to 0–2.4%.
- **Upstream fixes, tiers 1–4** on the branch: SphereFormer S1–S4, S6, S7; SAM3 A1–A5;
  recording category X2; the edge-offset diagnostic X3. With `road_boundary` and the
  `lmp_v10c` guards on top, pulled-in frames go to 0%.
- **SAM3 speed:** batched prompts and post-processing fixes (566 → ~210 ms per frame under load,
  default on). `cuda_graphs` (~170 ms) and CPU pinning (~159 ms together, ~6 Hz) are **opt-in**.
  TensorRT was dropped by decision.
- **Warm-up warnings:** `cuda_graphs` warms up at start-up before the node reports ready. Every
  warm-up is logged, as are a non-persistent cache and an ineffective setting, and the node
  falls back to eager on compile errors. SphereFormer warns when `accumulate_sweeps` has no
  odometry.

## Decisions waiting on you

1. **Tracker fix to the vehicle branch?** The `road_boundary` fix for crossed stations
   (re-initialise after 3 gated frames) exists only on `upstream_boundary_fixes`. Without it, a
   bad initialisation coasts for up to 3 s: a ~4 m swing of the planner's edge on Route2.
   **If next week's vehicle test runs `road_boundary` from the vehicle checkout, it runs without
   this fix.** Cherry-picking the `road_boundary.py` / `road_boundary_node.py` parts of `caddd21`
   is a small change.
2. **Shoulder:** SphereFormer counts the dirt shoulder as road, SAM3 ("gravel road") does not.
   On the branch the planner's road is ~0.9 m wider. Is that the road the planner should see?
3. **`lmp_v10c` guards** (`enable_coherent_source_mode`, `enable_pair_dropout_hysteresis`, off by
   default in the planner): needed once SAM3 runs faster, or the edge flickers between
   SAM3-fused and SphereFormer-only pairs. These are planner runtime parameters; the planner repo
   was not changed.
4. **Obstacle prompt:** `car or truck or person` costs SAM3 rate. `"car"` or `""` recovers it.

## Vehicle test, 2026-10-20: rates on the car

Branch perception on the vehicle, with only SAM3 and SphereFormer running (plus
`transform_node`; no YOLO, CLRerNet or radar), and the planner in the loop without obstacle
avoidance:

| topic | rate on the vehicle | offline replay, full perception stack |
|---|---|---|
| `/sam3_{left,right}_boundary` | **~7 Hz** | ~4.5 Hz default (~6 Hz with `cuda_graphs` + pinning) |
| `/sphereformer_{left,right}_boundary` | **~10 Hz** (the LiDAR rate) | 7.2–7.7 Hz |

Both are higher than offline, most likely because the other detectors are off. Plain `lmp_v10c`
was designed for SAM3 at ~1–2 Hz and showed little improvement at these rates. Next run: the
same with its guards on, via the new planner launch file
`~/latest_planner/.../AVA_Local_Planner/launch/lmp_unmarked_v10c_guards.launch.py` (bag
`lmp_unmarked_v10c_guards_*`). Its `source_mode_transition_frames` (5) counts frames; at ~7 Hz
that is ~0.7 s, against ~3 s at 1.5 Hz. Raise it (≈20–25) if source switches look abrupt.

## Testing still open

- **Moving vehicle in traffic.** No recording exists with SphereFormer, SAM3 and odometry
  together while driving. Record next week's drive with the `boundary_eval` category:
  `scripts/record_rosbag.sh boundary_eval`. **Run it from the worktree**; the category exists only
  on the branch's `topics.yaml`.
- **Opt-in speed options with the whole vehicle software running** (planner, controller,
  drivers): pinning and `cuda_graphs` were measured with perception only. Check the controller
  loop rate and driver rates.
- `road_boundary`'s SAM3 width (4.8–5.6 m) is below raw SAM3 (6.2 m), not yet explained.
  `default_width` 7.0 m is a guess.

## Running it again

From the worktree, with the vehicle stack down (container names are fixed):

```
cd ~/Downloads/ADPS-upstream
scripts/run_fusion.sh --down                       # if the vehicle fusion container is up
EXTRA_COMPOSE_FILES=docker-compose.upstream.yml scripts/run_radar.sh --replay BAG
#   + opt-in pinning:      EXTRA_COMPOSE_FILES="docker-compose.upstream.yml docker-compose.cpuset.yml"
#   + opt-in CUDA graphs:  SAM3_CUDA_GRAPHS=true   (wait for "cuda_graphs: ready" in the SAM3 log)
```

Before a vehicle test, read "Start-up and warm-up" in the plan doc. Defaults need no warm-up,
but `cuda_graphs` takes 16 s to 3 min. After switching back, bring the vehicle stack up from the
vehicle checkout. `docker compose up -d` does NOT restart an unchanged container, so restart
`sam3_ros` explicitly after changing checkouts.

Bags: Sep 22 `perception_input` (stationary, real road, with obstacles) for the planner-level
A/B; Route2 `/media/avalocal/1.0 TB Disk/Route2_raw_data/rosbag2_2026_05_05-14_52_50` (public
roads, ~19.5 m/s). For Route2, remap `/camera_fl/image_color:=/camera_fl/image`, leave radar
out, use `text_prompt="paved road or gravel road"`, and judge it at perception level:
`lmp_v10c` does not work on two-way roads at speed.

## Not started (deliberately)

- S5 + X1: per-point state (edge / occluded / unseen) and a typed boundary message, which needs
  a planner change on `perception-fusion-integration`.
- S8: retrain SphereFormer to label road under vehicles (labelling pipeline, training).
- A6: SAM3 keyframes plus video propagation to reach camera rate (restructures the node).
- Planner: `lmp_v10c` on two-way roads (road wider than its 8.5 m lane limit; overlap check
  deadlocks at speed). This belongs to the planner, outside this work.

## Gotchas that cost time

- `lidar_tc` is yawed −5.35° from the vehicle axis; anything binning in the LiDAR frame shears.
- LiDAR rings at range give arcs that end inside the road. Extremes count as edges only with
  non-road **ground** seen just beyond them.
- The ego roof and hood are labelled "car" by SphereFormer (cropped at 2.5 m on the branch).
- `cuda_graphs` graphs can only be recorded on the thread that recorded the first ones, so the
  warm-up runs on the inference thread.
- Waiting with `pgrep -f PATTERN` / `pkill -f` matches the waiting shell itself; wait on PIDs.
