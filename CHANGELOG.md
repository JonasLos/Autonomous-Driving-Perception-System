# Changelog

## 2026-09-02

### Fixed
- **The lane selector was scoring against the wrong ego reference, and that was the lane jumping.** `LanePairSelector` picks the pair whose centreline is closest to the ego path, and took the ego path to be the line `y = 0` in `lidar_tc`. It is not: `lidar_tc` is yawed about **5.35 deg** from the vehicle axis, so the ego path is `y = -0.0937 * x` and `y = 0` diverges from the vehicle by 9.4 cm per metre of range. At the ~18 m the score window actually samples — the near field is `TIER_NEAR_EMPTY` on 85% of frames — that is 1.75 m, half a lane width, which put the ego lane and the left-adjacent lane at *equal* distance from the reference and made the choice between them a coin flip decided by centimetres of interpolation noise. The reference is now a ray, `lane_ego_y_offset + tan(lane_ego_yaw_deg) * x`, subtracted per grid node. Over the 536-frame `lane_probe_2026-08-25`: pair switches **2 → 0**, frames jumping >0.5 m **11.5% → 0.0%** (arm A at git HEAD), p90 `|d centre_y|` **0.194 → 0.053 m**, ego-bracketed **74.1% → 100%**. It holds the lane with hysteresis set to **zero** and the window widened to `[6,40]`, so the hysteresis added on 2026-08-28 was never the fix — it was damping this.
- Measured four ways, two of which never read `/tf_static`: the detected lanes' own direction over 159 straight-road frames is **-5.354 deg** (IQR -5.68..-5.11); the lane vanishing point sits 6.6 px from the principal point, so the camera axis is parallel to the road to 0.11 deg and `T1` puts that axis at **-5.061 deg**; the bumper radar's +x is at **-5.443 deg**. Odom rules out the alternative reading — sideslip is +1.45 deg +/- 0.1, so the vehicle is not crabbing. `/tf_static` publishes `lidar_tc -> base_link` as **identity**, the only transform on the vehicle that disagrees, and identity is what a transform looks like when nobody calibrated it.
- `lane_score_max_x` **15.0 → 30.0**. `[6,15]` was narrowed because a wider window scored worse; that was the shear, and a shorter lever arm meant a smaller error, not less curvature. With the reference corrected, `[6,30]` takes `TIER_NEAR_EMPTY` from 85% to **0%** and containment to **100%** — every decision is now a full one rather than the ranked-last fallback.
- `lane_hysteresis_margin` **1.75 → 0.75**. At 1.75 m the margin exceeds half a lane width, so a genuine lane change cannot beat it. Note `lane_probe_2026-08-25` contains **no** lane change — its one manoeuvre is the turn onto the road — so nothing here shows the selector *following* one; that needs a bag that contains one.

- `.dockerignore` now excludes recorded bags (`*.mcap`, `*.db3`, `adps_*/`, `rosbag2_*/`, `probe_bag*/`, `lane_probe_*/`). `Dockerfile.clrernet` ends in `COPY . .`, and a 2.3 GB `adps_2026-08-25_11-52-15/` left in the repo root would have been baked straight into the image. Nothing inside a container ever reads a bag — they are replayed from the host into the running stack — so this is exclusion, not relocation. Caught on the first rebuild after a bag was left in the tree.

### Notes
- **The same 5.35 deg error is in the object pipeline and is not fixed by this change.** `detect_3d_node.py` declares `target_frame: base_link` against that identity transform, so every fused object is placed at an azimuth 5.35 deg off: 0.9 m at 10 m, 4.7 m at 50 m, **9.4 m at 100 m** — larger than any effect in the centre-patch investigation, whose far-field car sits at ~105 m.
- The published lane points still leave the node in `lidar_tc` and are therefore still sheared (1.4 m off the vehicle axis at 15 m, 2.8 m at 30 m). Only the *selection* is corrected here: the node now picks the right lane and reports it in the LiDAR's frame, which is what it has always done. Correcting the published geometry means either fixing `lidar_tc -> base_link` in the vehicle description and having consumers do a real tf2 lookup, or rotating `x,y,z` in `transform.py` after `u,v` are computed from the unrotated cloud (`T1` is correct — the projection must not move). That is a decision for whoever owns the planning consumer, not a side effect of this fix.
- Several source comments recorded parameter sweeps run against the broken reference, and read as justification for the old values. They now carry the re-run numbers and say what the old ones were measuring.

## 2026-08-17

### Fixed
- **SphereFormer now starts.** Two independent breakages, neither related to the sync work. (1) Its 371MB checkpoint is excluded from the build context by `.dockerignore:22`, so it was never in the image; `setup.py` installs it correctly but only `if os.path.isfile(...)`, making the absence silent at build time and fatal at runtime. Now bind-mounted into the source tree so the container-start `colcon build` installs it to `share/` the normal way. (2) With the model loading, it then died at its first forward pass on `voxel_grid requires pyg-lib>=0.6.0`. That was a torch-geometric API change rather than a missing install: `torch_cluster 1.6.3` is present and imports fine, but torch-geometric 2.8 dropped the torch-cluster fallback (`typing.py:66` now reads `WITH_GRID_CLUSTER = hasattr(pyg_lib.ops, 'grid_cluster')`), and requirements pinned neither package. `pyg-lib` is now installed in **stage 2** of `Dockerfile.sphereformer` -- not in `requirements-sphereformer.txt`, which is an input to the builder stage and would force a needless recompile of the SparseTransformer CUDA extension for a package that compile does not use. Verified: builder stage stayed cached (zero nvcc invocations), `pyg_lib 0.8.0+pt211cu130`, `WITH_GRID_CLUSTER=True`, node runs inference at ~2.5Hz and publishes all three of its topics.
- **Pairing is two-sided again.** `StampMatchedBuffer` gained `wait_for_newer`: a reference with no buffered sample captured at or after it is now *deferred* for a bounded wait instead of being forced backwards onto an older sample. This is the fix the 2026-08-13 Operational Findings called for, done in the shared buffer rather than as a detection delay in `fusion_node`, so all four consumers get it. `fusion_node`'s `max_pairing_skew` drops **0.12 → 0.06** as a result: the one-sided worst case was a full LiDAR period, the two-sided one is half.
- `sam3_mask_transform`: deleted its private `BufferedProjection` class and `_match_projection`, and migrated to `perception_common.stamp_sync`. This was the 2026-08-11 original that `stamp_sync.py` was generalised *from*, left behind when the other two nodes migrated; the copies had already drifted (its tree was built with `ros2_numpy`, the shared one with `sensor_msgs_py`, and only the shared one carried the single-parse fix from 2026-08-13). It also hand-rolled the validate-then-apply parameter callback and the unmatched-pairing prose, both of which already existed in the shared module.
- Nothing in the SAM3 container imports `ros2_numpy` any more, so the `git clone` of it at container start is gone — one less network dependency when working offline.
- All three watchdogs (`fusion_timeout`, `lane_timeout`, `boundary_timeout`) ran on `time.monotonic()`, so they fired against wall time regardless of `use_sim_time` and would trip continuously against a paused bag. They now read the node clock. `_last_publish` also initialises to `None` rather than `0.0`: under sim time the node clock reads 0 until the first `/clock`, and once it jumps to bag epoch a `0.0` baseline makes the first tick see a ~1.7e9s gap.
- `StampMatchedBuffer.add` inserts in stamp order instead of appending blindly. Transport can deliver two samples a few milliseconds out of order, which left the deque unsorted and quietly broke the three things that read `_entries[-1]` as the newest: left-side eviction, `newest()`, and the new "is there a sample at or after this reference" test.
- `transform.py` imported `src.configs`/`src.utils`, byte-identical copies of the `perception_common` modules, and read its own co-located `src/topics.yaml` rather than the installed one. Both duplicates and the extra `topics.yaml` are deleted.
- `topics.yaml`: three copies existed and had already started to drift (the `sam3:` block sat in a different place in two of them). Only `src/perception_common/topics.yaml` is installed by `setup.py`, so the other two are gone. `categories.raw` referenced `topics.raw.lidar` and all four entries of `categories.outputs` referenced a `topics.outputs.*` block that does not exist — `yq` resolves those to the literal string `null`, which `record_rosbag.sh` was passing to `ros2 bag record` as a topic name.
- `record_rosbag.sh` and `check_freq.sh` had `TOPIC_FILE` hardcoded to `/home/dev/Documents/...` and `SAVE_DIR` to `/media/dev/T9/...`, neither of which exists on this machine. Both are now derived from the script's own location and overridable (`TOPIC_FILE=`, `ROSBAG_DIR=`).

### Added
- SphereFormer's 371MB checkpoint is now bind-mounted into the container. `.dockerignore:22` excludes it from the build context, so it was never in the image and the node died at startup with `FileNotFoundError`. `setup.py` already installs it to `share/` correctly, but only `if os.path.isfile(...)` — so its absence was silent at build time and only surfaced at runtime. The mount targets the *source* tree, not the install tree, because the container runs `colcon build` at startup and setup.py then copies it into share exactly as a normal build would; mounting into the install tree would put a read-only file where colcon wants to write. Override with `SPHEREFORMER_CHECKPOINT=` to keep weights outside the repo.
- `scripts/play_rosbag.sh` — replays a bag into the running Docker stack. Sources the custom-message overlay, matches the containers' `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION`, publishes `/clock`, and defaults to the new `replay_input` category (`/camera_fl/image`, `/camera_fl/camera_info`, `/lidar_tc/velodyne_points`, `/tf_static`). It refuses to replay any topic the containers publish themselves unless given `--allow-internal`, and it warns when no `rmw_zenohd` is running — compose defines no router, so without one on the host the player and the containers never discover each other and the only symptom is silence.
- `USE_SIM_TIME` in a `x-common-env` YAML anchor shared by all five runtime services, reaching each node by a different route (compose `command:` for yolo/sam3/sphereformer, Dockerfile `CMD` reading the env var for clrernet/transform, which avoids duplicating those CMDs across two files). No node source change was needed to *accept* it: `rclpy`'s `TimeSource` declares `use_sim_time` itself, and `create_timer` already uses the node clock.
- `config/rosbag_qos_overrides.yaml` — the player reproduces recorded QoS, and `yolo.launch.py` subscribes RELIABLE. A BEST_EFFORT-recorded image topic therefore will not match, and YOLO goes silent with no error. `/tf_static` is kept `transient_local` so a late-starting container still receives it.
- `docker-compose.replay.yml` — optional overlay mounting source over the install paths of the three services that bake code in with `COPY . .`, for iterating on pairing code without a rebuild. Not needed to replay a bag.
- `transform.py` now matches `/camera_fl/camera_info` to each sweep by capture time (`camera_info_max_skew`, default 0.2) instead of applying whatever arrived last. `camera_info_wait_for_newer` defaults to **0.0** deliberately: deferring a sweep would add latency to `/lidar_2d_projection`, which every downstream node inherits, and would widen the very camera-vs-LiDAR phase gap the fusion nodes defer to close. A sweep is never dropped over camera_info — an unmatched one falls back to the last known geometry and warns.
- `wait_for_newer`, `deferral_pump_period` (read-only) and `stats_log_period` on all three transform/fusion nodes, plus a periodic INFO of `status()`.
- `status()` now reports `mean|skew|`/`max|skew|` over a rolling window plus `deferred`/`expired`/`stale`/`nonmono`/`resets`. Without a mean-|skew| readout there is no way to tell from outside a node whether a pairing change helped.
- `src/perception_common/test/test_stamp_sync.py` — 25 pure-pytest tests, no ROS context needed. The ordering guarantees are asserted here because the input sequences that break them are exactly the ones a bag will not reproduce on demand.

### Changed
- `wait_for_newer` defaults to `0.0` on the *class* and `0.06` on the *node parameters*, so the shared module stays a drop-in and every node has a live rollback: `ros2 param set <node> wait_for_newer 0.0` restores one-sided matching without a restart.
- Deferral is pumped by the caller, not by the module: `stamp_sync.py` stays `rclpy`-free (it is imported from four separate Python environments where `rclpy` resolves only through a carefully ordered `PYTHONPATH`) and takes `now` as an opaque float. Each node calls `drain()` after every `add()` — which is what resolves nearly all deferrals, in the same tick the awaited sample lands — and from a 0.02s timer that only matters when the fast stream stalls.
- Emission is now monotonic per stream, via three stacked guards: per-key FIFO with head-of-line blocking, a reference watermark, and a matched-sample watermark. The last is not redundant: for references `t1 < t2` matching samples `s1, s2`, `s2 < s1` is reachable whenever `t2 - t1 < 2 * max_skew`, which at 10Hz with an 0.06 bound is the normal operating point. The sample watermark degrades to the nearest non-decreasing sample rather than dropping the frame, so it costs monotonicity nothing.
- `sam3_mask_transform` passes `key="left"`/`"right"` rather than holding two buffers, so the sides stay independent while both still match against one shared cloud — which is what keeps the parse and the KDTree of a frame shared between its two contours.
- `scripts/lib/topics.sh` — the category-resolution loop, previously copied into three scripts, now lives once and errors on a dangling reference instead of emitting `null`.

### Operational Findings
- **Simulated against the 2026-08-13 measured timing** (10Hz LiDAR, 10.017Hz camera on a separate oscillator, 0.031s and 0.014s post-stamp arrival, 240s ≈ 4 beat periods). Reproduced as `test/test_pairing_regression.py`, so a regression fails a test rather than the vehicle:

  | config | matched | unmatched | mean\|skew\| | max\|skew\| | monotonic |
  |---|---|---|---|---|---|
  | one-sided, `max_skew=0.12` (previous production) | 2403 | 2 | 0.0675s | 0.1170s | yes |
  | one-sided, `max_skew=0.06` | 1014 | **1391** | 0.0385s | 0.0600s | yes |
  | bounded-wait, `wait=0.06 max_skew=0.06` (new) | 2404 | 1 | **0.0246s** | 0.0500s | yes |

  The middle row reproduces the 2026-08-13 starvation — 58% of frames lost at a tight one-sided bound — which is what makes the simulation credible rather than merely self-consistent. The new configuration matches at the same rate as the wide bound while cutting mean pairing error **2.7×**, from ~0.34m of displacement at 5m/s to ~0.12m. Note the middle row's mean|skew| is *low* precisely because it only matched during the favourable half of the beat; mean skew is only meaningful read alongside the match count.
- These are simulated arrival times, not a bag run. Confirm on real data with the A/B in `DOCKER.md` → Offline bag replay before trusting the 0.06 default on the vehicle, and sample a full beat period — a 60s window can sit entirely inside a favourable phase.
- **Live run against `rosbag2_2026_05_05` (12min, gravel road, 2026-08-17).** `fusion_node`: `matched=1826 unmatched=0 deferred=731 expired=1 nonmono=0`, `mean|skew|=0.0248s`, `max|skew|=0.0501s`, `/fused_bbox` at 10.07Hz against `/yolo/tracking` at 9.89Hz — full rate, no starvation at the 0.06 bound. The measured mean matches the simulation's 0.0246s prediction to within 0.2ms. 40% of detections were deferred, so the mechanism is doing real work rather than sitting inert. `sam3_mask_transform`: `matched=548 unmatched=0 deferred=0` — SAM3 inference is slow enough that the matching cloud has always already arrived, so its pairing was two-sided anyway and the deferral costs it nothing. Same asymmetry the 2026-08-13 entry noted for CLRerNet.
- **Live run against `rosbag2_2026_05_05-14_25_02` (Route1, 21min, lane-marked road, 2026-08-17)** — the run that exercises all four consumers, after rebuilding `transform_node`/`clrernet_node` so they carry the new code and `use_sim_time`:

  | node | matched | unmatched | deferred | expired | nonmono | mean\|skew\| | max\|skew\| |
  |---|---|---|---|---|---|---|---|
  | `fusion_node` (0.06) | 3157 | 26 | 1823 | 60 | 0 | 0.0249s | 0.0501s |
  | `clrernet_transform` (0.08) | 3074 | 0 | 90 | 0 | 0 | 0.0250s | 0.0772s |
  | `sam3_mask_transform` (0.08) | 530 | 0 | 0 | 0 | 0 | 0.0238s | 0.0772s |

  `/fused_bbox` 10.0Hz, `/clrernet/left_lane` 10.0Hz, `/clrernet/current_centerline` 9.8Hz, all against a 9.9Hz camera. `fusion_node`'s `unmatched` is a startup transient — it froze at 23 within the first seconds and moved only +3 across a 15-minute run while `matched` climbed past 3000. `transform_node` logged no unmatched-camera_info warnings. 104 consecutive `/fused_bbox` header stamps sampled: zero backwards steps.
- **Sim time verified by pausing the player mid-run.** With the bag paused, `/clock`, `/fused_bbox`, `/clrernet/left_lane` and `/sam3_left_boundary` all went to exactly zero messages over 10s, and resumed at full rate with no burst of watchdog empties and `nonmono=0`. Under the previous `time.monotonic()` watchdogs all three would have emitted an empty result every 0.5s for the duration of the pause, which is the failure this work removes. Note `/clock` takes a few seconds to reach the requested rate at startup (measured 36Hz early, 93Hz steady at `--clock 100`).
- `clrernet_transform` and `sam3_mask_transform` both peak at `max|skew|=0.0772s` against their 0.08 bound while `fusion_node` at 0.06 peaks at 0.0501s. Their means are identical (~0.024s), so the wider bound is only admitting a rare tail; tightening them to 0.06 would trade a few matches for a slightly tighter worst case. Left at 0.08 pending a reason to care.
- Bags recorded before 2026-08-13 carry `/camera_fl/image_color`, not the raw `/camera_fl/image` every node now subscribes to, so replaying them needs `-- --remap /camera_fl/image_color:=/camera_fl/image`. Without it the whole camera side of the stack sits silent with no error. `scripts/play_rosbag.sh --help` documents the form.
- `ros2 topic hz` on `/clrernet/all_lanes` reports nothing unless `clrernet_msgs` is on the host overlay, even though the topic is flowing (publisher and subscriber counts both 1). Run `scripts/install_host_custom_msgs.sh` before concluding a custom-message topic is dead.
- **The replay bottleneck was CPU thread oversubscription, not GPU.** With all five services up, the bag player fell to 6.31Hz and `/fused_bbox` to 5.7Hz -- but the GPU was only 31% busy and the bag disk 30% utilised, while load average sat at 47 on 32 cores. Cause: PyTorch sizes its CPU thread pool from the host core count, not the container's share, so each service defaulted to 24 threads and five of them fought over 32 cores. The player needs well under one core and simply could not get scheduled. Capping `OMP_NUM_THREADS`/`MKL_NUM_THREADS` at 4 in the shared env anchor:

  | | before | after |
  |---|---|---|
  | load average | 47.4 | 20.6 |
  | GPU utilisation | 31% | **83%** |
  | `/camera_fl/image` | 6.31Hz | **10.07Hz** |
  | `/fused_bbox` | 5.66Hz | **10.10Hz** |
  | `/clrernet/left_lane` | 5.37Hz | **10.05Hz** |
  | `/sphereformer_left_boundary` | 0.97Hz | **4.91Hz** |

  The GPU was being starved by CPU contention rather than being the limit. All five services now replay at full rate, and the GPU is finally the busy resource. Tune with `OMP_NUM_THREADS=`.
- `transform.py` never touches the GPU -- no `.cuda()`, no `device=` anywhere -- so its projection matmuls ran CPU-side across 24 threads (measured 538% CPU). The `device = torch.device("cuda:0" ...)` line removed above was dead precisely because nothing was ever moved to it. Porting that math to GPU is an obvious win but a behavioural change, so it is left alone here.
- **`boundary_timeout` was shorter than the SAM3 frame period.** SAM3 runs ~1.5Hz (0.5-0.8s inference), so at the old 0.5s default the watchdog fired *between consecutive real contours*: 28% of everything on `/sam3_*_boundary` was a spurious empty and the road boundary blinked out on over a quarter of frames. Raised to 2.0s, remeasured at 0% empties. The rule is that a watchdog timeout has to exceed its producer's period, or it stops meaning "dead" and starts meaning "slow".
- SAM3's throughput is set by the number of OR terms in `text_prompt`, which `_run_prompt_query` expands into one forward pass each. Measured: 1 term 0.25s, 2 terms 0.28s, 5 terms 0.63s — roughly 0.22s of shared image encoding plus ~0.08s per term. The five-term default costs 1.5Hz; `"gravel road or paved road"` gives 3.5Hz with better detection than the single comma-joined term, which barely fired on gravel (0.39Hz of contours).
- The deferral costs latency, by construction: `/fused_bbox` should appear up to `wait_for_newer` later than before. Skew falling *without* latency rising would mean the deferral is not actually engaging.

### Known Gaps

- The `topics.yaml` `yolo:` group still names `/yolov9/published_image` and `/yolov9/bboxInfo`, which are the legacy `yolov9_ros` topics. The active `Custom_YOLO_ROS` stack publishes `/yolo/detections`, `/yolo/tracking` and `/yolo/dbg_image`. So `perception_output` recordings miss the live detector topics, and `play_rosbag.sh`'s internal-topic guardrail does not know about them. Not changed here because it alters what `record_rosbag.sh` captures.

## 2026-08-13

### Fixed
- All image consumers now subscribe to the raw `/camera_fl/image` instead of `/camera_fl/image_color`, extending the 2026-08-11 SAM3 fix to `yolo_node`/`tracking_node`/`debug_node` (via the `input_image_topic` launch default) and to `clrernet_lane_detection` (via `topics.raw.camera`). Every one of them already requested `desired_encoding="bgr8"`, so `cv_bridge` debayers `bayer_rggb8` in-process and nothing else changes. `image_proc`'s `debayer_node` is lazily subscribed, so removing its last consumer makes it unsubscribe and go idle on its own — the ~77% of a core it was burning is freed without touching `ros_drivers`.
- `fusion_node`: each `DetectionArray` is now fused against the buffered `/lidar_2d_projection` captured nearest to its own header stamp, instead of against whichever projection arrived next. `yolo_node` and `tracking_node` both copy the source image header through untouched, so detections already carried the camera capture stamp in the same host clock domain as the LiDAR — the node was discarding information it had. Every `/fused_bbox` 3D position carried the full camera-plus-inference latency as displacement, and `/fused_bbox` feeds planning.
- `fusion_node`: the node is now detection-driven rather than projection-driven. At 10Hz clouds with an 80ms pairing bound, two consecutive clouds can match one detection array, so a projection-driven node publishes the same objects twice at two different geometries from a single detector frame.
- `clrernet_lane_transform`: same stamp-matched pairing, replacing the `max_detection_age` freshness gate. At 0.25s against a detector sitting behind `image_color` (≥0.5s), that gate was rejecting nearly every frame, so `/clrernet/left_lane`, `/clrernet/right_lane` and `/clrernet/current_centerline` were largely silent.
- `clrernet_lane_transform`: the projection was being parsed twice per frame — once via `pointcloud2_to_xyz_array(remove_nans=True)` for the geometry and once via `read_points(skip_nans=True)` for the pixel coordinates. Those two filters drop different rows (the latter also drops NaN `u`/`v`), while the KDTree built over `u`/`v` was used to index the former. Any disagreement silently returned the wrong 3D points with no error. A single parse removes the class of bug.
- `topics.yaml`: five camera entries pointed at topics that do not exist. The AVT driver publishes `~/image` with node name `camera_fl`/`camera_fr` and no namespace, so `/camera_fl/image_raw` and `/camera_fr/image_raw` were never valid; the rear cameras are `/camera_rear_1..3/`, and the `camera_rc`/`camera_tl`/`camera_tr` names survive only as TF frames and calibration filenames, with nothing in the vehicle launch mapping them onto the three namespaces. The keys are renamed to `camera_rear_1..3` rather than guessing that mapping.
- `yolo_ros/package.xml`: added the missing `perception_common` dependency. `fusion_node` has imported it since April; the build only worked because both land in the same install base.

### Added
- `perception_common/stamp_sync.py`: `StampMatchedBuffer` (buffers a fast stream, pairs a slow one against it by capture time) and `ProjectedCloud` (a projection parsed and KDTree-built once, memoized, shared across every consumer of that cloud). Generalised from the 2026-08-11 SAM3 implementation. The module is deliberately `rclpy`-free — it reads nothing but `msg.header.stamp` and reports unmatched pairings as prose for the caller to log.
- `fusion_node` and `clrernet_lane_transform`: `max_pairing_skew` (0.08), `projection_buffer_duration` (2.0), `fusion_timeout`/`lane_timeout` (0.5) and `projection_stamp_offset` (0.0), all settable at runtime. 0.08 is half a 10Hz LiDAR sweep plus headroom; below ~50ms buys nothing physical because the intra-sweep spread is 100ms wide regardless.
- `fusion_node` and `clrernet_lane_transform`: watchdogs that publish an empty result once output stops being refreshed. Previously a dead detector left its last 3D boxes and lane clouds standing in RViz and in planning, which reads as current geometry; in `fusion_node`'s case the cached detections were re-fused against fresh clouds indefinitely, so the boxes kept moving around the scene.
- `projection_stamp_offset` exists because the LiDAR stamp is end-of-sweep (`timestamp_first_packet: false`), so a point's true capture time is uniform over the preceding 100ms. `-0.05` centres pairing on the sweep. Left at 0.0 until measured.

### Changed
- `clrernet_lane_transform`: one KDTree per cloud instead of three per frame (it was rebuilt in `get_closest_lane_pair_3d` and again in each of the two `publish_3d_lane` calls), and the per-side lookup is vectorized rather than one `tree.query` per lane point.
- Removed `LatestStampedCache` from `perception_common/utils.py` now that its last caller is migrated. It gated a cached detection on freshness, which can only trade dropped frames for misplaced ones; leaving it importable from a shared package invited the next node to adopt it.

### Operational Findings
- **`fusion_node` pairing is one-sided, so its bound is a full LiDAR period, not half.** Measured live: `/yolo/tracking` arrives 0.014s after its capture stamp, `/lidar_2d_projection` 0.031s after its own. YOLO therefore outruns the projection pipeline by ~17ms, and a detection is processed *before* the cloud it should pair with has been buffered — matching can only reach backwards into clouds already held. The camera and LiDAR free-run on separate oscillators, so their phase sweeps the whole 0–100ms range with a ~60–100s beat. At `max_pairing_skew=0.08` the node matched for long stretches and starved completely in others (`/fused_bbox` dropping to 1.0Hz, i.e. watchdog-only, against a 10Hz detector). Raised to 0.12, which restored 9.93Hz with zero unmatched warnings. The honest cost is that a pair may be up to 0.12s apart, ~0.6m at 5m/s.
- `clrernet_lane_transform` matches 100% at 0.08 because CLRerNet inference is slow enough that the matching projection has already arrived — its pairing is genuinely two-sided. The asymmetry between the two nodes is the confirmation of the mechanism, not an inconsistency.
- The proper fix for `fusion_node` is to buffer detections by ~50ms so pairing becomes two-sided again, halving the worst-case error to half a LiDAR period. Not done here: it adds a deliberate latency to the obstacle path and should be measured against planning's tolerance first.
- Confirmed the debayer payoff: with every consumer moved to `/camera_fl/image`, `/camera_fl/image_color` reports **subscription count 0** and `debayer_node` sits at **1.0% CPU**, down from the ~77% recorded on 2026-08-11. The lazy `sub_raw_.shutdown()` fires as expected and no `ros_drivers` change was needed.
- Confirmed `/camera_fl/image` publishes `bayer_rggb8` at 2064x1544, so the `cv_bridge` debayer path is correct for every consumer. Rear cameras confirmed present as `/camera_rear_1..3/`; `/camera_fr/*` is absent because `enable_camera_fr` defaults to false.
- Verified against `/home/avalocal/ros_drivers/src`: the camera driver stamps on **host receive time** (`use_measurement_time: false`, `mono_camera_node.cpp:91`), so the stamp already contains exposure, GigE transport and driver queueing. The Velodyne stamps on host receive time too (`gps_time: false`), at end of sweep. Both are the same host clock, so there is no epoch mismatch and the "different clock domains" comment that justified skipping timestamp sync in `fusion_node` was obsolete.
- The residual camera-to-LiDAR bias is therefore **systematic and unmeasured**, and `max_pairing_skew` absorbs it silently rather than correcting it — pairing is now consistent, not accurate. **Closing that gap requires the devices to be hardware-synchronised first**; it is not a tolerance that can be tuned away, and `projection_stamp_offset` only corrects the known geometric part (end-of-sweep → sweep centroid). Prerequisites, all in `ros_drivers`: `use_measurement_time: true` needs camera-side PTP, which is `Off` in `Mako_G-319.yaml:85` — and that file is not even loaded for camera_fl, so PTP state is whatever is persisted in the camera's flash; `gps_time: true` needs PPS/GPS into the Velodyne. `readout_time_sec: 0.0218` in `camera_fl.intrinsics.yaml` is the only recorded hint of the camera-side offset.
- The debayer's cost is self-inflicted and independent of this work: it runs VNG, the slowest of its four algorithms (`debayer.cpp:59`, default `3`), on 2064×1544, and publishes RELIABLE depth-10. `qos_overriding_options` is available but no launch file sets it. If `/camera_fl/image_color` is ever needed again, set `debayer:=0` and a best-effort depth-1 override at `system_bringup.launch.py:142-149`.
- The camera's pixel format is not pinned by any config file: `avt_mako_15hz.yaml` overrides the package default and does not set `feature/PixelFormat`, so the camera uses whatever is in its flash. Confirm with `ros2 topic echo /camera_fl/image --field encoding --once` before trusting the raw path.

## 2026-08-11

### Fixed
- `sam3_mask_transform`: contours are now paired with the buffered `/lidar_2d_projection` captured nearest to their own header stamp, instead of with whichever projection was current when the contour arrived. The old pairing lifted stale pixels onto fresh geometry, displacing the boundary by however far the vehicle had travelled during the camera and inference pipeline — measured at 1.1s mean skew, ~5m at 5m/s. Skew is now bounded by the LiDAR period.
- `segmentation`: subscribes to the raw `/camera_fl/image` (`bayer_rggb8`, debayered by `cv_bridge`) instead of `/camera_fl/image_color`. The `image_proc` `debayer_node` runs a permanent ~5-frame backlog, putting `image_color` 0.455–0.651s behind its own capture stamp while the raw topic is 0.011–0.023s behind; that half-second was the dominant term in the fusion skew. The raw topic is also a third of the bandwidth.
- `segmentation`: the inference queue now discards the frame it supersedes rather than the incoming one. Dropping the newcomer left inference to start on the oldest frame of the busy window, adding up to one inference period (~0.2s) of avoidable staleness.
- `sam3_mask_transform`: left and right contours are handled independently. SAM3 skips publishing a side when it finds no points there, and that previously blocked the opposite side's boundary as well.
- `segmentation`: the inference queue is created before the image subscriptions, so a callback can no longer reach a queue that does not exist yet.

### Added
- `sam3_mask_transform`: `boundary_timeout` (default 0.5s) publishes an empty boundary cloud once a side stops being refreshed, so a stalled segmenter reads as "no boundary" downstream instead of leaving its last output standing in RViz.
- `sam3_mask_transform`: `max_pairing_skew` (default 0.08s) and `projection_buffer_duration` (default 2.0s) parameters, both settable at runtime. Unmatched-contour warnings state which of the two bounds was missed and in which direction.
- `segmentation`: `image_topic` and `compressed_image_topic` parameters; the input topic was previously hardcoded.

### Changed
- `sam3_mask_transform`: replaced the `max_detection_age` freshness gate (and its `LatestStampedCache` usage) with stamp-matched pairing. Widening that gate could only trade dropped frames for misplaced ones — at 1.0s it passed ~50% of projections, every one of them carrying second-old geometry.
- `sam3_mask_transform`: boundary point indices are de-duplicated before publishing, and the KDTree query is issued once per contour instead of once per contour point. Neighbouring contour pixels resolve to the same LiDAR returns, so the published cloud is now the size of the boundary rather than the size of the contour.
- `docker-compose.yml`: `sam3_mask_transform_node` starts with defaults; the `max_detection_age:=0.5` override is gone.

### Operational Findings
- Measured stamp-to-arrival delay across the SAM3 path: `/camera_fl/image` 0.011–0.023s → `/camera_fl/image_color` 0.455–0.651s → `/sam3_left_contour` 0.536–1.228s, against `/lidar_2d_projection` at 0.019–0.141s.
- `image_proc`/`debayer_node` for `camera_fl` sustains 10Hz output at ~77% CPU but never drains its queue, so its latency is a standing backlog rather than throughput loss. Other consumers of `/camera_fl/image_color` (including CLRerNet) still inherit it; fixing the node's QoS to best-effort/depth-1 would clear it stack-wide.

## 2026-05-14

### Fixed
- `fusion_node`: added `_foreground_points()` gap-based depth clustering to prevent overlapping objects in image space from biasing the fused 3D median between them. Points inside each bounding box are sorted by depth, split at the first MAD-scaled gap (≥ 5 cm), and only the nearest cluster feeds the median calculation.
- `objects_transform`: switched LiDAR point matching from first-found to closest-by-depth (`argmin` on z) for more accurate per-detection depth assignment; added debug logging for bbox matching, radar pairing, and published boxes.
- `tracking_node`: removed `frame_rate=1` kwarg from `BYTETracker`/`BOTSORT` constructor — argument was dropped in ultralytics 8.4.50, causing `on_configure` to fail silently and leaving the node stuck in unconfigured state.

### Changed
- `Dockerfile.yolo`: added `yolov9_ros` and `yolov9_msgs` to `COPY`, `rosdep install`, and `colcon build` so the YOLO container includes the full perception stack.

## 2026-04-29

### Added
- SAM3 topic namespace in shared topic configs:
  - `/sam3_left_contour`
  - `/sam3_right_contour`
  - `/sam3_left_boundary`
  - `/sam3_right_boundary`
- New SAM3 transform node implementation in the SAM3 submodule (`sam3_mask_transform.py`) with LiDAR projection to 3D boundary lifting.
- Runtime publication of `vision_msgs/BoundingBox3DArray` (`~/fused_bbox_3d`) from YOLO fusion output inside the YOLO submodule.
- Dedicated SAM3 executable script `scripts/sam3_mask_transform_node` in the SAM3 submodule.

### Changed
- Radar input for object fusion now uses `delphi_esr_tracks` instead of the legacy radar topic in [src/yolov9_ros/objects_transform.py](src/yolov9_ros/objects_transform.py).
- Docker runtime startup for `sam3_ros` now builds and launches both:
  - `segmentation_node`
  - `sam3_mask_transform_node`
- Docker runtime startup for `yolo_node` now performs in-container workspace build for:
  - `perception_common`
  - `yolo_msgs`
  - `yolo_ros`
  - `yolo_bringup`
- Docker build context filtering expanded in `.dockerignore` to avoid bundling nested `.git`, local virtualenvs, caches, and large model artifacts.

### Fixed
- SAM3 segmentation output headers now sanitize zero timestamps by applying current ROS time as fallback (prevents `0/0` stamped contour messages).
- SAM3 image now installs missing ROS/Jazzy dependencies required at runtime (`message_filters`, `sensor_msgs_py`, `tf_transformations`, and related build deps).
- SAM3 image now vendors `ros2_numpy` during build so projection/transform imports resolve consistently.
- YOLO submodule now declares `vision_msgs` dependency in `package.xml` to support RViz-friendly 3D bounding box publishing.

### Operational Findings
- Time synchronizer starvation in SAM3 was traced to timestamp domain mismatch:
  - `/lidar_2d_projection` was stamped in bag-time epoch.
  - SAM3 contour outputs were stamped in node wall-clock fallback epoch.
- Result: `ApproximateTimeSynchronizer` produced zero matches even with non-zero contour timestamps.

### Documentation
- Updated [DOCKER.md](DOCKER.md) SAM3 workflow and troubleshooting notes to reflect current Jazzy package names, runtime behavior, and timestamp/clock-domain debugging guidance.