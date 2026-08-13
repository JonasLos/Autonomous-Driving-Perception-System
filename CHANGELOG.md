# Changelog

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