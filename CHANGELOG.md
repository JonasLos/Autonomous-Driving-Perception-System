# Changelog

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