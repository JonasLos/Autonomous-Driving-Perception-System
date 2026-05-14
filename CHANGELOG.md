# Changelog

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