# Changelog

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