# Docker Setup and Usage

This project uses Docker Engine and Docker Compose for building and running all perception system components. Podman is no longer supported or recommended.

---

## Official Docker Engine Setup (Ubuntu 24.04+)

If you want to use the official Docker Engine and Compose (recommended):

### 1. Remove conflicting packages (if present)
```bash
sudo apt remove docker.io docker-compose docker-compose-v2 docker-doc podman-docker containerd runc
```

### 2. Add Docker’s official GPG key
```bash
sudo apt update
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```

### 3. Add the Docker repository
```bash
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF
```

### 4. Update and install Docker Engine and Compose
```bash
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
```

### 5. (Optional) Add your user to the docker group
```bash
sudo usermod -aG docker $USER
# Log out and back in for group changes to take effect
```

### 6. Test Docker
```bash
docker run hello-world
docker compose version
```

---

If you see errors about missing kernel headers or broken packages, resolve those first (see troubleshooting above).

---

**Note:**
- The Docker repository must match your Ubuntu codename (e.g., noble for 24.04).
- If you previously had issues, ensure the GPG key and sources are set up exactly as above.

---

## Building and Running the Perception System

1. Build the base image:
   ```bash
   docker compose --profile builder build base_builder
   ```
2. Build all images:
   ```bash
   docker compose --profile runtime build yolo_node
   ```
3. Run the system:
   ```bash
   docker compose --profile runtime up yolo_node
   ```

---

## Build Status

- The base_builder image was successfully built using Docker Compose:
  ```bash
  docker compose --profile builder build base_builder
  ```
  - Build completed without errors (approx. 40 seconds on a typical connection).

If you encounter errors, see the troubleshooting section above.

Continue with the next steps to build all images and run the system as described.

---

## YOLO Node Build and Run Status

- The yolo_node image was successfully built:
  ```bash
  docker compose --profile runtime build yolo_node
  ```
- The yolo_node container started and all processes (yolo_node, tracking_node, debug_node) launched successfully:
  ```bash
  docker compose --profile runtime up yolo_node
  ```
  - All nodes reported "Configured" and "Activated".
  - The container exited with code 137 after stopping (this is normal if you stopped it with Ctrl+C).

If you see all nodes activating and no errors, the YOLO node is running as expected.

---

## Zenoh RMW Support

- The Docker images now include rmw_zenoh_cpp for ROS 2 Jazzy.
- The YOLO container sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` by default, enabling communication with Zenoh-based ROS 2 nodes.
- If your ROS 2 driver or other nodes use Zenoh, no extra configuration is needed in the container.

Rebuild the images after any changes to the Dockerfiles to ensure Zenoh support is included.

---

## Switching Middleware (RMW)

- By default, the container uses Zenoh for ROS 2 communication (`RMW_IMPLEMENTATION=rmw_zenoh_cpp`).
- If you do not want to use Zenoh and prefer the default DDS (Fast DDS), set the following environment variable before launching your nodes:
  ```bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ```
- You can set this in your Dockerfile, docker-compose.yml, or interactively in the container.

---

## SAM3 (Zenoh) Container Workflow

The `sam3_ros` service has been updated for reproducible startup with Zenoh and a standalone package build.

### What is now configured
- `docker/Dockerfile.sam` installs:
  - `python3-ament-package`
   - `ros-jazzy-rmw-zenoh-cpp`
   - `ros-jazzy-message-filters`
   - `ros-jazzy-sensor-msgs-py`
   - `ros-jazzy-tf-transformations`
   - `ros-jazzy-ament-cmake-python`
   - normalizes `src/SAM3_ROS_NODE` during image build by:
      - creating `scripts/segmentation_node` if missing
         - creating `scripts/sam3_mask_transform_node`
      - removing stale `pub_test_image.py` install reference from `CMakeLists.txt`
      - applying default runtime parameters used in this workspace (`use_compressed_image=False`, `text_prompt="road"`, and the raw `/camera_fl/image` topic)
         - vendoring `ros2_numpy` into the SAM3 virtual environment
- `docker-compose.yml` for `sam3_ros` now:
  - mounts the repository root to `/root/ws`
  - mounts `${HOME}/.cache/huggingface` to `/root/.cache/huggingface`
  - sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
   - builds required packages at runtime:
      - `perception_common`
      - `sam2_msgs`
      - `sam3_ros`
   - launches both nodes:
      - `ros2 run sam3_ros segmentation_node`
      - `ros2 run sam3_ros sam3_mask_transform_node`

### Build and run SAM3
From the repository root:

```bash
docker compose build sam3_ros
docker compose up sam3_ros
```

Or rebuild and run in one step:

```bash
docker compose up --build sam3_ros
```

### Validate Zenoh RMW in the image

```bash
docker run --rm test-sam3:latest bash -lc 'ls /opt/ros/jazzy/lib/librmw_zenoh_cpp.so'
```

If the file exists, Zenoh RMW is installed correctly.

### SAM3 Synchronization Notes (2026-08-11)

`sam3_mask_transform` pairs each contour with the buffered `/lidar_2d_projection`
captured nearest to the contour's own header stamp, rather than with whichever
projection is current when the contour arrives. Pipeline latency therefore shifts
*when* a boundary is published, not *where* its points land, and it no longer has to
be absorbed by a freshness bound.

Diagnosing a boundary that is empty, frozen, or misplaced:

1. Measure where the latency actually is. Every stage's stamp-to-arrival delay:

   ```bash
   docker exec test_sam3_container bash -lc 'source /opt/ros/jazzy/setup.bash && for t in /camera_fl/image /camera_fl/image_color /sam3_left_contour /lidar_2d_projection; do echo "== $t"; timeout 10 ros2 topic delay $t 2>&1 | tail -1; done'
   ```

   Reference figures from the white-jeep stack: raw `/camera_fl/image` 0.01–0.02s,
   `/lidar_2d_projection` ~0.05s, `/sam3_left_contour` ~0.3s. If `/camera_fl/image`
   itself is late, the problem is upstream of this container.

2. Read the node's own warning. It distinguishes the two failure directions:
   *"nearest projection is Xs newer than the contour"* means the matching cloud was
   evicted before the contour arrived — raise `projection_buffer_duration`.
   *"nearest projection is Xs older"* means no LiDAR scan as recent as that image has
   arrived, so look at the LiDAR stream rather than at this node.

3. Tune live, no restart required:

   ```bash
   docker exec test_sam3_container bash -lc 'source /opt/ros/jazzy/setup.bash && source /tmp/sam3_ws_install/setup.bash && ros2 param set /sam3_mask_transform max_pairing_skew 0.08'
   ```

Boundaries publish empty (rather than going silent) once a side has not been refreshed
for `boundary_timeout`, so a stalled segmenter reads as "no boundary" downstream
instead of leaving its last output standing.

Earlier failure modes, still worth ruling out:

- Zero-stamped camera input (`sec=0, nanosec=0`) propagating into contours. The
  segmentation node now substitutes current ROS time and warns once; the transform
  node refuses to pair zero-stamped headers.
- Mixed time domains (bag time vs wall clock). Stamps here are only ever compared to
  each other, never to the node clock, so `use_sim_time` is not required — but the two
  input streams must still share one domain.

  ```bash
  docker exec test_sam3_container bash -lc 'source /opt/ros/jazzy/setup.bash && source /tmp/sam3_ws_install/setup.bash && ros2 topic echo /lidar_2d_projection --once | sed -n "1,8p"'
  docker exec test_sam3_container bash -lc 'source /opt/ros/jazzy/setup.bash && source /tmp/sam3_ws_install/setup.bash && ros2 topic echo /sam3_left_contour --once | sed -n "1,8p"'
  ```

---

## SphereFormer (ROS 2 Jazzy) Build and Run

Use these commands from the repository root to build and run the SphereFormer service.

### Build SphereFormer image

```bash
docker compose --profile runtime build sphereformer_node
```

### Rebuild and start SphereFormer

```bash
docker compose --profile runtime up -d --build sphereformer_node
```

### Start without rebuilding

```bash
docker compose --profile runtime up -d sphereformer_node
```

### Follow runtime logs

```bash
docker compose logs --tail=120 -f sphereformer_node
```

### Stop SphereFormer

```bash
docker compose --profile runtime stop sphereformer_node
```

### Notes
- The SphereFormer container builds and runs the `perception_common` and `sphereformer_ros` packages in-container.
- If Python source files change, prefer `up -d --build` so the image includes your latest code.
- Current segmentation output publishes all classes with RGB class colors on the SphereFormer segmentation topic. In RViz, set PointCloud2 `Color Transformer` to `RGB8`.

---

## CLRerNet (ROS 2 Jazzy) Build and Run

CLRerNet lane detection on Python 3.12 / torch 2.7.1+cu128. The CUDA 12.8 toolchain is required so kernels compile with `sm_120` (Blackwell / RTX 50-series) support.

### Build CLRerNet image

```bash
docker compose --profile runtime build clrernet_node
```

The first build takes 15–25 min because `mmcv` 2.2, `mmdet` 3.3, and the lane NMS CUDA extension are all compiled from source against the installed torch (no Python 3.12 wheels exist upstream).

### Rebuild and start CLRerNet

```bash
docker compose --profile runtime up -d --build clrernet_node
```

### Start without rebuilding

```bash
docker compose --profile runtime up -d clrernet_node
```

### Follow runtime logs

```bash
docker logs -f perception_clrernet_node
```

### Stop CLRerNet

```bash
docker compose --profile runtime stop clrernet_node
```

### Notes

- Requires `perception-cuda-base:latest` built on CUDA 12.8; older 12.6 base image will not emit `sm_120` kernels and inference will fail with `no kernel image is available for execution on the device` on RTX 50-series GPUs.
- The container launches two nodes: `clrernet_lane_detection` (image → lane polylines) and `clrernet_lane_transform` (lane polylines + projected lidar → 3D left/right/centerline `PointCloud2`).
- Subscribes to the raw `/camera_fl/image` for lane detection and the lidar 2D projection topic for 3D lifting; topic names are read from `perception_common/topics.yaml`. `cv_bridge` debayers `bayer_rggb8` in-process, avoiding the ~0.5s standing backlog on `image_proc`'s `/camera_fl/image_color`.
- Model checkpoint `clrernet_culane_dla34_ema.pth` is fetched from the upstream GitHub release during the image build; no local download required.
- The backbone (`DLANet`) is initialised with `pretrained=False` because the full checkpoint overwrites the ImageNet weights anyway, and the upstream ImageNet mirror (`dl.yf.io`) is unreliable.

### Vendored source and patches

The `clrernet` source tree (previously a git submodule at `src/clrernet_ros/src/clrernet`) is now vendored directly into this repository so the following patches can be committed:

- `configs/clrernet/base_clrernet.py` — `pretrained=False` on the DLA backbone (skip flaky ImageNet prefetch).
- `libs/datasets/pipelines/alaug.py` — conditional kwargs for `albumentations` 1.4.10, which otherwise raises `bbox_params must be specified for bbox transformations` when the test pipeline runs without bounding boxes.
- `libs/models/layers/nms/src/nms_kernel.cu`, `nms.cpp` — migrated deprecated torch APIs (`tensor.type()` → `tensor.scalar_type()` / `tensor.is_cuda()`, `tensor.data<T>()` → `tensor.data_ptr<T>()`) so the lane NMS CUDA extension builds against modern torch.

---

## Troubleshooting

### `librmw_zenoh_cpp.so: cannot open shared object file`

This means Zenoh RMW is selected but not present in the built image.

1. Ensure `docker/Dockerfile.sam` contains `ros-jazzy-rmw-zenoh-cpp` in the apt install list.
2. Rebuild the image:
   ```bash
   docker compose build --no-cache sam3_ros
   ```
3. Re-run:
   ```bash
   docker compose up sam3_ros
   ```

### `ModuleNotFoundError: No module named 'ament_package'`

This means Python ROS build tooling is missing in the image.

1. Ensure `docker/Dockerfile.sam` contains `python3-ament-package` in the apt install list.
2. Rebuild:
   ```bash
   docker compose build --no-cache sam3_ros
   ```

### "permission denied while trying to connect to the docker API at unix:///var/run/docker.sock"

This error means your user does not have permission to access the Docker daemon socket. To fix:

1. Add your user to the docker group:
   ```bash
   sudo usermod -aG docker $USER
   ```
2. Log out and log back in (or reboot) for group changes to take effect.
3. Verify with:
   ```bash
   docker run hello-world
   ```

If you still see this error, ensure the Docker service is running:
```bash
sudo systemctl status docker
sudo systemctl start docker
```
