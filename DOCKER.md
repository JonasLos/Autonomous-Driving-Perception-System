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
  - `ros-humble-rmw-zenoh-cpp`
   - normalizes `src/SAM3_ROS_NODE` during image build by:
      - creating `scripts/segmentation_node` if missing
      - removing stale `pub_test_image.py` install reference from `CMakeLists.txt`
      - applying default runtime parameters used in this workspace (`use_compressed_image=False`, `text_prompt="road"`, and `/camera_fl/image_color` topic)
- `docker-compose.yml` for `sam3_ros` now:
  - mounts the repository root to `/root/ws`
  - mounts `${HOME}/.cache/huggingface` to `/root/.cache/huggingface`
  - sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
  - builds only `sam3_ros` at runtime:
    - `colcon build --symlink-install --packages-select sam3_ros`

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
docker run --rm test-sam3:latest bash -lc 'ls /opt/ros/humble/lib/librmw_zenoh_cpp.so'
```

If the file exists, Zenoh RMW is installed correctly.

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
- Subscribes to `/camera_fl/image_color` for lane detection and the lidar 2D projection topic for 3D lifting; topic names are read from `perception_common/topics.yaml`.
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

1. Ensure `docker/Dockerfile.sam` contains `ros-humble-rmw-zenoh-cpp` in the apt install list.
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
