# Perception System for Autonomous Driving

![Demo Video](https://github.com/user-attachments/assets/e6f0876c-9d8a-4220-9c69-085cf4ed74b0) [ROS1 version]

A ready-to-use ROS 2 Jazzy perception stack for autonomous driving: YOLOv9 object detection,
SAM3 road segmentation, CLRerNet lane detection, and SphereFormer LiDAR semantic segmentation,
plus a LiDAR→camera projection node that the 2D stages fuse against.

Every node runs in its own container. There is no conda anywhere in the supported path — each
image carries a Python 3.12 virtual environment matched to that node's framework versions,
because the four models do not share a compatible torch/CUDA combination.

Daily implementation notes and integration updates are tracked in [CHANGELOG.md](CHANGELOG.md).
Container build/run detail, troubleshooting, and replay tuning live in [DOCKER.md](DOCKER.md).

---

## Features
- **Object Detection**: YOLOv9 via `yolo_ros` / `yolo_bringup` (Ultralytics), with LiDAR fusion producing `/fused_bbox`.
- **Road Segmentation**: SAM3 (`src/SAM3_ROS_NODE`), text-prompted, publishing `/sam3_*` contours and 3D boundaries.
- **Lane Detection**: CLRerNet (mmdet 3.3 / mmcv 2.2), publishing `/clrernet/*` lane polylines and a 3D centerline.
- **LiDAR Segmentation**: SphereFormer with the SparseTransformer CUDA extension, publishing SemanticKITTI classes with RGB colors.
- **LiDAR→2D Projection**: `src/transform.py`, publishing `/lidar_2d_projection` — the shared input every 2D stage lifts into 3D.
- **ROS Integration**: ROS 2 Jazzy, `rmw_zenoh_cpp` middleware, host networking, one container per node.

Topic names are centralized in [src/perception_common/topics.yaml](src/perception_common/topics.yaml)
and read by both the nodes and the bag scripts.

---

## Prerequisites

- **Docker Engine + Compose plugin.** Podman is not supported. Install steps: [DOCKER.md](DOCKER.md#official-docker-engine-setup-ubuntu-2404).
- **NVIDIA driver + NVIDIA Container Toolkit.** Every runtime service reserves `driver: nvidia, count: all`.
  CLRerNet is built for `sm_120`, so RTX 50-series (Blackwell) is supported; the CUDA 12.8 base is what makes that work.
- **ROS 2 Jazzy on the host.** Not needed to *build* the stack, but you need it for the Zenoh
  router, `ros2 topic`/`ros2 param` inspection, and the bag scripts.
- **`yq`** — required by `scripts/play_rosbag.sh` for topic resolution.

You do **not** need to `colcon build` the workspace on the host to run the containers. Each
image builds the packages it needs; `yolo_node`, `sam3_ros` and `sphereformer_node` re-run
`colcon build` at container start into `/tmp`, so a host-side `install/` is never sourced.

### Clone the Repository with Submodules
```bash
git clone --recursive https://github.com/ParimiHarsha/Autonomous-Driving-Perception-System.git
cd Autonomous-Driving-Perception-System
```

---

## Images and Services

`docker-compose.yml` defines two build-only services and five node services — four in the
`runtime` profile, plus `sam3_ros`, which declares no profile.

| Service | Image | Dockerfile | Built FROM | Profile |
|---|---|---|---|---|
| `cuda_base_builder` | `perception-cuda-base:latest` | [docker/Dockerfile.cuda-base](docker/Dockerfile.cuda-base) | `nvidia/cuda:12.8.1-devel-ubuntu24.04` | `builder` |
| `base_builder` | `perception-base:latest` | [docker/Dockerfile.base](docker/Dockerfile.base) | `ros:jazzy-ros-base-noble` | `builder` |
| `sphereformer_node` | `perception-sphereformer:latest` | [docker/Dockerfile.sphereformer](docker/Dockerfile.sphereformer) | `perception-cuda-base` | `runtime` |
| `clrernet_node` | `perception-clrernet:latest` | [docker/Dockerfile.clrernet](docker/Dockerfile.clrernet) | `perception-cuda-base` | `runtime` |
| `yolo_node` | `perception-yolo:latest` | [docker/Dockerfile.yolo](docker/Dockerfile.yolo) | `perception-base` | `runtime` |
| `transform_node` | `perception-transform:latest` | [docker/Dockerfile.transform](docker/Dockerfile.transform) | `perception-base` | `runtime` |
| `sam3_ros` | `test-sam3:latest` | [docker/Dockerfile.sam](docker/Dockerfile.sam) | `nvidia/cuda:12.6.1-devel-ubuntu24.04` | *(none — starts with a bare `up`)* |

All runtime services use `network_mode: host`, `ipc: host`, `restart: "no"`, and reserve all
GPUs. `sam3_ros` declares no profile, so a bare `docker compose up` starts it alone; the other
four need `--profile runtime`.

### Build order

The two GPU nodes and the two CPU-base nodes depend on locally-tagged base images, so the
bases must be built first — Compose will not build them implicitly.

```bash
# 1. Bases (order between the two does not matter)
docker compose --profile builder build cuda_base_builder base_builder

# 2. Node images
docker compose --profile runtime build sphereformer_node clrernet_node yolo_node transform_node
docker compose build sam3_ros
```

`clrernet_node` takes 15–25 minutes on a first build: mmcv 2.2.0, mmdet 3.3.0 and the lane NMS
CUDA extension are all compiled from source against torch 2.7.1+cu128, because no Python 3.12
wheels exist upstream. `sphereformer_node` compiles the SparseTransformer CUDA extension in a
separate builder stage for `TORCH_CUDA_ARCH_LIST="8.6;9.0+PTX"`.

### What each image installs

- **cuda-base** — ROS 2 Jazzy `ros-base`, `rmw_zenoh_cpp`, colcon/rosdep/vcstool, and a venv at
  `/opt/venv` with `uv`. [docker/ros_entrypoint.sh](docker/ros_entrypoint.sh) sources ROS and
  activates that venv for every container built on it.
- **base** — the same ROS 2 tooling and `rmw_zenoh_cpp` on the stock Jazzy image, no CUDA toolchain.
- **sphereformer** — `requirements-sphereformer.txt`, the compiled `sptr` extension copied from
  the builder stage, and `pyg-lib` (installed separately in the runtime stage so editing the
  requirements file does not invalidate the CUDA compile). Builds `perception_common` + `sphereformer_ros`.
- **clrernet** — torch 2.7.1 / torchvision 0.22.1 (cu128), mmengine 0.10.5, mmcv 2.2.0 from
  source, mmdet 3.3.0 with its `mmcv<=2.2.0` bound patched, `requirements-clrernet.txt`, a
  vendored `ros2_numpy`, and the lane NMS extension. Builds `clrernet_msgs`, `perception_common`,
  `clrernet_ros`.
- **yolo** — a separate venv at `/opt/yolo_venv` with `numpy<2`, `ultralytics`, `lap`, `empy`,
  `dill`, plus `src/Custom_YOLO_ROS/requirements.txt`. Builds `yolo_msgs`, `yolov9_msgs`,
  `yolo_ros`, `yolo_bringup`, `yolov9_ros`, `perception_common`.
- **transform** — a venv at `/opt/transform_venv` with `numpy<2`, `pyyaml`, `torch`, `cython`,
  `transforms3d`, a vendored `ros2_numpy`, and `ros-jazzy-tf-transformations`. Builds the
  workspace with `--merge-install`.
- **sam3** — Jazzy `ros-base` plus `cv_bridge`, `message_filters`, `sensor_msgs_py`,
  `tf_transformations`, `image_transport`/`compressed_image_transport`, `vision_msgs`, and `uv`.
  The SAM3 package's own dependencies come from `uv sync` against `src/SAM3_ROS_NODE/pyproject.toml`.

### Model weights

| Component | Path / source | How it gets there |
|---|---|---|
| YOLOv9 | `src/Custom_YOLO_ROS/yolov9_custom_model_with_signs_and_cones.pt` | Tracked in the `Custom_YOLO_ROS` submodule, so it is both baked into the image by `COPY` and shadowed by the repo bind mount at run time. Override with `YOLO_MODEL_PATH`. |
| SphereFormer | `src/sphereformer_ros/src/SphereFormer/model_semantic_kitti.pth` | Excluded from the build context by [.dockerignore](.dockerignore), so it is **bind-mounted read-only** into the source tree at run time. Override with `SPHEREFORMER_CHECKPOINT` to keep weights outside the repo. |
| CLRerNet | `clrernet_culane_dla34_ema.pth` | Downloaded from the upstream GitHub release during the image build. Nothing to fetch by hand. |
| SAM3 | Hugging Face Hub | Pulled at run time. Set `HF_TOKEN`; `${HOME}/.cache/huggingface` is mounted so it is fetched once. |

The DLA34 backbone is initialised with `pretrained=False` on purpose — the full CLRerNet
checkpoint overwrites those weights anyway, and the upstream ImageNet mirror is unreliable.

---

## Running the Perception System

### 1. Start a Zenoh router on the host

Compose defines **no** router service. Without one, the containers and any host-side ROS
process never discover each other, and the only symptom is that nothing happens.

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd &
```

### 2. Bring the nodes up

```bash
docker compose --profile runtime up -d \
    transform_node sphereformer_node yolo_node clrernet_node
docker compose up -d sam3_ros          # no profile on this service
```

Logs and shutdown:

```bash
docker compose logs -f yolo_node
docker compose --profile runtime stop transform_node sphereformer_node yolo_node clrernet_node
docker compose down
```

### Environment variables

`x-common-env` in `docker-compose.yml` is shared by all five node services, so the ROS graph
settings cannot drift apart between nodes:

| Variable | Default | Meaning |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | Must match anything you run on the host. |
| `RMW_IMPLEMENTATION` | `rmw_zenoh_cpp` | Middleware. Fast DDS also works; see [DOCKER.md](DOCKER.md#switching-middleware-rmw). |
| `USE_SIM_TIME` | `false` | `true` puts every node on `/clock`. **Never set on the vehicle** — there is no `/clock` there and every watchdog and deferral would freeze at 0. |
| `OMP_NUM_THREADS` / `MKL_NUM_THREADS` | `4` | Caps each node's torch CPU thread pool. PyTorch sizes it from the host core count, not the container's share, so five uncapped nodes oversubscribe the machine badly. |

Per-service:

| Variable | Service | Default |
|---|---|---|
| `YOLO_MODEL_PATH` | `yolo_node` | the tracked `.pt` under `src/Custom_YOLO_ROS/` |
| `YOLO_USE_FUSION` | `yolo_node` | `True` in Compose (the image itself defaults to `False`) |
| `YOLO_CONFIG_DIR` | `yolo_node` | `/tmp/Ultralytics` |
| `SPHEREFORMER_CHECKPOINT` | `sphereformer_node` | in-repo `model_semantic_kitti.pth` |
| `HF_TOKEN` | `sam3_ros` | empty — read from a gitignored `.env` if present |

Compose reads `.env` from the repo root. It is gitignored, so the vehicle-safe values are the
ones you get by doing nothing.

---

## Install Custom Messages On The Host

The standalone interface packages live under `src/custom_msgs/` so they can be built
independently from the heavier perception nodes. This host overlay installs `clrernet_msgs`,
`sam2_msgs`, `yolov9_msgs`, and `yolo_msgs` — you need it to `ros2 topic echo` the stack's
custom messages or to `ros2 bag record` them.

```bash
bash scripts/install_host_custom_msgs.sh
source /opt/ros/jazzy/setup.bash
source ~/ros_drivers/install/setup.bash
source ~/.local/opt/adps_custom_msgs/setup.bash
```

If you use another workspace such as `~/ros_drivers`, source that workspace before the custom
message overlay. `ros2 bag record` only sees message definitions from the currently active
overlay chain.

Verify the overlay is active:

```bash
ros2 pkg prefix clrernet_msgs
ros2 pkg prefix sam2_msgs
ros2 pkg prefix yolo_msgs
```

To install into a different prefix, pass the destination as the first argument:

```bash
bash scripts/install_host_custom_msgs.sh /path/to/custom_msgs_install
source /path/to/custom_msgs_install/setup.bash
```

---

## Replaying A Bag Offline

The containers subscribe to raw sensor topics exactly as they do on the vehicle, so a recorded
bag drives the whole stack with no code changes.

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd &                       # compose defines no router
USE_SIM_TIME=true docker compose --profile runtime up -d \
    transform_node sphereformer_node yolo_node clrernet_node
USE_SIM_TIME=true docker compose up -d sam3_ros
scripts/play_rosbag.sh /path/to/bag
```

`USE_SIM_TIME=true` puts the nodes on `/clock`, which is what makes pausing and rate-scaling the
bag behave — the watchdogs and the pairing deferral read the node clock. Do not launch
`~/ros_drivers` alongside a bag; the bag *is* the sensors.

`scripts/play_rosbag.sh --help` covers `--rate`, `--loop`, `--start-offset`, `--pause`,
`--category` and `--dry-run`. By default it plays only the `replay_input` topics
(`/camera_fl/image`, `/camera_fl/camera_info`, `/lidar_tc/velodyne_points`, `/tf_static`) and
refuses to replay topics the containers publish themselves.

### Iterating on node code without rebuilding

`yolo_node` and `sam3_ros` bind-mount the repo and `colcon build` at container start, so they
pick up edits on `docker compose restart`. `transform_node`, `clrernet_node` and
`sphereformer_node` bake their source in with `COPY . .`. To iterate on those, overlay
[docker-compose.replay.yml](docker-compose.replay.yml), which adds targeted read-only mounts:

```bash
USE_SIM_TIME=true docker compose \
    -f docker-compose.yml -f docker-compose.replay.yml \
    --profile runtime up -d transform_node clrernet_node
```

Toggling sim time never needs a rebuild — it is an environment variable.

See [DOCKER.md](DOCKER.md#offline-bag-replay) for QoS gotchas (a BEST_EFFORT-recorded image
topic silently never reaches YOLO's RELIABLE subscription) and the pairing-bound A/B procedure.

---

## Usage

### SphereFormer RViz Class Colors
SphereFormer ROS 2 output publishes all segmentation classes and encodes SemanticKITTI class
colors directly in the PointCloud2 `rgb` field.

RViz setup:
1. Add a `PointCloud2` display for the SphereFormer segmentation topic.
2. Set `Color Transformer` to `RGB8`.
3. Keep boundary topics enabled separately if you also want road edge overlays.

Semantic classes are the learned IDs 0-19:
- 0 unlabeled
- 1 car
- 2 bicycle
- 3 motorcycle
- 4 truck
- 5 other-vehicle
- 6 person
- 7 bicyclist
- 8 motorcyclist
- 9 road
- 10 parking
- 11 sidewalk
- 12 other-ground
- 13 building
- 14 fence
- 15 vegetation
- 16 trunk
- 17 terrain
- 18 pole
- 19 traffic-sign

---

## Legacy and unused files

These are kept for history and are **not** part of the supported Docker path:

- [docker/Dockerfile](docker/Dockerfile) — the original Ubuntu 18.04 / ROS 1 image. No Compose service references it.
- [docker/Dockerfile.cylinder3d](docker/Dockerfile.cylinder3d) — Cylinder3D experiment; superseded by SphereFormer, no Compose service.
- `scripts/enable_perception.sh` / `scripts/disable_perception.sh` — pre-container host scripts that assume conda environments and hard-coded `/home/dev` paths. Use Docker Compose instead.
- `src/legacy_code/` — the SAM2 ROS node, replaced by `src/SAM3_ROS_NODE`.
- [.github/workflows/docker-build.yml](.github/workflows/docker-build.yml) — pushes a `perception_image` tag that the current Compose file no longer produces.

---

## License
This project is licensed under the MIT License. See the `LICENSE` file for more details.

---

## Contact
For questions, issues, or suggestions, please open an issue on the GitHub repository or contact the repository owner.
