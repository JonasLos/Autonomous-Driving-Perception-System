## Plan: SphereFormer Jazzy Docker Options

SphereFormer is already packaged as ROS 2, but its vendored SparseTransformer backend is tied to a PyTorch 1.8 / CUDA 11 generation and the current sphereformer Dockerfile is only a scaffold. The recommended path is still a pure ROS 2 Jazzy / Ubuntu 24.04 container, but the next implementation work should now be driven by a specific container shape and a bounded decision gate: build a CUDA-capable Jazzy image, prove whether SparseTransformer can compile on a modern toolchain, and switch to a backend replacement if that proof fails quickly. A legacy sidecar remains a contingency only.

**Solution tracks considered**
1. Recommended: keep a pure Jazzy / Noble runtime and port the SphereFormer CUDA extension stack to a modern PyTorch / CUDA 12 toolchain inside Docker.
2. Short-lived probe only: keep Jazzy / Noble but embed a separate legacy Python environment for the old ML stack. This is likely brittle on a 50-series GPU and should only be used to collect compile/runtime evidence.
3. Contingency: run SphereFormer in a legacy sidecar image and bridge to Jazzy over ROS 2 networking or Zenoh if deployment constraints are relaxed later.
4. Fallback if the extension port is not worth it: replace the SphereFormer backend with a maintained LiDAR segmentation model while keeping the same ROS 2 node interface and topics.

**Steps**
1. Phase 1: establish a fail-fast Jazzy baseline in Docker. Create a new, lean `Dockerfile.cuda-base` as a sibling to the CPU-only base. This new base will contain only the OS (`nvidia/cuda:12.6.1-runtime-ubuntu24.04`), ROS 2 `ros-base`, and essential build tooling (`colcon`, `rosdep`). Heavy ML libraries will be installed in the final `Dockerfile.sphereformer` to keep the base reusable and its build cache stable. This blocks all later steps.
2. Phase 2: move sphereformer_ros dependency ownership into Docker. Create a `requirements-sphereformer.txt` file and use `uv` (for speed and reproducibility, mirroring the `sam3_ros` pattern) to install modern versions of `torch`, `spconv-cu121`, `torch-scatter`, `timm`, etc. The legacy `sphereformer_env.yml` will be used for reference only. This can run in parallel with step 3.
3. Add a dedicated extension smoke-build stage for SparseTransformer and related CUDA packages before running colcon. Use a multi-stage Dockerfile so the builder stage compiles sptr and other CUDA-linked dependencies first, and the runtime stage stays smaller. The objective is to fail on sptr import/build first, not deep inside a ROS launch. This runs in parallel with step 2 and blocks step 4.
4. Port vendored SparseTransformer for PyTorch 2 / CUDA 12. Remove THC header usage, replace deprecated torch.cuda.*Tensor allocation patterns with current ATen or torch.empty-style allocations, and add explicit architecture targeting through Docker build args or TORCH_CUDA_ARCH_LIST so the image can compile for the host GPU generation. This depends on step 3.
5. Harden sphereformer_ros runtime for the modern stack. Remove unconditional CUDA-only calls, make checkpoint loading device-safe, and verify that imports work from the installed package layout instead of assuming repo-root execution. This depends on step 4.
6. Rework the runtime service in docker-compose to match the proven SAM3 pattern: GPU reservation, host networking or chosen RMW, adequate shared memory, package-select colcon build, and a deterministic launch command for sphereformer_ros. This depends on steps 1 through 5.
7. Verification phase: validate image build, import of sptr and model construction, colcon build of sphereformer_ros, and a manual PointCloud2 replay against Jazzy topics to confirm the node starts and publishes expected outputs. This depends on step 6.
8. Decision gate: if step 4 shows the extension port is too costly or blocked by incompatible upstream dependencies, switch to one of two bounded fallbacks. Fallback A keeps Jazzy / Noble and preserves ROS topics but swaps the model backend. Fallback B isolates the legacy backend in a sidecar only if deployment requirements are later relaxed.

**Relevant files**
- `docker/Dockerfile.cuda-base` — create a new CUDA 12.6 / Ubuntu 24.04 / ROS 2 Jazzy base layer for GPU packages.
- `docker/Dockerfile.sphereformer` — replace the placeholder venv-and-ignore-failures flow with a multi-stage, CUDA-aware, fail-fast build.
- `docker/Dockerfile.base` — keep as the CPU-oriented base; reuse patterns only, not inheritance.
- `docker-compose.yml` — re-enable and harden the sphereformer runtime service definition.
- `src/sphereformer_ros/package.xml` — complete runtime dependency declarations for the ROS package.
- `src/sphereformer_ros/setup.py` — align installed files and Python entrypoint packaging with the final image layout.
- `src/sphereformer_ros/requirements-sphereformer.txt` — create this new file to explicitly lock modern Python dependencies for `uv`.
- `src/sphereformer_ros/sphereformer_ros.py` — remove unconditional CUDA assumptions, verify install-time import paths, and keep the ROS topic contract stable.
- `src/sphereformer_ros/src/SphereFormer/third_party/SparseTransformer/setup.py` — modernize extension build flags and architecture targeting.
- `src/sphereformer_ros/src/SphereFormer/third_party/SparseTransformer/src/sptr/attention/attention_cuda.cpp` — replace THC-era extension code patterns.
- `src/sphereformer_ros/src/SphereFormer/third_party/SparseTransformer/src/sptr/precompute/precompute.cpp` — replace THC-era extension code patterns.
- `src/sphereformer_ros/src/SphereFormer/third_party/SparseTransformer/src/sptr/rpe/relative_pos_encoding_cuda.cpp` — replace THC-era extension code patterns.
- `src/sphereformer_ros/sphereformer_env.yml` — treat as legacy reference only, not the authoritative Docker environment.

**Verification**
1. Build the sphereformer image without swallowing failures and confirm the first failing stage is isolated to either ROS packaging or CUDA extension compilation.
2. Inside the image, verify torch.cuda.is_available(), import spconv, and import sptr before any ROS launch attempt.
3. Run colcon build for sphereformer_ros only and confirm the installed package exposes the expected launch file and Python entrypoint.
4. Launch the node in Docker against a recorded or live PointCloud2 source and verify segmentation and boundary topics publish on Jazzy.
5. If the extension port is attempted, capture the exact compiler errors and use them as the decision gate for continuing the port versus switching to a fallback track.

**Decisions**
- Included scope: planning a Dockerized path for sphereformer_ros on ROS 2 Jazzy with possible solution tracks and a recommended primary path.
- Excluded scope: full implementation, model-accuracy tuning, and full end-to-end validation of sphereformer_ros in ROS 2 beyond the build and smoke-test path.
- Chosen constraint: prefer a pure ROS 2 Jazzy / Ubuntu 24.04 deployment over a legacy sidecar.
- Chosen container direction: create a CUDA-capable sibling base instead of extending the current CPU-only base.
- Working assumption: SparseTransformer is mandatory for the current SphereFormer model and has no CPU-only fallback, so any direct build path must solve or replace that extension layer.
- Working assumption: the current ROS contract can be preserved even if the backend model changes, because the node's subscription, post-processing, and PointCloud2 publishing are mostly backend-agnostic.

**Further Considerations**
1. Recommended decision rule: time-box the porting effort for `SparseTransformer` to a strict **one-day spike**. The goal is not to complete the port, but to prove its viability by successfully compiling the C++ extension against PyTorch 2.x after removing the deprecated `<THC/THC.h>` headers. If compilation remains blocked by fundamental issues after one day, the effort should be halted, and the team should pivot to the backend replacement track.
2. Recommended container shape: use a multi-stage Dockerfile with a CUDA devel builder stage and a CUDA runtime stage so CUDA extension compilation and ROS workspace build are separated and cached independently.
3. Recommended dependency ownership: use `apt` for ROS/system packages, `uv` with a `requirements-sphereformer.txt` file for Python wheels, and vendored source only where CUDA extensions must be built locally.
4. Port-versus-replace decision signal: backend replacement is materially cheaper if the team values delivery speed and maintainability more than exact model parity, because the current node logic after inference is already reusable.
5. Optimization opportunity: if the backend is replaced (Track B), evaluate if the new model can benefit from pre-computation (e.g., creating a static voxel grid at startup) to reduce per-frame latency.
6. Not recommended as a target architecture: a Python 3.8 / PyTorch 1.8 compatibility environment inside Noble purely to mimic the original SphereFormer paper stack, because it conflicts with the modern GPU, modern host OS, and long-term maintainability goals.
