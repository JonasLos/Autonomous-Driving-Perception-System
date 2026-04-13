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

## Troubleshooting

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
