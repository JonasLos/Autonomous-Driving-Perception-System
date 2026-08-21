# Perception System for Autonomous Driving

![Demo Video](https://github.com/user-attachments/assets/e6f0876c-9d8a-4220-9c69-085cf4ed74b0) [ROS1 version]

A ready-to-use system for autonomous driving, featuring YOLOv9 for object detection, SAMv2 for road segmentation, Ultrafast lane detection, and Sphereformer LiDAR segmentation. Fully integrated with ROS for seamless deployment in robotic systems. Ideal for developers and researchers focused on autonomous vehicle technologies.

Daily implementation notes and integration updates are tracked in [CHANGELOG.md](CHANGELOG.md).

---

## Features
- **Object Detection**: Utilizes YOLOv9 for real-time, accurate object detection.
- **Road Segmentation**: Employs SAMv2 for robust and precise road segmentation.
- **Lane Detection**: Implements clrernet lane detection for high-speed lane recognition.
- **LiDAR Segmentation**: Uses Sphereformer for advanced 3D LiDAR-based segmentation.
- **ROS Integration**: Fully integrated with ROS for seamless deployment and easy integration into robotic systems.

---

## Installation

### Prerequisites

Before installing the Autonomous Driving Perception System, ensure the following prerequisites are met:
-- **ROS**: This repository is being migrated to ROS2 (Jazzy). For ROS2 Jazzy installation instructions, visit the ROS2 documentation and install the `ros-jazzy-desktop` packages for your OS.
- **Python**: Python versions are algorithm dependent. Please use the `environment.yml` files for the correct versions.
- **Dependencies**: All necessary Python libraries and dependencies are listed in the corresponding `environment.yaml` files.

### Clone the Repository with Submodules
```bash
git clone --recursive https://github.com/ParimiHarsha/Autonomous-Driving-Perception-System.git
cd Autonomous-Driving-Perception-System
```

### Build the ROS2 Workspace
```bash
colcon build --merge-install
source install/setup.bash
```

### Install Custom Messages On The Host
The standalone interface packages now live under `src/custom_msgs/` so they can be built independently from the heavier perception nodes. This host overlay now installs `clrernet_msgs`, `sam2_msgs`, `yolov9_msgs`, and `yolo_msgs`.

```bash
bash scripts/install_host_custom_msgs.sh
source /opt/ros/jazzy/setup.bash
source ~/ros_drivers/install/setup.bash
source ~/.local/opt/adps_custom_msgs/setup.bash
```

If you use another workspace such as `~/ros_drivers`, source that workspace before the custom message overlay. `ros2 bag record` only sees message definitions from the currently active overlay chain.

You can verify the overlay is active with:

```bash
ros2 pkg prefix clrernet_msgs
ros2 pkg prefix sam2_msgs
ros2 pkg prefix yolo_msgs
```

### Replaying A Bag Offline

The perception containers take raw sensor topics, so a recorded bag can drive the whole stack
with no code changes — useful for debugging without the vehicle.

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd &                       # compose defines no router
USE_SIM_TIME=true docker compose --profile runtime up -d \
    transform_node sphereformer_node sam3_ros yolo_node clrernet_node
scripts/play_rosbag.sh /path/to/bag
```

`USE_SIM_TIME=true` puts the nodes on `/clock`, which is what makes pausing and rate-scaling
the bag behave — the watchdogs and the pairing deferral read the node clock. Never set it on
the vehicle: there is no `/clock` there. Do not launch `~/ros_drivers` alongside a bag; the
bag *is* the sensors.

See [DOCKER.md](DOCKER.md#offline-bag-replay) for QoS gotchas, the pairing-bound tuning
procedure, and how to iterate on node code without rebuilding images.

To install them into a different prefix, pass the destination as the first argument:

```bash
bash scripts/install_host_custom_msgs.sh /path/to/custom_msgs_install
source /opt/ros/jazzy/setup.bash
source ~/ros_drivers/install/setup.bash
source /path/to/custom_msgs_install/setup.bash
```

### Create and Activate Environments
Each perception module requires its own environment TBD:

#### SAMv2 Installation
```bash
conda create -n sam2 python=3.10
conda activate sam
cd src/road_segmentation/src/segment-anything-2
pip install -e .
cd checkpoints && ./download_ckpts.sh
```

#### YOLOv9 Installation
```bash
conda env create -f src/yolov9ros/src/environment.yaml
conda activate yolo_env
```
##### Download the Trained YOLOv9 model

```bash
cd src/yolov9ros/
```
Download the [trained model](https://drive.google.com/file/d/1UAX-7jSXQJcyRdumn8iXmwjfJxxyC9Tw/view?usp=sharing) here. And save it in yolov9_ros folder

#### Clrernet Lane Detection and Sphereformer
Follow the respective `README.md` files in their directories for installation details.


## Running the Perception System

The entire perception stack can be launched with a single command:

### Enable Perception
```bash
./enable_perception.bash
```

### Disable Perception
```bash
./disable_perception.bash
```

These scripts handle launching and shutting down all perception nodes, including object detection, road segmentation, lane detection, and LiDAR segmentation.

---

## Usage
Once installed, the system can be deployed within a ROS environment. The `enable_perception.bash` script will launch all perception nodes, while individual components can be run manually using their respective launch files and scripts. Ensure the appropriate topics are correctly published and subscribed to within your ROS setup.

### SphereFormer RViz Class Colors
SphereFormer ROS2 output now publishes all segmentation classes and encodes SemanticKITTI class colors directly in the PointCloud2 `rgb` field.

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

## License
This project is licensed under the MIT License. See the `LICENSE` file for more details.

---

## Contact
For questions, issues, or suggestions, please open an issue on the GitHub repository or contact the repository owner.
