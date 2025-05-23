#!/bin/bash

# Export PYTHONPATH for custom directories
echo "Setting PYTHONPATH..."
export PYTHONPATH=/home/dev/Documents/Autonomous-Driving-Perception-System:$PYTHONPATH
# export PYTHONPATH=$PWD/src/clrernet_ros/src:$PWD/src/clrernet_ros/src/clrernet:$PYTHONPATH

# Source the ROS workspace
echo "Sourcing the ROS workspace"
source devel/setup.bash

# Activate YOLOv9 conda environment and launch YOLOv9 nodes in a new terminal tab
echo "Launching YOLOv9 object detection in a new terminal tab..."
gnome-terminal --tab --title="YOLOv9 Object Detection" -- bash -c '
    source /home/dev/anaconda3/etc/profile.d/conda.sh &&
    conda activate yolov9 &&
    roslaunch yolov9_ros yolov9_ros.launch &
    sleep 2 &&
    printf "\033]0;YOLOv9 Object Detection\007" &&
    wait; exec bash'

# Activate SAM2 conda environment and launch SAM2 nodes in a new terminal tab
echo "Launching SAM2 segmentation in a new terminal tab..."
gnome-terminal --tab --title="SAM2 Segmentation" -- bash -c '
    source /home/dev/anaconda3/etc/profile.d/conda.sh &&
    conda activate sam2 &&
    roslaunch sam2_ros sam2_ros.launch &
    sleep 2 &&
    printf "\033]0;SAM2 Segmentation\007" &&
    wait; exec bash'

# Activate Sphereformer conda environment and launch it in a new terminal tab
echo "Launching Sphereformer Lidar Segmentation in a new terminal tab..."
gnome-terminal --tab --title="Sphereformer Lidar Segmentation" -- bash -c '
    source /home/dev/anaconda3/etc/profile.d/conda.sh &&
    conda activate sphereformer &&
    export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH &&
    roslaunch sphereformer_ros sphereformer_ros.launch &
    sleep 2 &&
    printf "\033]0;Sphereformer Lidar Segmentation\007" &&
    wait; exec bash'

# Activate UltraFast conda environment and launch it in a new terminal tab
# echo "Launching UltraFast Lane Detection in a new terminal tab..."
# gnome-terminal --tab --title="UltraFast Lane Detection" -- bash -c 'source /home/dev/miniconda3/etc/profile.d/conda.sh && 
#                                  echo "Launching UltraFast conda environment..." &&
#                                  conda activate lane-det && 
#                                  echo "Launching UltraFast Lane Detection..." &&
#                                  taskset -c 8,9 roslaunch ultrafastv2_ros ultrafastv2_ros.launch; exec bash'

# Launch Transformation Script (transform.py)
echo "Launching Transformation Script in a new terminal tab..."
gnome-terminal --tab --title="Lidar 2D Projection Transform" -- bash -c 'source /home/dev/anaconda3/etc/profile.d/conda.sh &&
    conda activate yolov9 &&
    python /home/dev/Documents/Autonomous-Driving-Perception-System/src/transform.py; exec bash'


# Wait for all processes to finish
wait

echo "Perception system has been successfully enabled."