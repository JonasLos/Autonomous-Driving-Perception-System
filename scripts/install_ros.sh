#!/bin/bash

# Update package list and install necessary dependencies
echo "This script can install ROS Melodic (legacy) or ROS2 Jazzy (use argument 'jazzy')."
if [ "$1" = "jazzy" ]; then
	echo "Installing ROS2 Jazzy (minimal steps)..."
	sudo apt-get update
	# Add ROS2 apt repository and key (platform-specific; user should verify)
	sudo apt-get install -y curl gnupg lsb-release
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - || true
	sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] https://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
	sudo apt-get update
	sudo apt-get install -y ros-jazzy-desktop
	echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
	source /opt/ros/jazzy/setup.bash
	echo "ROS2 Jazzy installed (best-effort). Review ROS2 installation docs for complete setup." 
	exit 0
fi

echo "Installing ROS Melodic (legacy). Use '$0 jazzy' to attempt ROS2 Jazzy installation instead." 
sudo apt-get update

# Set up ROS repository in sources.list and keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ bionic main" > /etc/apt/sources.list.d/autonomoustuff-public.list'

# Install ROS Melodic Desktop Full
sudo apt-get install -y ros-melodic-desktop

# Install dependencies for building ROS1 packages (minimal)
echo "Installing minimal dependencies for legacy ROS (catkin packages omitted)..."
# NOTE: catkin-specific packages (python-catkin-pkg, python3-catkin-pkg-modules) are intentionally omitted
# to keep the repository compatible with ROS2/ament workflows. Install them manually if you need ROS1 support.
sudo apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential libbullet-dev libpcl-dev
sudo apt install -y python3-rospkg-modules python3-empy # keep general Python ROS tooling

# Initialize rosdep
sudo rosdep init || true
rosdep update || true

# Install jsk_recognition_msgs and related message packages
echo "Installing jsk_recognition_msgs..."
sudo apt-get install -y ros-melodic-jsk-recognition-msgs

# Install radar_msgs via package manager
echo "Installing radar_msgs..."
sudo apt-get install -y ros-melodic-radar-msgs

# Set up environment variables for ROS
echo "Setting up ROS environment..."
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source /opt/ros/melodic/setup.bash

# Start a bash shell
exec "$@"