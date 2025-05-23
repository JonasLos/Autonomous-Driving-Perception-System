#!/bin/bash

# Update package list and install necessary dependencies
echo "Updating package list and installing dependencies..."
sudo apt-get update

# Set up ROS repository in sources.list and keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update

# Install ROS Melodic Desktop Full
sudo apt-get install -y ros-melodic-ros-base

# Install dependencies for building ROS packages
echo "Installing dependencies for building ROS packages..."
sudo apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-pkg

# Initialize rosdep
sudo rosdep init
rosdep update

# Install jsk_recognition_msgs and related message packages
echo "Installing jsk_recognition_msgs..."
sudo apt-get install -y ros-melodic-jsk-recognition-msgs

# Install radar_msgs via package manager
echo "Installing radar_msgs..."
sudo apt-get update
sudo apt-get install -y ros-melodic-radar-msgs

# Set up environment variables for ROS
echo "Setting up ROS environment..."
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source /opt/ros/melodic/setup.bash

# Start a bash shell
exec "$@"