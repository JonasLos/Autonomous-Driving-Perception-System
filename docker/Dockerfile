# Use the official Ubuntu 18.04 base image
FROM ubuntu:18.04

# Set non-interactive mode for APT
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    sudo \
    lsb-release \
    software-properties-common \
    locales \
    gnupg2 \
    python3-pip \
    git \
    nano \
    bash \
    tzdata \
    bzip2  # Required for Miniconda

# Set up locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Create a non-root user with sudo privileges
RUN useradd -ms /bin/bash dev && \
    echo "dev:dev" | chpasswd && \
    adduser dev sudo && \
    echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Copy ROS installation script into the container
COPY ../scripts/install_ros.sh /home/dev/install_ros.sh
RUN chmod +x /home/dev/install_ros.sh

# Install ROS inside the image (non-interactive)
RUN bash -c "source ~/.bashrc && /home/dev/install_ros.sh"

# Switch to non-root user
USER dev
WORKDIR /home/dev

# Install Conda
RUN curl -o ~/miniconda.sh -sSL https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    bash ~/miniconda.sh -b -p $HOME/miniconda && \
    rm ~/miniconda.sh && \
    echo "export PATH=$HOME/miniconda/bin:\$PATH" >> ~/.bashrc && \
    echo "source ~/.bashrc" >> ~/.bash_profile && \
    export PATH=$HOME/miniconda/bin:$PATH && \
    conda init bash

# Set timezone
RUN sudo ln -fs /usr/share/zoneinfo/UTC /etc/localtime && \
    echo "UTC" | sudo tee /etc/timezone && \
    sudo apt-get update && \
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata

# Set working directory inside the repo
WORKDIR /home/dev/Documents/Autonomous-Driving-Perception-System

# Default to bash shell
CMD ["/bin/bash"]
