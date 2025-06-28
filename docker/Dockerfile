# Use the official Ubuntu 18.04 base image
FROM ubuntu:18.04

# Set non-interactive mode for APT
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary packages including git
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
    vim \
    gedit 

# Set up locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Set up a non-root user with sudo access (username: dev)
RUN useradd -ms /bin/bash dev && \
    echo "dev:dev" | chpasswd && \
    adduser dev sudo && \
    echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to the non-root user
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

# Set timezone non-interactively
RUN sudo ln -fs /usr/share/zoneinfo/UTC /etc/localtime && \
    echo "UTC" | sudo tee /etc/timezone && \
    sudo apt-get update && \
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata

# Set working directory inside the cloned repo
WORKDIR /home/ubuntuuser/Autonomous-Driving-Perception-System

# Set the default command to run the ROS installation script and start bash
CMD ["/bin/bash"]
