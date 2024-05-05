FROM ros:humble-ros-base

# Setup User information 
ARG USERNAME=glados
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Default values for installion prompts
ARG DEBIAN_FRONTEND=noninteractive

# Setup shell environment
SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash

# Install General Dependencies
RUN apt update; apt install build-essential \ 
    sudo \
    -y nano \
    software-properties-common \
    autoconf \
    -y git \
    curl \
    -y libserial-dev \
    cmake \
    python3-pip

# Install ROS packages
RUN apt install ros-humble-laser-filters -y \
    ros-humble-laser-geometry -y \
    ros-humble-rplidar-ros -y \
    ros-humble-tf-transformations -y \
    ros-humble-teleop-tools -y \
    ros-humble-teleop-twist-keyboard -y \
    ros-humble-teleop-twist-joy -y \
    ros-humble-teleop-tools-msgs -y \
    ros-humble-nav2-bringup -y \
    ros-humble-navigation2 -y \
    ros-humble-octomap -y \
    ros-humble-octomap-mapping -y \
    ros-humble-octomap-msgs -y \
    ros-humble-octomap-ros -y \
    ros-humble-octomap-server -y \
    ros-humble-octomap-rviz-plugins -y \
    && rm -rf /var/lib/apt/lists/*

ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib

# Add user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN usermod -aG dialout ${USERNAME}

USER $USERNAME

# Install necessary python packages
RUN pip install --user --upgrade numpy; \
    pip install --user --upgrade pyserial

#Setup ROS2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc