# Base image for ROS Noetic on Ubuntu 20.04 Focal
FROM ros:noetic-ros-base-focal

COPY . /opt/barracuda-navigation

# Set working directory
WORKDIR /opt

# Install dependencies for the navigation and update other tools
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# Source the workspace on container start
CMD ["/bin/bash", "/opt/barracuda-navigation/entrypoint.sh"]
