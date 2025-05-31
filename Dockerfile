# Base image for ROS Noetic on Ubuntu 20.04 Focal
FROM ros:noetic-ros-base-focal


# Install dependencies for the navigation and update other tools
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends\
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*


COPY . /opt/barracuda-navigation

# Set working directory
WORKDIR /opt

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /opt/barracuda-navigation/catkin_ws && \
    catkin_make" 

WORKDIR /opt

# Source the workspace on container start
CMD ["/bin/bash", "/opt/barracuda-navigation/entrypoint.sh"]
