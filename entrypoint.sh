#!/usr/bin/bash
source /opt/ros/noetic/setup.bash

# Build & Source catkin_ws
cd barracuda-navigation/catkin_ws
catkin_make
source devel/setup.bash

# # Start the launch file (add --wait after other node starts roscore)
# roslaunch barracuda_navigation barracuda_navigation.launch 
