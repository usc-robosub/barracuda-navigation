#!/bin/bash

source /opt/ros/noetic/setup.bash
source barracuda-navigation/catkin_ws/devel/setup.bash

# Start the launch file (add --wait after other node starts roscore)
# roslaunch barracuda_navigation barracuda_navigation.launch
roslaunch rrt_star_planner test_rrt_star_planner.launch
