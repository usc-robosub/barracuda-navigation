#!/bin/bash

source /opt/ros/noetic/setup.bash
source barracuda-navigation/catkin_ws/devel/setup.bash

chmod +x /opt/barracuda-navigation/catkin_ws/src/rrt_star_planner/scripts/test_rrt_star_planner.py

roslaunch barracuda_navigation barracuda_navigation.launch --wait
