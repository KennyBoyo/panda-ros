#!/usr/bin/bash
# cd /home/medrobotics/ROS_Workspaces/stars_ws
# gnome-terminal -- roslaunch panda_ros cartesian_impedance_equilibrium_controller.launch


cd /home/medrobotics/Documents/os3/build
gnome-terminal -- "./testOS3"


cd /home/medrobotics/ROS_Workspaces/stars_ws
gnome-terminal -- roslaunch mrn_aruco aruco_detect_kinect.launch
gnome-terminal -- ./src/panda_ros/scripts/analytics/analytics_calc.py
gnome-terminal -- ./src/panda_ros/scripts/analytics/analytics_vis.py
