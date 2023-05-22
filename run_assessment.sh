#!/usr/bin/bash
cd /home/medrobotics/ROS_Workspaces/stars_ws
gnome-terminal -- roscore
sleep 2
gnome-terminal -- roslaunch rosbridge_server rosbridge_websocket.launch
gnome-terminal -- roslaunch panda_ros cartesian_impedance_equilibrium_controller.launch

sleep 3
/home/medrobotics/scripts/run_analytics.sh