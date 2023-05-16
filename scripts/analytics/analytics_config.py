import numpy as np

datapipe_prefix = "/home/medrobotics/ROS_Workspaces/stars_ws/src/panda_ros/scripts/analytics"
mag_pipe = datapipe_prefix + '/data/mag_pipe.txt'
count_pipe = datapipe_prefix + '/data/count_pipe.txt'
angle_pipe = datapipe_prefix + '/data/angle_pipe.txt'
coord_pipe = datapipe_prefix + '/data/coord_pipe.txt'
plot_res = 15
mag_array = 0.25*np.ones((2*plot_res, plot_res))
count_array = np.ones((2*plot_res, plot_res))