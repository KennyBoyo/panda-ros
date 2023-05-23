import numpy as np
from datetime import datetime
now = datetime.now() # current date and time

date_time = now.strftime("%m_%d_%Y_%H_%M")
datapipe_prefix = "/home/medrobotics/ROS_Workspaces/stars_ws/src/panda_ros/scripts/analytics"
mag_pipe = datapipe_prefix + f'/data/mag_pipe_{date_time}.txt'
count_pipe = datapipe_prefix + f'/data/count_pipe_{date_time}.txt'
angle_pipe = datapipe_prefix + f'/data/angle_pipe_{date_time}.txt'
coord_pipe = datapipe_prefix + f'/data/coord_pipe_{date_time}.txt'
cubic_pipe = datapipe_prefix + f'/data/cubic_pipe_{date_time}.txt'
plot_res = 15
mag_array = 0.25*np.ones((2*plot_res, plot_res))
count_array = np.ones((2*plot_res, plot_res))