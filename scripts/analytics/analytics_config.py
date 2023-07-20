import numpy as np
from datetime import datetime
now = datetime.now()

date_time = now.strftime("%m_%d_%Y_%H_%M")
datapipe_prefix = f"/home/medrobotics/ROS_Workspaces/stars_ws/src/panda_ros/scripts/analytics/data/{date_time}_"

mag_pipe = datapipe_prefix + 'mag_pipe.txt'
count_pipe = datapipe_prefix + 'count_pipe.txt'
angle_pipe = datapipe_prefix + 'angle_pipe.txt'
coord_pipe = datapipe_prefix + 'coord_pipe.txt'
cubic_coord_pipe = datapipe_prefix + 'cubic_coord_pipe.txt'
cubic_value_pipe = datapipe_prefix + 'cubic_value_pipe.txt'
plot_res = 15
mag_array = 0.25*np.ones((2*plot_res, plot_res))
count_array = np.ones((2*plot_res, plot_res))