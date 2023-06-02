import numpy as np
from datetime import datetime
now = datetime.now() # current date and time

date_time = now.strftime("%m_%d_%Y_%H_%M")
# date_time = "unity1"
# date_time = "unity2"
# date_time = "unity_weak"
# date_time = "four_corners_random"

# date_time = "sides_random"
# date_time = "sides_horizontal"
# date_time = "sides_vertical"

# date_time = "center_random"
# date_time = "center_horizontal"
# date_time = "center_vertical"

# date_time = "full_random"
date_time = "full_horizontal"
# date_time = "full_vertical"
# datapipe_prefix = f"/home/medrobotics/ROS_Workspaces/stars_ws/src/panda_ros/scripts/analytics/data/{date_time}_"
# datapipe_prefix = f"/home/kenzo/Documents/catkin_ws/src/panda_ros/scripts/analytics/data/{date_time}_"
datapipe_prefix = f"c:/Users/Jun Khai/Documents/Uni/Year 4 Sem 2/METR4912/panda_ros/scripts/analytics/data/{date_time}_"
# date_time = "05_25_2023_15_29"
# date_time = "05_25_2023_18_41"
mag_pipe = datapipe_prefix + 'mag_pipe.txt'
count_pipe = datapipe_prefix + 'count_pipe.txt'
angle_pipe = datapipe_prefix + 'angle_pipe.txt'
coord_pipe = datapipe_prefix + 'coord_pipe.txt'
cubic_coord_pipe = datapipe_prefix + 'cubic_coord_pipe.txt'
cubic_value_pipe = datapipe_prefix + 'cubic_value_pipe.txt'
plot_res = 15
mag_array = 0.25*np.ones((2*plot_res, plot_res))
count_array = np.ones((2*plot_res, plot_res))