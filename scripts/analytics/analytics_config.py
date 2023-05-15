import numpy as np

mag_pipe = 'data/mag_pipe.txt'
count_pipe = 'data/count_pipe.txt'
coord_pipe = 'data/coord_pipe.txt'
plot_res = 15
mag_array = 0.25*np.ones((2*plot_res, plot_res))
count_array = np.ones((2*plot_res, plot_res))