import scipy.interpolate as si
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 
import scipy.interpolate as scipolate

bottom = [0.3, 0, 0.3]
straight_top = [0.35, -0.1, 0.7]
accross = [0.6, 0.35, 0.25]
def spline_trajectory_generator(cv, degree, n):
	cv = np.asarray(cv)
	count = cv.shape[0]

	degree = np.clip(degree, 1, count - 1)
	kv = np.clip(np.arange(count + degree + 1) - degree, 0, count - degree)

	# Return samples
	max_param = count - degree
	spl = scipolate.BSpline(kv, cv, degree)

	spline_trajectory = spl(np.linspace(0,max_param,n))
	
	return spline_trajectory



spline = spline_trajectory_generator([bottom, straight_top, accross], 4, 60)
spline2 = spline_trajectory_generator([bottom, straight_top, accross], 7, 60)
fig = plt.figure()
ax1 = fig.add_subplot(121, projection='3d')
ax2 = fig.add_subplot(122, projection='3d')

# data = np.random.random(size=(3, 3, 3))
# z, x, y = data.nonzero()
ax1.scatter(spline[:, 0], spline[:, 1], spline[:, 2], c=spline[:, 2], alpha=1)
ax2.scatter(spline2[:, 0], spline2[:, 1], spline2[:, 2], c=spline2[:, 2], alpha=1)
plt.show()