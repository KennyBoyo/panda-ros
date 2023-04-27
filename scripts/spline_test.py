import scipy.interpolate as si
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 

degree = 5
cv = [[1,2,3], [14,10,3], [7,8,3]]
cv = np.asarray(cv)
count = cv.shape[0]

degree = np.clip(degree,1,count-1)
kv = np.clip(np.arange(count+degree+1)-degree,0,count-degree)

# Return samples
max_param = count - (degree )
spl = si.BSpline(kv, cv, degree)

spline = spl(np.linspace(0,max_param,1000))
print(spline.shape)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# data = np.random.random(size=(3, 3, 3))
# z, x, y = data.nonzero()
ax.scatter(spline[:, 0], spline[:, 1], spline[:, 2], c=spline[:, 2], alpha=1)
plt.show()