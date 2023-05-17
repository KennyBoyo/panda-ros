import numpy as np
from sklearn.mixture import GaussianMixture
from analytics_vis import *

#https://github.com/sitzikbs/gmm_tutorial

with open(coord_pipe, "rb") as f:
    points = np.loadtxt(f)

# points = np.c_[(points[:, :3], np.sum(np.abs(points[:, 3:6])**2,axis=-1)**(1./2))]
#fit the gaussian model
gmm = GaussianMixture(n_components=6, covariance_type='diag', random_state=0)

gmm.fit(points)
cls = gmm.predict(points)
data = visualize_3d_gmm(points, gmm.weights_, gmm.means_[:, :].T, np.sqrt(gmm.covariances_[:, :]).T, cls)
fig = plot_3d_objects(data, 1)
fig.show()

