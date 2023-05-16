import numpy as np
from sklearn.mixture import GaussianMixture
from analytics_vis import *

#https://github.com/sitzikbs/gmm_tutorial

with open(coord_pipe, "rb") as f:
    points = np.loadtxt(f)
print((np.sum(np.abs(points[:, 3:6])**2,axis=-1)**(1./2)).shape)
print(points[:, :3].shape)

points = np.c_[(points[:, :3], np.sum(np.abs(points[:, 3:6])**2,axis=-1)**(1./2))]
print(points.shape)
#fit the gaussian model
gmm = GaussianMixture(n_components=6, covariance_type='diag', random_state=0)

gmm.fit(points)
cls = gmm.predict(points)
visualize_3d_gmm(points, gmm.weights_, gmm.means_[:, :].T, np.sqrt(gmm.covariances_[:, :]).T, cls)


