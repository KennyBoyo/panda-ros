import numpy as np
from sklearn.mixture import GaussianMixture
from analytics_vis import *

#https://github.com/sitzikbs/gmm_tutorial

with open(coord_pipe, "rb") as f:
    points = np.loadtxt(f)

#fit the gaussian model
gmm = GaussianMixture(n_components=10, covariance_type='diag', random_state=0)

gmm.fit(points)
print(gmm.weights_.shape[0])
visualize_3d_gmm(points, gmm.weights_, gmm.means_[:, :].T, np.sqrt(gmm.covariances_[:, :]).T)


