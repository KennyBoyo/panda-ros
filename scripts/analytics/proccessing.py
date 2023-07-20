#!/usr/bin/env python3
import sys
import numpy as np
import plotly.graph_objects as go
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
from analytics_config import *
from analytics_utils import *
from sklearn.mixture import GaussianMixture

# ================================================================================================================
# Utility file used to further process and parse data for reporting
# ================================================================================================================

def plot_alphahull():
	"""
	Genetares the alpha hull for a set of data
	"""
	with open(angle_pipe, 'rb') as f:	
			angle_array = np.loadtxt(f)
		
	data = []
	data.append(go.Mesh3d(x=angle_array[:, 0], y=angle_array[:, 1], z=angle_array[:, 2], alphahull=-2, opacity=1))#, color='lightpink))
	fig = plot_3d_objects(data, 2)
	fig.show()

def plot_counts():
	"""
	Plots the count sphere for a set of data
	"""
	with open(count_pipe, 'rb') as f:
		count_array = np.loadtxt(f)

	data = []
	(x_pns_surface, y_pns_surface, z_pns_surface) = gen_spherical(0, 0, 0, 0.75*np.ones((2*plot_res, plot_res)), plot_res)
	data.append(go.Surface(x=x_pns_surface, y=y_pns_surface, z=z_pns_surface, opacity=1, surfacecolor=count_array, colorscale=[[0, "rgb(255, 0, 0)"],[1, "rgb(0, 255, 0)"]], cmin=0, cmax=10 ))
	fig = plot_3d_objects(data, 1, width=1000, height=1000)
	fig.show()


def plot_shoulder_torques(filename="None", view ="back"):
	"""
	Plots the shoulder torque spheroid from a specified file

	Args:
		filename (str, optional): path of file. Defaults to "None".
		view (str, optional): the view that the graph should be saved as. Defaults to "back".
	"""
	if filename == None:
		with open(mag_pipe, "rb") as f:
			mag_array = np.loadtxt(f)
	else:
		with open(f"c:/Users/Jun Khai/Documents/Uni/Year 4 Sem 2/METR4912/panda_ros/scripts/analytics/data/{filename}_mag_pipe.txt", "rb") as f:
			mag_array = np.loadtxt(f)
	
	data = []
	(x_pns_surface, y_pns_surface, z_pns_surface) = gen_spherical(0, 0, 0, mag_array, plot_res)
	data.append(go.Surface(x=x_pns_surface, y=y_pns_surface, z=z_pns_surface, opacity=1, surfacecolor=x_pns_surface**2 + y_pns_surface**2 + z_pns_surface**2))
	fig = plot_3d_objects(data, 1, width=1000, height=1000, view=view)
 	
	fig.update_layout(coloraxis_colorbar=dict(yanchor="top", y=1, x=0,
										  ticks="outside",
										  ticksuffix=" Nm"))
 
	fig.write_image(f"./thesis_plots/spheroid/{filename}_{view}.png")


def plot_gmm(n_components=7, ctype = "all", view="back", filename=None):
	"""
	Plots a gaussian mixture model with a set number of components from a data file

	Args:
		n_components (int, optional): number of components to be in the gaussian mixture model. Defaults to 7.
		ctype (str, optional): class type of the data, can be "force", "torque", or "all". Defaults to "all".
		view (str, optional): view that the graph should be saved in. Defaults to "back".
		filename (_type_, optional): path to the file to retrieve data from. Defaults to None.

	Returns:
		np.array: a set of points collected from the data 
	"""
	if filename == None:
		with open(coord_pipe, "rb") as f:
			points = np.loadtxt(f)
	else:
		with open(f"c:/Users/Jun Khai/Documents/Uni/Year 4 Sem 2/METR4912/panda_ros/scripts/analytics/data/{filename}_coord_pipe.txt", "rb") as f:
			points = np.loadtxt(f)

	# Postion and force
	if ctype == "force":
		points = np.c_[(points[:, :3], np.sum(np.abs(points[:, 3:6])**2,axis=-1)**(1./2))]
	if ctype == "torque":
		points = np.c_[np.c_[(points[:, :3], np.sum(np.abs(points[:, 6:9])**2,axis=-1)**(1./2))], points[:, 9]]
	
	# [0.2943, 0.2433, 0.2712]
	points[:, 0] = points[:, 0] - 0.2943*2
	points[:, 1] = points[:, 1] - 0.2433*2
	points[:, 2] = points[:, 2] + 0.2712*2
 
 	# Fit the gaussian model
	gmm = GaussianMixture(n_components=n_components, covariance_type='diag', random_state=0)
	gmm.fit(points)
	cls = gmm.predict(points)
	
	data = visualize_3d_gmm(points, gmm.weights_, gmm.means_[:, :].T, np.sqrt(gmm.covariances_[:, :]).T, cls, ctype=ctype)
	fig = plot_3d_objects(data, 1, width=800, height=800, view=view)
 
	if ctype == "force":
		fig.update_layout(legend=dict(
			yanchor="bottom",
			y=0.5,
			xanchor="left",
			x=0.8
		))
	
	else:
		fig.update_layout(legend_orientation="h")
		fig.update_layout(legend=dict(
			yanchor="bottom",
			y=-0.2,
			xanchor="left",
			x=0.14
		))
	
	
 
	if ctype == "force":
		fig.write_image(f"./thesis_plots/force/{filename}_{ctype}_{view}.png")
	elif ctype == "torque":
		fig.write_image(f"./thesis_plots/torque/{filename}_{ctype}_{view}.png")    
	elif ctype == "all":
		fig.write_image(f"./thesis_plots/all/{filename}_{ctype}_{view}.png")
	# fig.to_image(format="png")
	# fig.show()
 
 
	# fig.write_image(f"./{filename}_{ctype}_{view}.png")    
	return points


import numpy as np
import scipy.stats as st
from scipy.stats import multivariate_normal as MVN
import sklearn.metrics as metrics
from tqdm import tqdm
from multiprocessing import Process, Queue
from sklearn.model_selection import train_test_split

def distributions_js(distribution_p, distribution_q, n_samples=10 ** 5):
	# jensen shannon divergence. (Jensen shannon distance is the square root of the divergence)
	# all the logarithms are defined as log2 (because of information entrophy)
	X = distribution_p.rvs(n_samples)
	p_X = distribution_p.pdf(X)
	q_X = distribution_q.pdf(X)
	log_mix_X = np.log2(p_X + q_X)

	Y = distribution_q.rvs(n_samples)
	p_Y = distribution_p.pdf(Y)
	q_Y = distribution_q.pdf(Y)
	log_mix_Y = np.log2(p_Y + q_Y)

	return (np.log2(p_X).mean() - (log_mix_X.mean() - np.log2(2))
			+ np.log2(q_Y).mean() - (log_mix_Y.mean() - np.log2(2))) / 2

print("should be different:")
print(distributions_js(st.norm(loc=10000), st.norm(loc=0)))
print("should be same:")
print(distributions_js(st.norm(loc=0), st.norm(loc=0)))
 
def get_BIC(iterations=20, n_cluster=10, filename=None, ctype="all"): 	
	"""
	Get the Bayesian Information Criterion (BIC) score for the data retrieved

	Args:
		iterations (int, optional): Number of iterations to be used to determine BIC. Defaults to 20.
		n_cluster (int, optional): Number of Clusters to analyse up to. Defaults to 10.
		filename (_type_, optional): Path to file to retrieve data from. Defaults to None.
		ctype (str, optional):  class type of the data, can be "force", "torque", or "all". Defaults to "all".

	Returns:
		int: returns the optimal number of clusters to split the GMM into
	"""
	if filename == None:
		with open(coord_pipe, "rb") as f:
			X = np.loadtxt(f)
	else:
		with open(f"c:/Users/Jun Khai/Documents/Uni/Year 4 Sem 2/METR4912/panda_ros/scripts/analytics/data/{filename}_coord_pipe.txt", "rb") as f:
			X = np.loadtxt(f)
   
   
   	# Postion and force
	if ctype == "force":
		X = np.c_[(X[:, :3], np.sum(np.abs(X[:, 3:6])**2,axis=-1)**(1./2))]
	if ctype == "torque":
		X = np.c_[np.c_[(X[:, :3], np.sum(np.abs(X[:, 6:9])**2,axis=-1)**(1./2))], X[:, 9]]
   
	n_clusters=np.arange(2, n_cluster)
	bics=[]
	bics_err=[]
	for n in tqdm(n_clusters):
		tmp_bic=[]
		for _ in tqdm(range(iterations)):
			gmm=GaussianMixture(n, n_init=2).fit(X) 
			
			tmp_bic.append(gmm.bic(X))
		val=np.mean(SelBest(np.array(tmp_bic), int(iterations/5)))
		err=np.std(tmp_bic)
		bics.append(val)
		bics_err.append(err)

	plt.figure()
	plt.errorbar(n_clusters,bics, yerr=bics_err, label='BIC')
	plt.title("BIC Scores", fontsize=20)
	plt.xticks(n_clusters)
	plt.xlabel("N. of clusters")
	plt.ylabel("Score")
	plt.legend()

	plt.figure()
	plt.errorbar(n_clusters, np.gradient(bics), yerr=bics_err, label='BIC')
	plt.title("Gradient of BIC Scores", fontsize=20)
	plt.xticks(n_clusters)
	plt.xlabel("N. of clusters")
	plt.ylabel("grad(BIC)")
	plt.legend()
 
	dbics = np.gradient(bics)
	dbics_range = np.amax(dbics) - np.amin(dbics)
	thres = 0.5
	for i, val in enumerate(dbics):
		print(val)
		if val > (np.amin(dbics) + thres * dbics_range):
			print("n_clusters:", i+2)
			return i+2
 
namelist = ["unity1", "unity2", "unity_weak", "four_corners_random", "sides_random", "sides_horizontal", "sides_vertical", "center_random", "center_horizontal", "center_vertical", "full_random", "full_horizontal", "full_vertical"]

for i, filename in enumerate(namelist):
	nc = get_BIC(iterations=10, filename=filename, ctype="force")
	# plt.show()
	plot_gmm(n_components=nc, ctype="force", view="back", filename=filename)
	plot_gmm(n_components=nc, ctype="force", view="side", filename=filename)
	plot_gmm(n_components=nc, ctype="force", view="top", filename=filename)
	
	nc = get_BIC(iterations=10, filename=filename, ctype="torque")
	plot_gmm(n_components=nc, ctype="torque", view="back", filename=filename)
	plot_gmm(n_components=nc, ctype="torque", view="side", filename=filename)
	plot_gmm(n_components=nc, ctype="torque", view="top", filename=filename)
 
 
	nc = get_BIC(iterations=10, filename=filename, ctype="all")
	plot_gmm(n_components=nc, ctype="all", view="back", filename=filename)
	plot_gmm(n_components=nc, ctype="all", view="side", filename=filename)
	plot_gmm(n_components=nc, ctype="all", view="top", filename=filename)