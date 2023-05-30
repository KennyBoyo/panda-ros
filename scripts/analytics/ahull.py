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

def plot_alphahull():
	with open(angle_pipe, 'rb') as f:	
			angle_array = np.loadtxt(f)
		
	data = []
	data.append(go.Mesh3d(x=angle_array[:, 0], y=angle_array[:, 1], z=angle_array[:, 2], alphahull=-2, opacity=1))#, color='lightpink))
	fig = plot_3d_objects(data, 2) 

	fig.show()

def plot_counts():
	with open(count_pipe, 'rb') as f:
		count_array = np.loadtxt(f)

	data = []
	(x_pns_surface, y_pns_surface, z_pns_surface) = gen_spherical(0, 0, 0, 0.75*np.ones((2*plot_res, plot_res)), plot_res)
	data.append(go.Surface(x=x_pns_surface, y=y_pns_surface, z=z_pns_surface, opacity=1, surfacecolor=count_array, colorscale=[[0, "rgb(255, 0, 0)"],[1, "rgb(0, 255, 0)"]], cmin=0, cmax=10 ))
	fig = plot_3d_objects(data, 1, width=1000, height=1000)
	fig.show()


def plot_shoulder_torques():
	with open(mag_pipe, 'rb') as f:
		mag_array = np.loadtxt(f)
	
	data = []
	(x_pns_surface, y_pns_surface, z_pns_surface) = gen_spherical(0, 0, 0, mag_array, plot_res)
	data.append(go.Surface(x=x_pns_surface, y=y_pns_surface, z=z_pns_surface, opacity=1, surfacecolor=x_pns_surface**2 + y_pns_surface**2 + z_pns_surface**2))
	# data.append(go.Surface(x=x_pns_surface, y=y_pns_surface, z=z_pns_surface, opacity=1, surfacecolor=count_array))
	fig = plot_3d_objects(data, 1, width=1000, height=1000)
	fig.show()


def plot_gmm(ctype = "all"):
	with open(coord_pipe, "rb") as f:
		points = np.loadtxt(f)

	# Postion and force
	if ctype == "force":
		points = np.c_[(points[:, :3], np.sum(np.abs(points[:, 3:6])**2,axis=-1)**(1./2))]
	if ctype == "torques":
		points = np.c_[(points[:, :3], np.sum(np.abs(points[:, 6:])**2,axis=-1)**(1./2))]
	#fit the gaussian model
	gmm = GaussianMixture(n_components=7, covariance_type='diag', random_state=0)
	gmm.fit(points)
	cls = gmm.predict(points)
	# print(points.shape)
	data = visualize_3d_gmm(points, gmm.weights_, gmm.means_[:, :].T, np.sqrt(gmm.covariances_[:, :]).T, cls)
	fig = plot_3d_objects(data, 2, width=2000, height=1000)
	# fig.show()
	return points


import numpy as np
import scipy.stats as st
from scipy.stats import multivariate_normal as MVN
import sklearn.metrics as metrics
from tqdm import tqdm
from multiprocessing import Process, Queue

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

# print("should be different:")
# print(distributions_js(st.norm(loc=10000), st.norm(loc=0)))
# print("should be same:")
# print(distributions_js(st.norm(loc=0), st.norm(loc=0)))

def SelBest(arr:list, X:int)->list:
	'''
	returns the set of X configurations with shorter distance
	'''
	dx=np.argsort(arr)[:X]
	return arr[dx]


def get_silhouette(X, iterations = 20):
	n_clusters=np.arange(2, 20)
	sils=[]
	sils_err=[]
	for n in tqdm(n_clusters):
		tmp_sil=[]
		for _ in tqdm(range(iterations)):
			gmm=GaussianMixture(n, n_init=2).fit(X) 
			labels=gmm.predict(X)
			sil=metrics.silhouette_score(X, labels, metric='euclidean')
			tmp_sil.append(sil)
		val=np.mean(SelBest(np.array(tmp_sil), int(iterations/5)))
		err=np.std(tmp_sil)
		sils.append(val)
		sils_err.append(err)
	plt.errorbar(n_clusters, sils, yerr=sils_err)
	plt.title("Silhouette Scores", fontsize=20)
	plt.xticks(n_clusters)
	plt.xlabel("N. of clusters")
	plt.ylabel("Score")
	plt.show()

# def get_silhouette(X, iterations = 20):
# 	n_clusters=np.arange(2, 20)
# 	sils=[]
# 	sils_err=[]
# 	procs = []
# 	for n in tqdm(n_clusters):
# 		proc = Process(target=process_silhoutte_clusterset, args=(X, n, iterations))
# 		procs.append(proc)
# 		proc.start()
# 	for proc in procs:
# 		proc.join()

# 	while not sil_val_queue.empty():
# 		sils.append(sil_val_queue.get()) 
# 	while not sil_err_queue.empty():
# 		sils_err.append(sil_err_queue.get()) 

# 	plt.errorbar(n_clusters, sils, yerr=sils_err)
# 	plt.title("Silhouette Scores", fontsize=20)
# 	plt.xticks(n_clusters)
# 	plt.xlabel("N. of clusters")
# 	plt.ylabel("Score")
# 	plt.show()

# def process_silhoutte_clusterset(X, n, iterations):
# 	tmp_sil=[]
# 	for _ in tqdm(range(iterations)):
# 		gmm=GaussianMixture(n, n_init=2).fit(X) 
# 		labels=gmm.predict(X)
# 		sil=metrics.silhouette_score(X, labels, metric='euclidean')
# 		tmp_sil.append(sil)
# 	val=np.mean(SelBest(np.array(tmp_sil), int(iterations/5)))
# 	err=np.std(tmp_sil)
# 	return val, err


# sil_val_queue = Queue()
# sil_err_queue = Queue()

# def get_BIC(): 	
# data = plot_gmm("force")
with open(coord_pipe, "rb") as f:
	data = np.loadtxt(f)
get_silhouette(data)
plt.show()
# plot_gmm("force")



# names = ['America', 'Europe', 'Africa']
# 	procs = []
# 	proc = Process(target=print_func)  # instantiating without any argument
# 	procs.append(proc)
# 	proc.start()

# 	# instantiating process with arguments
# 	for name in names:
# 		# print(name)
# 		proc = Process(target=print_func, args=(name,))
# 		procs.append(proc)
# 		proc.start()

# 	# complete the processes
# 	for proc in procs:
# 		proc.join()