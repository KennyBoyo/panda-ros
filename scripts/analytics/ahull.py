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


def plot_gmm(n_components=7, ctype = "all"):
	with open(coord_pipe, "rb") as f:
		points = np.loadtxt(f)

	# Postion and force
	if ctype == "force":
		points = np.c_[(points[:, :3], np.sum(np.abs(points[:, 3:6])**2,axis=-1)**(1./2))]
	if ctype == "torques":
		points = np.c_[(points[:, :3], np.sum(np.abs(points[:, 6:])**2,axis=-1)**(1./2))]
	
	
	points[:, 0] = points[:, 0] - 0.5
	points[:, 1] = points[:, 1] - 0.5
	points[:, 2] = points[:, 2] + 0.5
 
 	#fit the gaussian model
	gmm = GaussianMixture(n_components=n_components, covariance_type='diag', random_state=0)
	gmm.fit(points)
	cls = gmm.predict(points)
	# print(points.shape)
	data = visualize_3d_gmm(points, gmm.weights_, gmm.means_[:, :].T, np.sqrt(gmm.covariances_[:, :]).T, cls)
	fig = plot_3d_objects(data, 1, width=800, height=700)
	fig.show()
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

def SelBest(arr:list, X:int)->list:
	'''
	returns the set of X configurations with shorter distance
	'''
	dx=np.argsort(arr)[:X]
	return arr[dx]


def get_silhouette(X, iterations = 20, n_cluster=10):
	n_clusters=np.arange(2, n_cluster)
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

	plt.figure()
	plt.errorbar(n_clusters, sils, yerr=sils_err)
	plt.title("Silhouette Scores", fontsize=20)
	plt.xticks(n_clusters)
	plt.xlabel("N. of clusters")
	plt.ylabel("Score")

def gmm_js(gmm_p, gmm_q, n_samples=10**5):
	X = gmm_p.sample(n_samples)[0]
	log_p_X = gmm_p.score_samples(X)
	log_q_X = gmm_q.score_samples(X)
	log_mix_X = np.logaddexp(log_p_X, log_q_X)

	Y = gmm_q.sample(n_samples)[0]
	log_p_Y = gmm_p.score_samples(Y)
	log_q_Y = gmm_q.score_samples(Y)
	log_mix_Y = np.logaddexp(log_p_Y, log_q_Y)

	return np.sqrt((log_p_X.mean() - (log_mix_X.mean() - np.log(2))
			+ log_q_Y.mean() - (log_mix_Y.mean() - np.log(2))) / 2)

def get_gmm_js(X, iterations = 20, n_cluster=10):
	n_clusters=np.arange(2, n_cluster)
	results=[]
	res_sigs=[]
	for n in tqdm(n_clusters):
		dist=[]
		for iteration in tqdm(range(iterations)):
			train, test=train_test_split(X, test_size=0.5)
			gmm_train=GaussianMixture(n, n_init=2).fit(train) 
			gmm_test=GaussianMixture(n, n_init=2).fit(test) 
			dist.append(gmm_js(gmm_train, gmm_test))
		selec=SelBest(np.array(dist), int(iterations/5))
		result=np.mean(selec)
		res_sig=np.std(selec)
		results.append(result)
		res_sigs.append(res_sig)

	plt.figure()
	plt.errorbar(n_clusters, results, yerr=res_sigs)
	plt.title("Distance between Train and Test GMMs", fontsize=20)
	plt.xticks(n_clusters)
	plt.xlabel("N. of clusters")
	plt.ylabel("Distance")
 
def get_BIC(X, iterations=20, n_cluster=10): 	
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
	
	print(np.log(np.abs(10*np.gradient(bics))))

	plt.figure()
	plt.errorbar(n_clusters, np.log(np.abs(10*np.gradient(bics))), label='BIC')
	plt.title("Gradient of gradient of BIC Scores", fontsize=20)
	plt.xticks(n_clusters)
	plt.xlabel("N. of clusters")
	plt.ylabel("grad(BIC)")
	plt.legend()
 
 
	dbics = np.gradient(bics)
	print(dbics.min)
	dbics_range = np.amax(dbics) - np.amin(dbics)
	print(dbics_range)
	thres = 0.75
	for i, val in enumerate(dbics):
		print(val)
		if val > (np.amin(dbics) + thres * dbics_range):
			print("n_clusters:", i+2)
			return i+2
	# print("n_clusters:", nc)
 
 

# data = plot_gmm("force")
with open(coord_pipe, "rb") as f:
	data = np.loadtxt(f)
data = np.c_[(data[:, :3], np.sum(np.abs(data[:, 3:6])**2,axis=-1)**(1./2))]

print(data[:4, :3])
data[:, 0] = data[:, 0] - 0.5
data[:, 1] = data[:, 1] - 0.5
data[:, 2] = data[:, 2] + 0.5
print(data[:4, :3])
# get_silhouette(data, iterations=20)
nc = get_BIC(data, iterations=10)
# get_gmm_js(data, iterations=20)
plt.show()
plot_gmm(n_components=nc, ctype="force")
# plot_gmm(n_components=5)
