#!/usr/bin/env python3
import numpy as np
import plotly.graph_objects as go
from analytics_config import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cmx
import os

def rot_mat(angle, dir):
	"""
	Generates a rotation matrix which rotates about a specified direction

	Args:
		angle (float): angle in radians to e rotated
		dir (str): direction to rotate about (x, y or z)

	Returns:
		np.array: 3x3 matrix representing a rotation by an angle about the specified direction
	"""
	if dir == 'x':
		return np.array([[1, 0, 0], [0, np.cos(angle), -np.sin(angle)], [0, np.sin(angle), np.cos(angle)]])
	if dir == 'y':
		return np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
	if dir == 'z':
		return np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
	return

def parse_data(angles, effort, res=100):
	"""
	Function which parses a numpy array of angles (from openpose representing adduction, flexion, and torsion joint angles) and efforts 
	and returns it in the form required for the torque spheroid (binned values)

	Args:
		angles (np.array(np.array(float))): adduction, flexion, and torsion angles passed from openpose
		effort (np.array(float)): effort for each joint
		res (int, optional): resolution of binning. Defaults to 100.

	Returns:
		_type_: _description_
	"""
	for i in range(len(angles)):
		angles[i, :3] = cartesian2sphere(shoulder2cartesian(angles[i, :3]))
		
	theta_bins, theta_borders = gen_bins(-np.pi, np.pi, 2*res)
	phi_bins, phi_borders = gen_bins(0, np.pi, res)
	
	theta_indices = np.digitize(angles[:, 0], theta_borders)
	phi_indices = np.digitize(angles[:, 1], phi_borders)

	mags = np.sum(np.abs(effort[:3])**2,axis=-1)**(1./2)
	
	return np.stack((theta_indices, phi_indices), axis=1), mags

def gen_spherical(x, y, z, r, resolution=50):
	"""Return the coordinates for plotting a sphere centered at (x,y,z)"""
	u, v = np.mgrid[-np.pi:np.pi:resolution*2j, 0:np.pi:resolution*1j]
		
	X = r * np.cos(u)*np.sin(v) + x
	Y = r * np.sin(u)*np.sin(v) + y
	Z = r * np.cos(v) + z
	return (X, Y, Z)

def plot_3d_objects(data, axes_size=100, axes=True, uirevision=True, width=600, height=600, view="back"):
	if axes:
		data.append(go.Scatter3d(x = [0, axes_size], y = [0, 0], z = [0, 0], mode='lines', line = dict(color='red', width = 4), name="x", legendgroup="axes", showlegend=False))
		data.append(go.Scatter3d(x = [0, 0], y = [0, axes_size], z = [0, 0], mode='lines', line = dict(color='green', width = 4), name="y", legendgroup="axes", showlegend=False))
		data.append(go.Scatter3d(x = [0, 0], y = [0, 0], z = [0, axes_size], mode='lines', line = dict(color='blue', width = 4), name="z", legendgroup="axes", showlegend=False))

	if view == "back":
		eye = dict(
			x=-2.2,
			y=0.5,
			z=0,
		)
	if view == "side":
		eye = dict(
			x=0,
			y=0.5,
			z=-2.2,
		)
	if view == "top":
		eye = dict(
			x=-0.5,
			y=2,
			z=0,
		)
  
	# if view == "back":
	# 	eye = dict(
	# 		x=-2,
	# 		y=0.5,
	# 		z=0,
	# 	)
	# if view == "side":
	# 	eye = dict(
	# 		x=0,
	# 		y=0.5,
	# 		z=-2,
	# 	)
	# if view == "top":
	# 	eye = dict(
	# 		x=-0.5,
	# 		y=2,
	# 		z=0,
	# 	)

	fig = go.Figure(data=data)
	fig.update_layout(
		scene=dict(
			camera=dict(
				up=dict(
					x=0,
					y=1,
					z=0
				),
				eye=eye
			),
			aspectratio = dict( x=1, y=1, z=1 ),
			aspectmode = 'manual'
		),
	)
	fig.update_layout(uirevision=uirevision)
	fig.update_layout(width=width)
	fig.update_layout(height=height)
	fig.update_layout(legend=dict(
		font=dict(size=20)
	))
	
	return fig

def shoulder2cartesian(angles, base=np.array([0, -1, 0])  ):
	# Converts shoulder angles (Flexion, adduction, rotation) to cartesian coodinates TODO:MAKE SURE TO CHECK SHOULDER ANGLE ORDER
	return  rot_mat(angles[2], 'x') @ rot_mat(angles[0], 'z') @ base

def cartesian2sphere(coords):
	# Converts cartesian coodinates to spherical coordinates
	x = coords[0]
	y = coords[1]
	z = coords[2]
	xy = x**2 + y**2
	mag = np.sqrt(xy + z**2)
	theta = np.arctan2(y, x)
	phi = np.arctan2(np.sqrt(xy), z)

	return theta, phi, mag

def generate_data(n_points, seed=0):
	"""
	Generate mock data from a uniform distribution

	Args:
		n_points (int): number of points to be generated
		seed (int, optional): random seed for data. Defaults to 0.

	Returns:
		np.array: generated data
	"""
	return np.random.uniform(-np.pi, np.pi, (n_points, 3))

def generate_normal_data(n_points, seed=0):
	"""
	Generate mock data from a normal distribution

	Args:
		n_points (int): number of points to be generated
		seed (int, optional): random seed for data. Defaults to 0.

	Returns:
		np.array: generated data
	"""
	return np.random.normal(0, np.pi/3, (n_points, 3))

def gen_bins(low, high, res=100):
	"""
	Utility function to generate angle bins in torque spheroid

	Args:
		low (float): _description_
		high (_type_): _description_
		res (int, optional): _description_. Defaults to 100.

	Returns:
		_type_: _description_
	"""
	borders = np.linspace(low - np.abs(high - low)/(2*res), high + np.abs(high - low)/(2*res), res+2)
	bins = np.linspace(low, high, res)
	# Wrap low around to avoid indexing problem
	bins = np.append(bins, [low])
	return bins, borders

#https://github.com/sitzikbs/gmm_tutorial
def plot_sphere(w=0, c=[0,0,0], r=[1, 1, 1], subdev=10, ax=None, sigma_multiplier=3):
	'''
		plot a sphere surface
		Input: 
			c: 3 elements list, sphere center
			r: 3 element list, sphere original scale in each axis ( allowing to draw elipsoids)
			subdiv: scalar, number of subdivisions (subdivision^2 points sampled on the surface)
			ax: optional pyplot axis object to plot the sphere in.
			sigma_multiplier: sphere additional scale (choosing an std value when plotting gaussians)
		Output:
			ax: pyplot axis object
	'''

	if ax is None:
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')
	pi = np.pi
	cos = np.cos
	sin = np.sin
	phi, theta = np.mgrid[0.0:pi:complex(0,subdev), 0.0:2.0 * pi:complex(0,subdev)]
	x = sigma_multiplier*r[0] * sin(phi) * cos(theta) + c[0]
	y = sigma_multiplier*r[1] * sin(phi) * sin(theta) + c[1]
	z = sigma_multiplier*r[2] * cos(phi) + c[2]
	cmap = cmx.ScalarMappable()
	cmap.set_cmap('jet')
	c = cmap.to_rgba(w)

	ax.plot_surface(x, y, z, color=c, alpha=0.2, linewidth=1)

	return ax

def visualize_3d_gmm(points, w, mu, stdev, cls, export=True, ctype="all"):
	'''
	plots points and their corresponding gmm model in 3D
	Input: 
		points: N X 3, sampled points
		w: n_gaussians, gmm weights
		mu: 3 X n_gaussians, gmm means
		stdev: 3 X n_gaussians, gmm standard deviation (assuming diagonal covariance matrix)
	Output:
		None
	'''
	n_gaussians = mu.shape[1]
	colorset = cmx.Set1(np.linspace(0, 1, n_gaussians))
	colors = np.zeros((points.shape[0], colorset.shape[1]))
	for c in range(len(colors)):
		colors[c] = colorset[cls[c]]
	

	point_sets = []
	for i in range(n_gaussians):
		point_sets.append([])
	for i in range(len(points)):
		point_sets[cls[i]].append(points[i])
	for i in range(len(point_sets)):
		point_sets[i] = np.array(point_sets[i])
  
	
	print(mu.shape)
	data = []
	for i in range(n_gaussians):
		if ctype == "force":
			name = "Force: %.3f" % (np.linalg.norm(mu[3:6, i]))
		if ctype == "torque":
			name = "Shoulder: %.3f, Elbow: %.3f" % (np.linalg.norm(mu[3, i]), np.linalg.norm(mu[4, i]))
		if ctype == "all":
			name = "Force Mag: %.3f, Joint Torque Mag: %.3f" % (np.linalg.norm(mu[3:6, i]), np.linalg.norm(mu[6:, i]))
		data.append(go.Scatter3d(x=point_sets[i][:, 0], y=point_sets[i][:, 1], z=point_sets[i][:, 2],
								   mode='markers', marker=dict(
		size=5,
		opacity=0.8
		),
		name=name
	))
		
	return data