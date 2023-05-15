#!/usr/bin/env python3
import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import plotly.express as px
import plotly.graph_objects as go
import multiprocessing
from multiprocessing import shared_memory
from sensor_msgs.msg import JointState
import dash
from dash import dcc, html
from dash.dependencies import Input, Output

def rot_mat(angle, dir):
	if dir == 'x':
		return np.array([[1, 0, 0], [0, np.cos(angle), -np.sin(angle)], [0, np.sin(angle), np.cos(angle)]])
	if dir == 'y':
		return np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
	if dir == 'z':
		return np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
	return

def parse_data(angles, effort, res=100):
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

def plot_3d_objects(data, axes_size=100, axes=True):
	if axes:
		data.append(go.Scatter3d(x = [0, axes_size], y = [0, 0], z = [0, 0], mode='lines', line = dict(color='red', width = 4)))
		data.append(go.Scatter3d(x = [0, 0], y = [0, axes_size], z = [0, 0], mode='lines', line = dict(color='green', width = 4)))
		data.append(go.Scatter3d(x = [0, 0], y = [0, 0], z = [0, axes_size], mode='lines', line = dict(color='blue', width = 4)))
	
	fig = go.Figure(data=data)
	fig.update_layout(
		scene=dict(
			camera=dict(
				up=dict(
					x=0,
					y=1,
					z=0
				),
				eye=dict(
					x=0,
					y=1.0707,
					z=1,
				)
			),
			aspectratio = dict( x=1, y=1, z=1 ),
			aspectmode = 'manual'
		),
	)
	return fig

def shoulder2cartesian(angles, base=np.array([0, -1, 0])  ):
	# Converts shoulder angles (Flexion, adduction, rotation) to cartesian coodinates TODO:MAKE SURE TO CHECK SHOULDER ANGLE ORDER
	return rot_mat(angles[0], 'z') @ rot_mat(angles[2], 'x') @  base

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

# def cartesian2sphere(coords):
# 	# Converts cartesian coodinates to spherical coordinates
# 	print(coords)
# 	x = coords[0]
# 	y = coords[1]
# 	z = coords[2]
# 	xz = x**2 + z**2
# 	mag = np.sqrt(xz + y**2)
# 	theta = np.arctan2(z, x)
# 	phi = np.arctan2(np.sqrt(xz), y)
# 	print("tp:", theta, phi)

# 	return theta, phi, mag

def generate_data(n_points, seed=0):
	return np.random.uniform(-np.pi, np.pi, (n_points, 3))

def generate_normal_data(n_points, seed=0):
	return np.random.normal(0, np.pi/3, (n_points, 3))

def gen_bins(low, high, res=100):
	borders = np.linspace(low - np.abs(high - low)/(2*res), high + np.abs(high - low)/(2*res), res+2)
	bins = np.linspace(low, high, res)
	# Wrap low around to avoid indexing problem
	bins = np.append(bins, [low])
	return bins, borders

# res = 15
# angles = generate_data(10000)
# effort = generate_data(10000)
# theta_bins, theta_borders = gen_bins(-np.pi, np.pi, 2*res)
# phi_bins, phi_borders = gen_bins(0, np.pi, res)

# indices, mags = parse_data(angles, effort, res)
# r = np.random.uniform(0.25, 0.25, (2*res, res))

# for i in range(len(indices)):
# 	r[indices[i][0]-2, indices[i][1]-2] += mags[i]

# data = []
# (x_pns_surface, y_pns_surface, z_pns_surface) = gen_spherical(0, 0, 0, r, res)
# data.append(go.Surface(x=x_pns_surface, y=y_pns_surface, z=z_pns_surface, opacity=1, surfacecolor=x_pns_surface**2 + y_pns_surface**2 + z_pns_surface**2))
# global_figure = plot_3d_objects(data, 75)


# ====================================================================================================================
# Constants
# ====================================================================================================================
lock = multiprocessing.Lock()
shm_name = 'random1'
plot_res = 15
mag_array = 0.25*np.ones((2*plot_res, plot_res))
count_array = np.ones((2*plot_res, plot_res))

# ====================================================================================================================
# Threading
# ====================================================================================================================
def create_shared_memory_nparray():
	NP_DATA_TYPE = np.float64
	d_size = np.dtype(NP_DATA_TYPE).itemsize * 2 * (plot_res**2)

	shm = shared_memory.SharedMemory(create=True, size=d_size, name=shm_name)
	# numpy array on shared memory buffer
	dst = np.ndarray(shape=(2*plot_res, plot_res), dtype=NP_DATA_TYPE, buffer=shm.buf)
	dst[:] = 0.25*np.ones((2*plot_res, plot_res))
	# print(f'NP SIZE: {(dst.nbytes / 1024) / 1024}')
	return shm

def release_shared(name):
	shm = shared_memory.SharedMemory(name=name)
	shm.close()
	shm.unlink()  # Free and release the shared memory block


# ====================================================================================================================
# Ros Subscriber
# ====================================================================================================================
class AnalyticsNode:
	def __init__(self):
		self.sub = rospy.Subscriber("/os3/step_problem_solution", JointState, self.callback)
		
	def callback(self, js: JointState):
		res = 15
		# mag_array = np.append(an)
		# angles = generate_data(10000)
		# effort = generate_data(10000)

		angles = np.array([js.position]) 
		effort = np.array([js.effort])

		indices, mags = parse_data(angles, effort, res)

		for i in range(len(indices)):
			curr_count = count_array[indices[i][0]-2, indices[i][1]-2]
			curr_mag = mag_array[indices[i][0]-2, indices[i][1]-2]
			mag_array[indices[i][0]-2, indices[i][1]-2] = (curr_mag * curr_count + mags[i]) / (curr_count + 1)
			count_array[indices[i][0]-2, indices[i][1]-2] += 1


		

		# existing_shm = shared_memory.SharedMemory(name=shm_name)
		# lock.acquire()
		# np_array = np.ndarray((2*plot_res, plot_res,), dtype=np.int64, buffer=existing_shm.buf)
		# for i in range(len(indices)):
		# 	np_array[indices[i][0]-2, indices[i][1]-2] = mags[i]
		# lock.release()



# ====================================================================================================================
# Webapp for displaying analytics
# ====================================================================================================================
app = dash.Dash(__name__)
app.layout = html.Div(
	html.Div([
		dcc.Graph(id='live-graph'),
		dcc.Interval(
			id='timer',
			interval=1000, # in milliseconds
			n_intervals=0
		)
	])
)

# Multiple components can update everytime interval gets fired.
@app.callback(Output('live-graph', 'figure'),
			Input('timer', 'n_intervals'))
def update_graph_live(n):
	# existing_shm = shared_memory.SharedMemory(name=shm_name)
	
	data = []
	(x_pns_surface, y_pns_surface, z_pns_surface) = gen_spherical(0, 0, 0, mag_array, plot_res)
	data.append(go.Surface(x=x_pns_surface, y=y_pns_surface, z=z_pns_surface, opacity=1, surfacecolor=count_array))
	fig = plot_3d_objects(data, 1)
	fig.update_layout(uirevision=True)

	return fig


def start_ros():
	rospy.init_node("analytics_node", anonymous=True)
	an = AnalyticsNode()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

# ====================================================================================================================
# Main
# ====================================================================================================================
def main(args):
	# print(multiprocessing.current_process().name)
	# if multiprocessing.current_process().name == "MainProcess":
	# 		print("HIIHIHIHI")
	# shm = create_shared_memory_nparray()
	rospy.init_node("analytics_node", anonymous=True)
	an = AnalyticsNode()
	
	try:
		app.run_server(debug=True)
	except KeyboardInterrupt:
		print("Shutting down")
	# try:
	# 	rospy.spin()
	# except KeyboardInterrupt:
	# 	print("Shutting down")


if __name__ == '__main__':
	main(sys.argv)