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

# ====================================================================================================================
# Webapp for displaying analytics
# ====================================================================================================================
app = dash.Dash(__name__)
app.layout = html.Div(
	html.Div([
		dcc.Graph(id='collection_completeness', style={'display': 'inline-block'}),
		dcc.Graph(id='shoulder_torques', style={'display': 'inline-block'}),
		dcc.Graph(id='gaussian_mixture'),
		dcc.Graph(id='live-ws', style={'display': 'inline-block'}),
		dcc.Interval(
			id='timer-fast',
			interval=2*1000, # in milliseconds
			n_intervals=0
		),
		dcc.Interval(
			id='timer-slow',
			interval=5*1000, # in milliseconds
			n_intervals=0
		)
	]),
)

# Multiple components can update everytime interval gets fired.
@app.callback(Output('shoulder_torques', 'figure'),
			Input('timer-fast', 'n_intervals'))
def update_graph_live(n):
	with open(mag_pipe, 'rb') as f:
		mag_array = np.loadtxt(f)

	with open(count_pipe, 'rb') as f:
		count_array = np.loadtxt(f)

	
	data = []
	(x_pns_surface, y_pns_surface, z_pns_surface) = gen_spherical(0, 0, 0, mag_array, plot_res)
	data.append(go.Surface(x=x_pns_surface, y=y_pns_surface, z=z_pns_surface, opacity=1, surfacecolor=x_pns_surface**2 + y_pns_surface**2 + z_pns_surface**2))
	# data.append(go.Surface(x=x_pns_surface, y=y_pns_surface, z=z_pns_surface, opacity=1, surfacecolor=count_array))
	fig = plot_3d_objects(data, 1, width=1000, height=1000)
	return fig

# Multiple components can update everytime interval gets fired.
@app.callback(Output('collection_completeness', 'figure'),
			Input('timer-fast', 'n_intervals'))
def update_graph_live(n):
	with open(count_pipe, 'rb') as f:
		count_array = np.loadtxt(f)

	data = []
	(x_pns_surface, y_pns_surface, z_pns_surface) = gen_spherical(0, 0, 0, 0.75*np.ones((2*plot_res, plot_res)), plot_res)
	data.append(go.Surface(x=x_pns_surface, y=y_pns_surface, z=z_pns_surface, opacity=1, surfacecolor=count_array, colorscale=[[0, "rgb(255, 0, 0)"],[1, "rgb(0, 255, 0)"]], cmin=0, cmax=10 ))
	fig = plot_3d_objects(data, 1, width=1000, height=1000)
	return fig

# Multiple components can update everytime interval gets fired.
@app.callback(Output('gaussian_mixture', 'figure'),
			Input('timer-fast', 'n_intervals'))
def update_graph_live(n):
	with open(coord_pipe, "rb") as f:
		points = np.loadtxt(f)

	# Postion and force
	# points = np.c_[(points[:, :3], np.sum(np.abs(points[:, 3:6])**2,axis=-1)**(1./2))]
	#fit the gaussian model
	gmm = GaussianMixture(n_components=7, covariance_type='diag', random_state=0)
	gmm.fit(points)
	cls = gmm.predict(points)
	
	data = visualize_3d_gmm(points, gmm.weights_, gmm.means_[:, :].T, np.sqrt(gmm.covariances_[:, :]).T, cls)
	fig = plot_3d_objects(data, 2, width=1500, height=1500)
	return fig

# Multiple components can update everytime interval gets fired. Alphahull is done on a slow timer because it takes a long time to generate.
# @app.callback(Output('live-ws', 'figure'),
# 			Input('timer-slow', 'n_intervals'))
# def update_graph_ws(n):
# 	with open(angle_pipe, 'rb') as f:	
# 		angle_array = np.loadtxt(f)
	
# 	data = []
# 	data.append(go.Mesh3d(x=angle_array[:, 0], y=angle_array[:, 1], z=angle_array[:, 2], alphahull=0.1, opacity=1))#, color='lightpink))
# 	fig = plot_3d_objects(data, 2) 

# 	return fig

# ====================================================================================================================
# Main
# ====================================================================================================================
def main(args):
	app.run_server(debug=True)

if __name__ == '__main__':
	main(sys.argv)