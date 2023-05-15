#!/usr/bin/env python3
import sys
import numpy as np
import plotly.graph_objects as go
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
from analytics_config import *
from analytics_utils import *

# ====================================================================================================================
# Webapp for displaying analytics
# ====================================================================================================================
app = dash.Dash(__name__)
app.layout = html.Div(
	html.Div([
		dcc.Graph(id='collection_completeness', style={'display': 'inline-block'}),
		dcc.Graph(id='live-graph', style={'display': 'inline-block'}),
		dcc.Graph(id='live-ws', style={'display': 'inline-block'}),
		dcc.Interval(
			id='timer-fast',
			interval=2*1000, # in milliseconds
			n_intervals=0
		),
		dcc.Interval(
			id='timer-slow',
			interval=10*1000, # in milliseconds
			n_intervals=0
		)
	])
)

# Multiple components can update everytime interval gets fired.
@app.callback(Output('live-graph', 'figure'),
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
	fig = plot_3d_objects(data, 1)
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
	fig = plot_3d_objects(data, 1)
	return fig

# Multiple components can update everytime interval gets fired.
@app.callback(Output('live-ws', 'figure'),
			Input('timer-slow', 'n_intervals'))
def update_graph_ws(n):
	with open(coord_pipe, 'rb') as f:	
		coord_array = np.loadtxt(f)
	
	data = []
	data.append(go.Mesh3d(x=coord_array[:, 0], y=coord_array[:, 1], z=coord_array[:, 2], alphahull=0.5, opacity=1))#, color='lightpink))
	fig = plot_3d_objects(data, 2) 

	return fig

# ====================================================================================================================
# Main
# ====================================================================================================================
def main(args):
	app.run_server(debug=True)

if __name__ == '__main__':
	main(sys.argv)