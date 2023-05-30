#!/usr/bin/env python3
import sys
import numpy as np
import plotly.graph_objects as go
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
from analytics_config import *
from analytics_utils import *

with open(angle_pipe, 'rb') as f:	
		angle_array = np.loadtxt(f)
	
data = []
data.append(go.Mesh3d(x=angle_array[:, 0], y=angle_array[:, 1], z=angle_array[:, 2], alphahull=-2, opacity=1))#, color='lightpink))
fig = plot_3d_objects(data, 2) 

fig.show()