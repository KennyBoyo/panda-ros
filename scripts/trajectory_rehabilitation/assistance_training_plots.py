import pandas as pd
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 
from nav_msgs.msg import Path
import bagpy
import numpy as np 
import os 
import argparse
import pathlib
saveFolderPath = ""



titles = ["Recorded_trajectory", "B_Spline_trajectory", "Repeated_B_spline_trajectory"]
colors = ["g", "b", "k"]


def render_plots(files):
    for file, title in zip(files, titles):
        print(title)
        bag = bagpy.bagreader(f"{file}")
        csvfiles = []

        for t in bag.topics:
            data = bag.message_by_topic(t)
            csvfiles.append(data)

        # remove bag 
        read = pd.read_csv(f'{file[:-4]}/ssistive_controller-status.csv')
        groups = read.groupby('assistance_mode')
        fig = plt.figure(figsize=(11,8))

        total_handles = []
        total_labels = []
        for idx, ((group, data), color) in enumerate(zip(groups, colors)):
            if group == "TUNNEL_ASSIST":
                group = "CHANNEL_ASSIST"
                
            path = np.load(os.path.join(pathlib.Path(file).parent, "path_data", f"{data['trajectory_id'].iloc[0]}.npy"))
            ax = fig.add_subplot(2,groups.ngroups,idx+1, projection='3d')

            ax.plot(path[:,0], path[:,1], path[:,2], alpha=1, linewidth =2, color = 'r', label = "Desired Trajectory")
            ax.set_title(group)
            ax.set_xlabel("x(m)")
            ax.set_ylabel("y(m)")
            ax.set_zlabel("z(m)")
            ax.plot(data['p_current.x'], data['p_current.y'], data['p_current.z'], color = color, alpha=1, label = group)
            handles, labels = ax.get_legend_handles_labels()

            if idx == 2:
                total_handles += handles
                total_labels += labels
            else:
                total_handles.append(handles[1])
                total_labels.append(labels[1])

        fig.legend(total_handles, total_labels, loc='upper left', ncol=2)
        for idx, ((group, data), color) in enumerate(zip(groups, colors)):
            if group == "TUNNEL_ASSIST":
                group = "CHANNEL_ASSIST"
            q_low = data["d_nearest"].quantile(0.01)
            q_hi  = data["d_nearest"].quantile(0.95)
            data["t"] = data["header.stamp.secs"] + data["header.stamp.nsecs"]*1e-9
            data["t"] -= data["t"].iloc[0]
            df_filtered = data[(data["d_nearest"] < q_hi) & (data["d_nearest"] > q_low)]
            print(group)
            print(df_filtered["d_nearest"].mean())
            if idx !=0:
                ax = fig.add_subplot(2,groups.ngroups,idx+4, sharey = ax)
            else:
                ax = fig.add_subplot(2,groups.ngroups,idx+4)
                ax.set_ylabel("Trajectory deviation (m)")

            ax.set_title(group)
            ax.set_xlabel("Time, seconds")
            ax.plot(df_filtered["t"], df_filtered['d_nearest'], color = color)

        fig.suptitle(title, fontsize = 15, horizontalalignment="left")
        plt.tight_layout(pad=0.2, w_pad=1, h_pad=0.9, rect=[0, 0, 0.9, 1])
        # plt.savefig(os.path.join(saveFolderPath, f"{title}.png"))
    
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('input_files', nargs='+', type=argparse.FileType('r'))
    args = parser.parse_args()

    input_files = [f.name for f in args.input_files]
    print([pathlib.Path(f).parent for f in input_files])

    render_plots(files=input_files)
