from typing import Dict, List, Any, Tuple
import argparse

import math
import numpy as np
import matplotlib.pyplot as plt

import utils
import visualization as viz

"""
ros2 bag record -o "bag/rosbag_$(date +%Y%m%d_%H%M%S)" /odometry/filtered/global /odometry/filtered/local /ublox_gps_node/fix

python utils/plot_localization.py <rosbag path> [options]

python amiga-ros2-nav/utils/plot_localization.py  bag/{bag} -o output/{filename}.png -p 1
"""


# -- Main

def plot_all(bag: Dict[str, List[Any]], out_file: str, plot_every: int = 1):
    fig, ax = plt.subplots()
    viz.plot_odometry_path(ax, bag["/odometry/filtered/local"], "local EKF", plot_every)
    viz.plot_odometry_path(ax, bag["/odometry/filtered/global"], "global EKF", plot_every)
    viz.plot_utm_path(ax, bag['/ublox_gps_node/fix'], "UTM", plot_every)

    # -- Dedup legend
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys())

    ax.grid()
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    plt.savefig(out_file)

def main():
    parser = argparse.ArgumentParser(
                prog='EKF Plotter',
                description='Plots the path taken from the EKF and GPS.'
            )
    parser.add_argument('bagfile_path')           
    parser.add_argument('-o', '--out_path', type=str, default="out.png")
    parser.add_argument('-p', '--plot_every', type=int, default=1)
    args = parser.parse_args()
    
    # -- Parse ROSBAG
    topics = [
        '/odometry/filtered/global',
        '/odometry/filtered/local',
        '/ublox_gps_node/fix',
    ]
    bag = utils.parse_bag(args.bagfile_path, topics)
    
    # -- Plot Local EKF, Global EKF, and GPS
    plot_all(bag, args.out_path, args.plot_every)
    
         
if __name__ == "__main__":
    main()