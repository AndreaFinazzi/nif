'''
@file   radar_plot.py
@auther USRG @ KAIST, Andrea Finazzi
@date   2021-12-24
@brief  Radar data plotter
'''
import numpy as np
import rclpy
import csv
import pandas as pd

import matplotlib.pyplot as plt
import matplotlib.animation as animation

from delphi_esr_msgs.msg import EsrTrack
from typing import List, Set, Dict, Tuple, Optional

import math
import tf2_py
import tf2_ros
from tf2_ros.transform_broadcaster import TransformBroadcaster

from pymap3d.ecef import ecef2geodetic
from pymap3d.ned import ned2geodetic, geodetic2ned, geodetic2enu

from nifpy_common_nodes.base_node import BaseNode
from nif_lgsvl_simulation.quaternion_euler import Quaternion_Euler

# plt.ion()

N = 60
tracks : List[EsrTrack] = []
tracks_angles = np.zeros(N)
tracks_ranges = np.zeros(N)
tracks_colors = np.zeros(N)

index : int = 0
tracks_filtered : List[EsrTrack] = []

def esr_track_callback(msg: EsrTrack):
    global index
    tracks.append(msg)
    tracks_angles[index] = msg.track_angle
    tracks_ranges[index] = msg.track_range
    tracks_colors[index] =  (N - msg.track_id) * 2 / N 

    index = index +1 if index < (N - 1) else 0

def esr_track_filtered_callback(msg: EsrTrack):
    tracks_filtered.append(msg)

def flat_init():
    scatter_flat.set_offsets([])
    return scatter_flat,

def flat_update(frame, *fargs):
    # plt.figure(tracks_fig.number)
    # scatter_flat  = tracks_flat_ax.scatter(tracks_angles, tracks_ranges, c=tracks_colors)
    data = np.hstack((tracks_angles[:frame,np.newaxis], tracks_ranges[:frame, np.newaxis]))
    scatter_flat.set_offsets(data)

    tracks_flat_fig.canvas.draw()
    tracks_flat_fig.canvas.flush_events()

    return scatter_flat,



def timer_callback():
    tracks_fig.canvas.draw()
    tracks_flat_fig.canvas.draw()

    tracks_fig.canvas.flush_events()
    tracks_flat_fig.canvas.flush_events()



tracks_fig, tracks_ax = plt.subplots(subplot_kw={'projection': 'polar'})
# tracks_fig.add_subplot(tracks_ax)
tracks_ax.set_rmax(200)
tracks_ax.set_rticks([5, 10, 20, 50, 100, 200])  # Less radial ticks
tracks_ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
tracks_ax.grid(True)

tracks_flat_fig, tracks_flat_ax = plt.subplots()
tracks_flat_ax.set_xlim([-30, 30])
tracks_flat_ax.set_ylim([-2, 200])
tracks_flat_ax.set_xticks([-5, -10, -15, -20, -25, 0, 5, 10, 15, 20, 25], )
tracks_flat_ax.set_yticks([5, 10, 20, 50, 100, 200]) 
tracks_flat_ax.grid(True)

tracks_ax.set_title("A line plt on a polar axis", va='bottom')

# tracks_filtered_fig = plt.figure()
# tracks_filtered_ax  = plt.axes(plt.subplot(111), polar=True)
# tracks_filtered_fig.add_subplot(tracks_filtered_ax)
# 
scatter  = tracks_ax.scatter(tracks_angles, tracks_ranges, c=tracks_colors)
scatter_flat  = tracks_flat_ax.scatter(tracks_angles, tracks_ranges, c=tracks_colors)

# tracks_filtered_fig.show()
# ax.axis["left"]
# ax.axis["bottom"]

anim_flat = animation.FuncAnimation(
    tracks_flat_fig, 
    flat_update, 
    interval=10)
    # frames=len(tracks_ranges + 1),
    # blit=False, 
    # init_func=flat_init,
    # repeat=True)

tracks_fig.show()
tracks_flat_fig.show()

def main(args=None):

    rclpy.init(args=args)
    node = BaseNode('radar_plotter')

    esr_track_sub = node.create_subscription(
        EsrTrack, '/radar_front/esr_track/filtered', esr_track_callback, rclpy.qos.qos_profile_sensor_data)
    esr_track_sub_filtered = node.create_subscription(
        EsrTrack, '/radar_front/esr_track/filtered', esr_track_filtered_callback, rclpy.qos.qos_profile_sensor_data)

    timer = node.create_timer(0.01, timer_callback)

    rclpy.spin(node)

if __name__ == '__main__':
    main()