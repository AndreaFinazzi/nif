'''
@file   radar_plot.py
@auther USRG @ KAIST, Andrea Finazzi
@date   2021-12-24
@brief  Radar data plotter
'''
import numpy as np
from numpy.core.shape_base import block
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

N = 60
RANGE_MAX = 200
ANGLE_MAX = 20
tracks : List[EsrTrack] = []
tracks_np = np.zeros((N, 2))
tracks_colors = np.zeros((N, 3))

index : int = 0
tracks_filtered : List[EsrTrack] = []

fig = plt.figure(figsize=(7,7))
ax = plt.axes(xlim=(0,ANGLE_MAX),ylim=(0,RANGE_MAX))

scatter=ax.scatter(tracks_np[:,0], tracks_np[:,1], c=tracks_colors)

ax.set_xlim([-2, 200])
ax.set_ylim([-50, 50])
ax.set_xticks([-5, -10, -15, -20, -25, 0, 5, 10, 15, 20, 25], )
ax.set_yticks([-5, -10, -20, -50, 0, 5, 10, 20, 50]) 
ax.grid(True)

def update(frame_number):

    scatter.set_offsets(tracks_np)
    scatter.set_color(tracks_colors)

    return scatter,


def esr_track_callback(msg: EsrTrack):
    global index
    tracks.append(msg)
    tracks_np[index] = ( msg.track_range, msg.track_range_rate )

    colorcode = np.max( [min( [msg.track_range_rate, 40.0] ), -40.0] ) / 80.0 + 0.5
    tracks_colors[index][0] = 1.0 - colorcode
    tracks_colors[index][1] = colorcode
    tracks_colors[index][2] = 0.0
    
    index = index +1 if index < (N - 1) else 0

def esr_track_filtered_callback(msg: EsrTrack):
    tracks_filtered.append(msg)

def timer_callback():
    fig.canvas.flush_events()


anim = animation.FuncAnimation(fig, update, interval=10)
fig.show()

# ROS 
rclpy.init()
node = BaseNode('radar_plotter')

esr_track_sub = node.create_subscription(
    EsrTrack, '/radar_front/esr_track', esr_track_callback, rclpy.qos.qos_profile_sensor_data)
# esr_track_sub_filtered = node.create_subscription(
#     EsrTrack, '/radar_front/esr_track/filtered', esr_track_filtered_callback, rclpy.qos.qos_profile_sensor_data)

timer = node.create_timer(0.01, timer_callback)

rclpy.spin(node)
