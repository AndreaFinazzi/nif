import sys
from numpy.lib.polynomial import RankWarning
import pymap3d as pm
import numpy as np
import matplotlib.pyplot as plt
import sys, os

import csv
import pickle
import sys
import logging
import graph_ltpl

ax = plt.subplot(1,1,1)

class WPTFileVisualizer:
    def __init__(self, file_path, legend):

        self.wpt_x, self.wpt_y, self.wpt_yaw_rad = self.readTrackCenterFile(file_path)
        self.visualization(legend)


    def readTrackCenterFile(self, file_path_):
        path_x_list_ = []
        path_y_list_ = []
        path_yaw_list_ = []

        cnt = 0
        fileHandle = open(file_path_, "r")
        for line in fileHandle:
            cnt += 1
            fields = line.split(",")
            if fields[0] != " " and fields[1] != " ":
                x = float(fields[0])
                y = float(fields[1])
                yaw = float(fields[1])
                path_x_list_.append(x)
                path_y_list_.append(y)
                path_yaw_list_.append(yaw)
        fileHandle.close()

        # Check loaded data
        assert len(path_x_list_) == len(
            path_y_list_
        ), "path x and y length are different"
        assert len(path_x_list_) != 0, "path file empty"
        assert len(path_yaw_list_) != 0, "path file empty"

        return path_x_list_, path_y_list_, path_yaw_list_

    def visualization(self, legend):
        plt.plot(self.wpt_x, self.wpt_y, label = legend)
        plt.legend(loc="upper left")
        plt.axis('equal')
        plt.grid()

class RaceLineFileVisualizer:
    def __init__(self, file_path, legend):

        self.wpt_x, self.wpt_y, self.wpt_yaw_rad = self.readTrackCenterFile(file_path)
        self.visualization(legend)


    def readTrackCenterFile(self, file_path_):
        path_x_list_ = []
        path_y_list_ = []
        path_yaw_list_ = []

        cnt = 0
        fileHandle = open(file_path_, "r")
        for line in fileHandle:
            cnt += 1
            if(cnt > 3):
                fields = line.split(";")
                if fields[0] != " " and fields[1] != " ":
                    x = float(fields[1])
                    y = float(fields[2])
                    yaw = float(fields[3])
                    path_x_list_.append(x)
                    path_y_list_.append(y)
                    path_yaw_list_.append(yaw)
        fileHandle.close()

        # Check loaded data
        assert len(path_x_list_) == len(
            path_y_list_
        ), "path x and y length are different"
        assert len(path_x_list_) != 0, "path file empty"
        assert len(path_yaw_list_) != 0, "path file empty"

        return path_x_list_, path_y_list_, path_yaw_list_

    def visualization(self, legend):
        plt.plot(self.wpt_x, self.wpt_y, label = legend)
        plt.legend(loc="upper left")
        plt.axis('equal')
        plt.grid()

class GraphVisualizer:
    def __init__(self, file_path, legend):

        self.graph_path = file_path
        self.nodes = []
        self.nodes_pos = []
        self.loadGraph()
        self.visualization(legend)


    def loadGraph(self):
        f = open(self.graph_path, 'rb')
        graph_base = pickle.load(f)
        f.close()

        nodes= graph_base.get_nodes()
        self.nodes = nodes
        for i, node in enumerate(nodes):
            # get children and parents of node
            pos, _, raceline, children, parents = graph_base.get_node_info(layer=node[0],
                                                                    node_number=node[1],
                                                                    return_child=True,
                                                                    return_parent=True)
            self.nodes_pos.append(pos)

    def visualization(self, legend):
        node_x = []
        node_y = []
        for i in range(len(self.nodes_pos)):
            node_x.append(self.nodes_pos[i][0])
            node_y.append(self.nodes_pos[i][1])

        plt.scatter(node_x, node_y, s = 1.5 )

if __name__ == "__main__":

    TRACK_NAME = 'LG_SVL'

    dir = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', '..'))
    pit_wpt = os.path.join(dir,'params',TRACK_NAME,'pit_lane.csv',)
    pit_in_wpt = os.path.join(dir,'params',TRACK_NAME,'pit_in_wpt.csv',)
    pit_out_wpt = os.path.join(dir,'params',TRACK_NAME,'pit_out_wpt.csv',)
    raceline = os.path.join(dir,'inputs/traj_ltpl_cl',TRACK_NAME,'traj_race_cl.csv',)
    graph = os.path.join(dir,'inputs/track_offline_graphs',TRACK_NAME,'stored_graph.pckl',)


    WPTFileVisualizer(pit_wpt, "pit-entire")
    WPTFileVisualizer(pit_in_wpt, "pit-in")
    WPTFileVisualizer(pit_out_wpt, "pit-out")
    RaceLineFileVisualizer(raceline, "race-line")
    GraphVisualizer(graph, "graph_node")
    plt.grid()
    plt.show()

