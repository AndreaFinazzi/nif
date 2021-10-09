from xml.etree.ElementTree import Element, SubElement, ElementTree, dump
from xml.etree.ElementTree import parse
import glob
import os
from lxml import etree
import pandas as pd
import argparse
import numpy as np
from easydict import EasyDict as edict
import math
from tqdm import tqdm

from pymap3d.ecef import ecef2geodetic
from pymap3d.ned import ned2geodetic, geodetic2ned, geodetic2enu

import csv
import pickle
import sys
import logging
# sys.path.append('/home/usrg/adehome/nif/src/planning/nif_multilayer_planning_nodes/lib/GraphBasedLocalTrajectoryPlanner/graph_ltpl')
# # print(sys.path)
# import .
import graph_ltpl
import trajectory_planning_helpers as tph



HEADER = '''\
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4 
TYPE F F F F 
COUNT 1 1 1 1 
WIDTH {}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {}
DATA ascii
'''

def indent(elem, level=0): 
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

def write_pcd(points, save_pcd_path):
    n = len(points)
    lines = []
    for i in range(n):
        # print(points[i])
        x, y, z, i = points[i]
        lines.append('{:.6f} {:.6f} {:.6f} {}'.format(x, y, z, i))

    with open(save_pcd_path, 'w') as f:
        f.write(HEADER.format(n, n))
        f.write('\n'.join(lines))

def write_csv(points, save_csv_path):
    header = ['x', 'y']

    with open(save_csv_path, 'w', encoding='UTF8') as f:
        writer = csv.writer(f)

        # write the header
        writer.writerow(header)
        for point in points:
            # write the data
            writer.writerow(point)


def parse_configs():
    parser = argparse.ArgumentParser(description='Read the pickle data, and convert files')
    parser.add_argument('--filename', type=str, default='stored_graph.pckl',
                        help='csv file name')
    parser.add_argument('--check_header', type=bool, default=False,
                        help='check headers name')
    parser.add_argument('--set_topic', type=str, default="/novatel_bottom/bestpos",
                        help='check headers name')

    lon_0, lat_0 = -86.235148, 39.809786  #indy
    # lon_0, lat_0 = -86.3418060783425, 39.8125900071711  #Lucas Oil Racing
    
    nedPoints = []

    configs = edict(vars(parser.parse_args()))

    root_path = './pickle_files/IMS' 
    input_name = configs.filename
    file_name = os.path.join(root_path, input_name)
    osm_file_name = file_name[:-4] + "osm" 
    pcd_file_name = file_name[:-4] + "pcd" 

    f = open(file_name, 'rb')
    graph_base = pickle.load(f)
    f.close()

    print("Loaded database with " + str(len(graph_base.get_nodes()))
                                                        + " node and " + str(len(graph_base.get_edges()))
                                                        + " edges from file...")

    node_id = -237278
    node_id_list = []
    way_id = -99999999
    count = 1

    root = Element('osm', version='0.6', upload='false', generator='JOSM')
    
    x_prev = 0.
    y_prev = 0.
    idx = 0

    # print(graph_base.get_edges())
    print(graph_base)
    # tic = time.time()
    rmv_cnt = 0
    edge_cnt = 0
    nodes = graph_base.get_nodes()

    for i, node in enumerate(nodes):
        # get children and parents of node
        pos, _, raceline, children, parents = graph_base.get_node_info(layer=node[0],
                                                                node_number=node[1],
                                                                return_child=True,
                                                                return_parent=True)
        # print(pos)
        print(node[0] , node[1])
        # print(len(graph_base.get_layer_info(node[0])[0]))


        if (node_id == -237278):
                node_id -= 1
                continue

        llh = ned2geodetic(pos[0], -pos[1], 0.,lat_0, lon_0, 0.0)
        lat_buf = llh[0] 
        lon_buf = llh[1] 

        # print(lat, lon)
        lat = str(lat_buf)
        lon = str(lon_buf)
        
        layer = str(node[0])
        node_number = str(node[1])

        # lat = string_values[0]
        # lon = string_values[1]
        node = Element('node', id='%d'%node_id, action='modify',lat=lat,lon=lon)
        tag_layer_id = Element('tag', k='layer',v=layer)
        tag_node_number = Element('tag', k='node_number',v=node_number)

        # tag_yaw = Element('tag', k='yaw',v=yaw)
        # tag_speed = Element('tag',k='speed',v=speed)
        node.append(tag_layer_id)
        node.append(tag_node_number)
        root.append(node)
        node_id_list.append(node_id)
        node_id -= 1

        pointBuf = [pos[0], pos[1], 0.0, idx]
        nedPoints.append(pointBuf)
        # print(pointBuf)
        idx = idx + 1


    indent(root)
    # dump(root)            
    tree = ElementTree(root)
    tree.write(osm_file_name)
    write_pcd(nedPoints, pcd_file_name)


    print("===========" * 5)
    print("INPUT : " , file_name)
    print("OUTPUT OSM FILE: " , osm_file_name)
    print("OUTPUT PCD FILE: " , pcd_file_name)
    print("COMPLETED!")




if __name__ == "__main__":
    parse_configs()
