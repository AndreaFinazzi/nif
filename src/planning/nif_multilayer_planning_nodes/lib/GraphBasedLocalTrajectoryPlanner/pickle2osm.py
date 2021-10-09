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

    # lon_0, lat_0 = -86.235148, 39.809786  #indy
    lon_0, lat_0 = -86.3418060783425, 39.8125900071711  #Lucas Oil Racing
    
    nedPoints = []

    configs = edict(vars(parser.parse_args()))

    root_path = './pickle_files/LOR' 
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

    # way 
    for i in range(graph_base.num_layers):
        start_layer = i
        for s in range(graph_base.nodes_in_layer[start_layer]):
            pos, psi, raceline, children, _ = graph_base.get_node_info(layer=start_layer,
                                                                       node_number=s,
                                                                       return_child=True)
            # loop over child-nodes
            for node in children:
                edge_cnt += 1

                end_layer = node[0]
                e = node[1]
                spline = graph_base.get_edge(start_layer=start_layer,
                                             start_node=s,
                                             end_layer=end_layer,
                                             end_node=e)[0]
                x_coeff = np.atleast_2d(spline[0])
                y_coeff = np.atleast_2d(spline[1])
                # print(x_coeff, y_coeff)

                spline_sample, inds, t_values, _ = tph.interp_splines.interp_splines(coeffs_x=x_coeff,
                                                                                     coeffs_y=y_coeff,
                                                                                     stepsize_approx=1.0,
                                                                                     incl_last_point=True)
                # print(spline_sample)

                psi, kappa = tph.calc_head_curv_an.calc_head_curv_an(coeffs_x=x_coeff,
                                                                     coeffs_y=y_coeff,
                                                                     ind_spls=inds,
                                                                     t_spls=t_values)

                # print(pos)
                # print(node[0] , node[1])
                # print(len(graph_base.get_layer_info(node[0])[0]))
                node_id_list = []
                for pos_splined in spline_sample:
                    if (node_id == -237278):
                            node_id -= 1
                            continue
                    print(pos_splined)
                    llh = ned2geodetic(pos_splined[0], -pos_splined[1], 0.,lat_0, lon_0, 0.0)
                    lat_buf = llh[0] 
                    lon_buf = llh[1] 

                    # print(lat, lon)
                    lat = str(lat_buf)
                    lon = str(lon_buf)
                    
                    layer_s = str(start_layer)
                    layer_e = str(end_layer)
                    start_node = str(s)

                    # lat = string_values[0]
                    # lon = string_values[1]
                    node = Element('node', id='%d'%node_id, action='modify',lat=lat,lon=lon)
                    tag_start_layer = Element('tag', k='start_layer',v=layer_s)
                    tag_end_layer =   Element('tag', k='end_layer',v=layer_e)
                    tag_node_number = Element('tag', k='start_node',v=start_node)

                    node.append(tag_start_layer)
                    node.append(tag_end_layer)
                    node.append(tag_node_number)
                    root.append(node)
                    node_id_list.append(node_id)
                    node_id -= 1

                    pointBuf = [pos[0], pos[1], 0.0, idx]
                    nedPoints.append(pointBuf)
                    # print(pointBuf)
                    idx = idx + 1

                way = Element('way', id='%d'%way_id, action='modify')

                way_layer_s = str(start_layer)
                way_layer_e = str(end_layer)
                node_s = str(s)
                node_e = str(e)                
                way_tag_start_layer = Element('tag', k='start_layer',v=way_layer_s)
                way_tag_end_layer = Element('tag', k='end_layer',  v=way_layer_e)   
                way_tag_node_s = Element('tag', k='start_node',v=node_s)
                way_tag_node_e = Element('tag', k='end_node',  v=node_e)   

                for node_id_tmp in node_id_list:
                    nd = Element('nd', ref='%d'%node_id_tmp)
                    way.append(nd)
                way.append(way_tag_start_layer)
                way.append(way_tag_end_layer)                    
                way.append(way_tag_node_s)
                way.append(way_tag_node_e)                    

                root.append(way)
                way_id = way_id + 1


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
