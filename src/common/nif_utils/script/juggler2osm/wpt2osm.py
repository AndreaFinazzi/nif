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
        print(points[i])
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
    parser = argparse.ArgumentParser(description='The csv data read')
    parser.add_argument('--filename', type=str, default='',
                        help='csv file name')
    parser.add_argument('--check_header', type=bool, default=False,
                        help='check headers name')
    parser.add_argument('--optional_dir', type=str, default='',
                        help='optionally add directory name')

    # lon_0, lat_0 = -86.235148, 39.809786  #indy
    lon_0, lat_0 = -86.3418060783425, 39.8125900071711  #Lucas Oil Racing
    
    nedPoints = []

    configs = edict(vars(parser.parse_args()))

    
    if(configs.optional_dir == ''):
        root_path = './wpt' 
    else:
        root_path = './wpt/' + configs.optional_dir

    file_name = os.path.join(root_path, configs.filename)
    osm_file_name = file_name[:-3] + "osm" 
    pcd_file_name = file_name[:-3] + "pcd" 

    # osm_file_name = os.path.join(root_path, osm_file_name)


    csv_file_read = pd.read_csv(file_name, sep=',', header=0)
    df = pd.DataFrame(csv_file_read)
    data_top = df.head()

    # print(df[lat_])

    print("===========" * 5)
    print("INPUT : " , file_name)
    print("OUTPUT OSM FILE: " , osm_file_name)
    print("OUTPUT PCD FILE: " , pcd_file_name)

    node_id = -237278
    node_id_list = []
    way_id = -99999999
    count = 1

    root = Element('osm', version='0.6', upload='false', generator='JOSM')
    
    x_prev = 0.
    y_prev = 0.
    idx = 0
    for x, y in tqdm(zip(df['x'] , df['y'])):
        if (np.isnan(x) == False and np.isnan(y) == False):
            # print("lat lon: " , lat, lon) 

            if (node_id == -237278):
                node_id -= 1
                continue
            llh = ned2geodetic(x, -y, 0.,lat_0, lon_0, 0.0)
            lat_buf = llh[0] 
            lon_buf = llh[1] 

            height = float(0.)
            yaw_deg = float(0.) * 180 / math.pi 
            velocity = float(0.) #km/h

            # print(lat, lon)
            lat = str(lat_buf)
            lon = str(lon_buf)
            alt = str(height) 
            yaw = str(yaw_deg)
            speed = str(velocity)
            # lat = string_values[0]
            # lon = string_values[1]
            node = Element('node', id='%d'%node_id, action='modify',lat=lat,lon=lon)
            tag_alt = Element('tag', k='alt',v=alt)
            tag_yaw = Element('tag', k='yaw',v=yaw)
            tag_speed = Element('tag',k='speed',v=speed)
            node.append(tag_alt)
            node.append(tag_yaw)
            node.append(tag_speed)
            root.append(node)
            node_id_list.append(node_id)
            node_id -= 1

            pointBuf = [x, y, 0.0, idx]
            nedPoints.append(pointBuf)
            # print(pointBuf)
            idx = idx + 1
    
    indent(root)
    # dump(root)            
    tree = ElementTree(root)
    tree.write(osm_file_name)
    write_pcd(nedPoints, pcd_file_name)


    print("===========" * 5)
    print("COMPLETED!")




if __name__ == "__main__":
    parse_configs()
