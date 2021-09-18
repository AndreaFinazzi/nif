from xml.etree.ElementTree import Element, SubElement, ElementTree, dump
from xml.etree.ElementTree import parse
import glob
import os
from lxml import etree
import numpy as np
from pyproj import Proj
import math

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


### READ xml file ###
root_path = "./include/waypoints"

osm_file_path = os.path.join(root_path, 'LOR_inner_new.osm') # input
pcd_path = os.path.join(root_path, 'LOR_inner_new.pcd') # output 

tree = parse(osm_file_path)
root = tree.getroot()

nodes = root.findall("node")
ways = root.findall("way")

ids = np.empty((0))
lats = np.empty((0))
lons = np.empty((0))

removed_ids = []
replaced_ids = []

# utm_proj = Proj("+proj=utm +zone=52N, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
utm_proj = Proj("+proj=utm +zone=16N, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")

# origin_y, origin_x = utm_proj(126.7495693,37.6969797) #any latlon is okay
# origin_y, origin_x = utm_proj(126.7495693,37.6969797) #ilsan
# origin_y, origin_x = utm_proj(-86.235148, 39.809786)  #indy
origin_y, origin_x = utm_proj(-86.3418060783425, 39.8125900071711)  #Lucas Oil Racing



print(origin_x, origin_y)
ConvertedPoints = []

# def write_pcd(points, save_pcd_path):

#     with open(save_pcd_path, 'w') as f:
#         f.write(HEADER.format(len(points), len(points)) + '\n')
#         for point in points:
#             np.savetxt(f, point, delimiter = ' ', fmt = '%f %f %f %d')

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
    
idx = 0
for node in nodes:
    id_ = int(node.attrib['id'])
    lat = float(node.attrib['lat'])
    lon = float(node.attrib['lon'])
    y, x = utm_proj(lon, lat, inverse=False)
    x = x - origin_x 
    y = y - origin_y 
    # yaw_bias = 2.
    yaw_bias = -90.

    yaw_bias = yaw_bias * math.pi / 180
    utm_x = x * math.cos(yaw_bias) + y* math.sin(yaw_bias)
    utm_y = -x * math.sin(yaw_bias) + y* math.cos(yaw_bias)

    # print(lat, lon, x ,y)
    pointBuf = [utm_y, utm_x, 0.0, idx]
    print(pointBuf)
    ConvertedPoints.append(pointBuf)
    # print(pointBuf)
    idx = idx + 1
    
write_pcd(ConvertedPoints, pcd_path)
    
    