from xml.etree.ElementTree import Element, SubElement, ElementTree, dump
from xml.etree.ElementTree import parse
import glob
import os
from lxml import etree
import numpy as np
# from pyproj import Proj
import math
from pymap3d.ecef import ecef2geodetic
from pymap3d.ned import ned2geodetic
from pypcd import pypcd
# print(pypcd.__version__)
from tqdm import tqdm

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

# lon_0, lat_0 = utm_proj(126.7495693,37.6969797) #any latlon is okay
# lon_0, lat_0 = utm_proj(126.7495693,37.6969797) #ilsan
lon_0, lat_0 = -86.235148, 39.809786  #indy
# lon_0, lat_0 = -86.3418060783425, 39.8125900071711  #Lucas Oil Racing


### READ xml file ###
root_path = "./include/pcd"

pcd_path = os.path.join(root_path, 'full_map.pcd') # input 
osm_file_path = os.path.join(root_path, 'full_map.osm') # output

pc = pypcd.PointCloud.from_path(pcd_path)

print("===========" * 5)
print("INPUT : " , pcd_path)
print("OUTPUT : " , osm_file_path)
print(pc)

node_id = -237278
node_id_list = []
way_id = -99999999
count = 1

root = Element('osm', version='0.6', upload='false', generator='JOSM')


for point in tqdm(pc.pc_data):
    llh = ned2geodetic(point[0], -point[1], 0, lat_0, lon_0, 0, deg=True)

    # yaw_bias = yaw_bias * math.pi / 180
    # utm_x_trans = utm_x * math.cos(yaw_bias) - utm_y* math.sin(yaw_bias) 
    # utm_y_trans = utm_x * math.sin(yaw_bias) + utm_y* math.cos(yaw_bias) 
    
    # utm_lat = utm_x_trans + origin_x
    # utm_lon = utm_y_trans + origin_y

    if (node_id == -237278):
        node_id -= 1
        continue
    lat_buf = llh[0] 
    lon_buf = llh[1] 

    height = float(llh[2])
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

# way = Element('way', id='%d'%way_id, action='modify')
# for node_id_tmp in node_id_list:
#     nd = Element('nd', ref='%d'%node_id_tmp)
#     way.append(nd)
#     count +=1
#     if(count%20 == 0):
#         way = Element('way', id='%d'%way_id, action='modify')
#         way.append(nd)
#         next_way_id = way_id+1
#         # tag_way_id = Element('tag', k='this_id',v='%d'%way_id)
#         # tag_next_id = Element('tag',k='next_id',v='%d'%next_way_id)
#         # way.append(tag_way_id)
#         # way.append(tag_next_id)
#         way_id +=1

indent(root)
# dump(root)            
tree = ElementTree(root)
tree.write(osm_file_path)
print("===========" * 5)
print("COMPLETED!")




