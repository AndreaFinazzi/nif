from xml.etree.ElementTree import Element, SubElement, ElementTree, dump
from xml.etree.ElementTree import parse
import glob
import os
from lxml import etree
import numpy as np

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
osm_file_path = '/home/usrg/adehome/nif/src/planning/nif_dk_graph_planner/map/IMS/IMS_fc.osm' # original
new_osm_file_path = '/home/usrg/adehome/nif/src/planning/nif_dk_graph_planner/map/IMS/IMS_fc_modified.osm' # modified


tree = parse(osm_file_path)
root = tree.getroot()

nodes = root.findall("node")
ways = root.findall("way")

ids = np.empty((0))
lats = np.empty((0))
lons = np.empty((0))

removed_ids = []
replaced_ids = []

lat_diff_thres = 0.0000001
lon_diff_thres = 0.0000001

for node in nodes:
    id_ = int(node.attrib['id'])
    lat = float(node.attrib['lat'])
    lon = float(node.attrib['lon'])
    tags = node.findall("tag")
    # lat = node.attrib['lat']
    # lon = node.attrib['lon']

    # same_idx = np.where((lats == lat) | (lons == lon))
    same_idx = np.where(((lats <= lat+lat_diff_thres) & (lats >= lat-lat_diff_thres)) & ((lons <= lon+lon_diff_thres) & (lons >= lon-lon_diff_thres)))
    
    if len(same_idx[0]) == 0:
        ids = np.hstack((ids, id_))
        lats = np.hstack((lats, lat))
        lons = np.hstack((lons, lon))

    # Find same lat, lon
    else:
        removed_ids.append(id_)
        idx = same_idx[0][0]
        replaced_ids.append(int(ids[idx]))

print(len(removed_ids))
print(len(replaced_ids))


for way in ways:
    nds = way.findall("nd")
    tags = way.findall("tag")

    for nd in nds:
        ref_nd = int(nd.attrib['ref'])

        if ref_nd in removed_ids:
            idx = removed_ids.index(ref_nd)
            nd.attrib['ref'] = str(replaced_ids[idx])

### WRITE ####
new_root = Element('osm', version='0.6', upload='false', generator='JOSM')

for node in nodes:
    id_ = int(node.attrib['id'])
    lat = float(node.attrib['lat'])
    lon = float(node.attrib['lon'])
    tags = node.findall("tag")
    
    if id_ in removed_ids:
        idx = removed_ids.index(id_)
        new_idx = replaced_ids[idx]
        # print(id_, new_idx)
        continue

    new_node = Element('node', id=node.attrib['id'], lat=node.attrib['lat'],lon=node.attrib['lon'])
    for tag_tmp in tags:
        new_nd = Element('tag', k=tag_tmp.attrib['k'], v=tag_tmp.attrib['v'])
        new_node.append(new_nd)
    new_root.append(new_node)


for way in ways:
    nds = way.findall("nd")
    tags = way.findall("tag")

    new_way = Element('way', id=way.attrib['id'], action='modify')

    for nd_tmp in nds:
        new_nd = Element('nd', ref=nd_tmp.attrib['ref'])
        new_way.append(new_nd)
    for tag_tmp in tags:
        new_nd = Element('tag', k=tag_tmp.attrib['k'], v=tag_tmp.attrib['v'])
        new_way.append(new_nd)
    new_root.append(new_way)
    
indent(new_root)
# dump(root)
new_tree = ElementTree(new_root)
new_tree.write(new_osm_file_path)
print('finish')

