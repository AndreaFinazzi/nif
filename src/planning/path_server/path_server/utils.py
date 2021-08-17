# Copyright 2021 Matt Boler
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License. 

import numpy as np
import pandas as pd
import pymap3d as pm
from scipy.spatial.transform import Rotation

from timeit import default_timer as timer

from typing import List

class Pose(object):
    def __init__(self, 
            position: np.ndarray = np.zeros((3,1)), 
            orientation: Rotation = Rotation.from_euler('z', 0), 
        ):
        """
        """
        self.position = position # position of vehicle in world frame
        self.orientation = orientation # rotation from vehicle to world frame

class Path(object):
    """ A Path is an ordered collection of waypoints
    """
    def __init__(self,
            lat0: float,
            lon0: float
        ):
        """
        """
        self.lat0 = lat0
        self.lon0 = lon0
        self.debug = False
    def load_map(self,
            path: str = None
        ) -> bool:
        """ path to csv file containing waypoints
        Waypoints will be in a (lat, lon) in map.csv
        """
        if path is None:
            return False
        else:
            df = pd.read_csv(path)
            nodes = df.apply(lambda x: x.tolist(), axis=1)
            num_nodes = len(nodes)
            self.np_map = np.zeros((3, num_nodes), dtype=np.float32)
            self.np_yaws = np.zeros((1, num_nodes), dtype=np.float32)
            # Convert GPS map to local NED coordinates
            for idx, node in enumerate(nodes):
                lat = node[0]
                lon = node[1]
                n, e, d = pm.geodetic2ned(lat, lon, 0, self.lat0, self.lon0, 0)
                self.np_map[0, idx] = n
                self.np_map[1, idx] = e
                self.np_map[2, idx] = 0
                if (self.debug):
                    print("Found point:")
                    print(idx)
                    print(node)
                    print(np.asarray([n, e, d]))
            # Calculate yaws for curvature if needed
            for i in range(num_nodes-1):
                x1 = self.np_map[0, i]
                x2 = self.np_map[0, i+1]
                y1 = self.np_map[1, i]
                y2 = self.np_map[1, i+1]
                yaw = np.arctan2(y2-y1, x2-x1)
                self.np_yaws[0, i] = yaw
            if (self.debug):
                print("Loaded map: ")
                print(self.np_map)
            return True

class PathManager(object):
    """ The PathManager handles interactions between a Path and a body following the Path
    """
    def __init__(self, map_path, lat0, lon0) -> None:
        self.path = Path(lat0, lon0)
        self.path.load_map(map_path)
        self.debug = False
    
    def getNextPointsByDistance(self, 
        pose:Pose, 
        distance:float
        ) -> np.ndarray:
        nextPoints = self.searchFromNumpy(pose, distance)
        return nextPoints
    
    def searchFromNumpy(self, 
        pose, 
        distance
        ) -> np.ndarray:
        map_translated = self.path.np_map - pose.position
        local_map = np.dot(pose.orientation.as_matrix().T, map_translated)
        if self.debug:
            print("Map translated: ")
            print(map_translated)
            print("Local map:")
            print(local_map)
        is_forward = local_map[0, :] > 0
        is_near = np.sum(np.abs(local_map)**2, axis=0)**(1.0/2) < distance
        is_valid = np.logical_and(is_forward, is_near)
        good_np_map = local_map[:, is_valid]
        good_np_yaw = self.path.np_yaws[:, is_valid]
        np_points = np.vstack((good_np_map, good_np_yaw))
        np_points_sorted = np_points[:, np_points[0].argsort()]
        return np_points_sorted
