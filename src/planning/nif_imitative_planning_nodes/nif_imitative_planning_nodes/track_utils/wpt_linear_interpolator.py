#!/usr/bin/python

from ast import parse
import dill
import matplotlib.pyplot as plt
import pandas as pd
from collections import defaultdict
import math
import csv
from itertools import zip_longest


wpt_file_path = "/home/usrg/workspace/iac/AC_track_database/LVMS/lvms_outer_line.csv"
wpt_data = pd.read_csv(wpt_file_path)

parsed_data = defaultdict(dict)

pos_x_field = "pos_x"
pos_y_field = "pos_y"
pos_z_field = "pos_z"

parsed_data["pos_x"] = pd.DataFrame(wpt_data, columns=[pos_x_field]).values.tolist()
parsed_data["pos_y"] = pd.DataFrame(wpt_data, columns=[pos_y_field]).values.tolist()
parsed_data["pos_z"] = pd.DataFrame(wpt_data, columns=[pos_z_field]).values.tolist()

interpolated_x = []
interpolated_y = []
interpolated_z = []

interpolate_dist = 1.0  # in meter

for idx in range(0, len(parsed_data["pos_x"]) - 1):
    x_0 = parsed_data["pos_x"][idx][0]
    x_1 = parsed_data["pos_x"][idx + 1][0]
    diff_x = x_1 - x_0
    y_0 = parsed_data["pos_y"][idx][0]
    y_1 = parsed_data["pos_y"][idx + 1][0]
    diff_y = y_1 - y_0
    z_0 = parsed_data["pos_z"][idx][0]
    z_1 = parsed_data["pos_z"][idx + 1][0]
    diff_z = z_1 - z_0

    dist = math.sqrt(pow(x_1 - x_0, 2) + pow(y_1 - y_0, 2))

    # for interpolate_idx in range(0,int(dist),interpolate_dist):
    #     interpolated_x.append(x_0 + diff_x * interpolate_idx)
    #     interpolated_y.append(y_0 + diff_y * interpolate_idx)
    #     interpolated_z.append(z_0 + diff_z * interpolate_idx)

    cum_dist = 0
    cum_count = 0
    while(cum_dist < dist):
        interpolated_x.append(x_0 + diff_x / dist * cum_count)
        interpolated_y.append(y_0 + diff_y / dist * cum_count)
        interpolated_z.append(z_0 + diff_z / dist * cum_count)
        cum_count += 1
        cum_dist += interpolate_dist

plt.plot(interpolated_x, interpolated_y)
plt.show()

interpolated_x.insert(0, "interpolated_x")
interpolated_y.insert(0, "interpolated_y")
interpolated_z.insert(0, "interpolated_z")

write_target = []
filename = "/home/usrg/workspace/iac/AC_track_database/LVMS/interpolated_lvms_outer_line.csv"

with open(filename, mode='w') as file:
    writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    write_target.append(interpolated_x)
    write_target.append(interpolated_y)
    write_target.append(interpolated_z)

    writer.writerows(zip_longest(*write_target, fillvalue=''))  # write to csv
