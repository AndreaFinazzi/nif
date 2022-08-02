#!/usr/bin/python

import dill
import matplotlib.pyplot as plt
import pandas as pd
from collections import defaultdict


csv_file_path = "/home/usrg/workspace/iac/AC_rosbag/rosbag2_2022_03_22-00_09_31/export/AC_bag_00000001.csv"
data = pd.read_csv(csv_file_path)

# LVMS
l_track_boundary = "/home/usrg/workspace/iac/AC_track_database/LVMS/linear_interpolated_trackbound/interpolated_lvms_inner_line.csv"
r_track_boundary = "/home/usrg/workspace/iac/AC_track_database/LVMS/linear_interpolated_trackbound/interpolated_lvms_outer_line.csv"
track_centerline_file_path = "/home/usrg/workspace/iac/AC_track_database/LVMS/centerline/ac_lvms_centerline.csv"
track_raceline_file_path = "/home/usrg/workspace/iac/AC_track_database/LVMS/raceline/raceline.csv"

# MONZA
# l_track_boundary = "/home/usrg/workspace/iac/AC_track_database/Monza/linear_interpolated_trackbound/monza_inner_line.csv"
# r_track_boundary = "/home/usrg/workspace/iac/AC_track_database/Monza/linear_interpolated_trackbound/monza_outer_line.csv"
# track_centerline_file_path = "/home/usrg/workspace/iac/AC_track_database/Monza/centerline/ac_monza_centerline.csv"
# track_raceline_file_path = "/home/usrg/workspace/iac/AC_track_database/Monza/raceline/raceline.csv"

l_data = pd.read_csv(l_track_boundary)
r_data = pd.read_csv(r_track_boundary)
c_data = pd.read_csv(track_centerline_file_path)
race_data = pd.read_csv(track_raceline_file_path)

filtered_ai_data = defaultdict(dict)
l_track_boundary_data = defaultdict(dict)
r_track_boundary_data = defaultdict(dict)
centerline_data = defaultdict(dict)
raceline_data = defaultdict(dict)

l_track_boundary_data["interpolated_x"] = pd.DataFrame(l_data, columns=["interpolated_x"]).values.tolist()
l_track_boundary_data["interpolated_y"] = pd.DataFrame(l_data, columns=["interpolated_y"]).multiply(-1).values.tolist()
l_track_boundary_data["interpolated_z"] = pd.DataFrame(l_data, columns=["interpolated_z"]).values.tolist()

r_track_boundary_data["interpolated_x"] = pd.DataFrame(r_data, columns=["interpolated_x"]).values.tolist()
r_track_boundary_data["interpolated_y"] = pd.DataFrame(r_data, columns=["interpolated_y"]).multiply(-1).values.tolist()
r_track_boundary_data["interpolated_z"] = pd.DataFrame(r_data, columns=["interpolated_z"]).values.tolist()

centerline_data["center_x"] = pd.DataFrame(c_data, columns=["center_x"]).values.tolist()
centerline_data["center_y"] = pd.DataFrame(c_data, columns=["center_y"]).multiply(-1).values.tolist()

raceline_data[" x_m"] = pd.DataFrame(race_data, columns=[" x_m"]).values.tolist()
raceline_data[" y_m"] = pd.DataFrame(race_data, columns=[" y_m"]).multiply(-1).values.tolist()

print(raceline_data[" pos_x"])


plt.plot(l_track_boundary_data["interpolated_x"], l_track_boundary_data["interpolated_y"],'darkgrey',label='Track boundaries')
plt.plot(r_track_boundary_data["interpolated_x"], r_track_boundary_data["interpolated_y"],'darkgrey')
plt.plot(centerline_data["center_x"], centerline_data["center_y"],label='Center line')
plt.plot(raceline_data[" x_m"], raceline_data[" y_m"],label='Race line')
plt.legend()



num_of_ai = 2
for ai_idx in range(1, num_of_ai + 1):
    pos_x_field = "AI_" + str(ai_idx) + "_pos_x"
    pos_y_field = "AI_" + str(ai_idx) + "_pos_z"

    filtered_ai_data[ai_idx]["pos_x"] = pd.DataFrame(data, columns=[pos_x_field]).values.tolist()
    filtered_ai_data[ai_idx]["pos_z"] = pd.DataFrame(data, columns=[pos_y_field]).values.tolist()
    
    # plt.plot(filtered_ai_data[ai_idx]["pos_x"], filtered_ai_data[ai_idx]["pos_z"])
    
idx = 0

plt.axes().set_aspect('equal')
plt.grid(True,'both')
plt.show()
