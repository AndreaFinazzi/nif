#!/usr/bin/python

from traceback import print_tb
import dill
import matplotlib.pyplot as plt
import pandas as pd
from collections import defaultdict
import math

csv_file_path = "/media/usrg/T7/rosbag2_2022_04_04-21_30_41/export/AC_bag_00000001.csv"
data = pd.read_csv(csv_file_path)

filtered_ai_data = defaultdict(dict)

plt.subplot(2, 1, 1)
num_of_ai = 1
for ai_idx in range(1, num_of_ai + 1):
    pos_x_field = "AI_" + str(ai_idx) + "_pos_x"
    pos_y_field = "AI_" + str(ai_idx) + "_pos_y"
    pos_z_field = "AI_" + str(ai_idx) + "_pos_z"

    vel_x_field = "AI_" + str(ai_idx) + "_linear_velocity_vector_x"  # AI_1_linear_velocity_vector_x
    vel_y_field = "AI_" + str(ai_idx) + "_linear_velocity_vector_y"  # AI_1_linear_velocity_vector_z
    vel_z_field = "AI_" + str(ai_idx) + "_linear_velocity_vector_z"  # AI_1_linear_velocity_vector_y

    pos_x = pd.DataFrame(data, columns=[pos_x_field]).values.tolist()
    pos_y = pd.DataFrame(data, columns=[pos_y_field]).values.tolist()
    pos_z = pd.DataFrame(data, columns=[pos_z_field]).values.tolist()

    vel_x = pd.DataFrame(data, columns=[vel_x_field]).values.tolist()
    vel_z = pd.DataFrame(data, columns=[vel_y_field]).values.tolist()
    vel_y = pd.DataFrame(data, columns=[vel_z_field]).values.tolist()

    # x_axis_rot = math.atan2(vel_z, vel_y)
    # y_axis_rot = math.atan2(vel_x, vel_z)
    # z_axis_rot = math.atan2(vel_y, vel_x)

    minus_pos_x = [-1 * value[0] for value in pos_x]
    minus_pos_z = [-1 * value[0] for value in pos_z]
    minus_pos_y = [-1 * value[0] for value in pos_y]

    minus_vel_x = [-1 * value[0] for value in vel_x]
    minus_vel_z = [-1 * value[0] for value in vel_z]
    minus_vel_y = [-1 * value[0] for value in vel_y]

    plt.plot(minus_pos_z, minus_pos_x)

plt.scatter(minus_pos_z[0], minus_pos_x[0])
plt.scatter(minus_pos_z[1000], minus_pos_x[1000])
plt.scatter(minus_pos_z[2000], minus_pos_x[2000])
plt.scatter(minus_pos_z[3000], minus_pos_x[3000])
plt.scatter(minus_pos_z[8657], minus_pos_x[8657])
plt.scatter(minus_pos_z[4260], minus_pos_x[4260])

plt.subplot(2, 1, 2)
angle_x = []
angle_y = []
angle_z = []
for i, (rot_x, rot_y, rot_z) in enumerate(zip(vel_x, vel_y, vel_z)):
    x_axis_rot = math.atan2(rot_z[0], rot_y[0])
    y_axis_rot = math.atan2(rot_x[0], rot_z[0])
    z_axis_rot = math.atan2(rot_y[0], rot_x[0])

    angle_x.append(x_axis_rot)
    angle_y.append(y_axis_rot)
    angle_z.append(z_axis_rot)

plt.plot(angle_z)
plt.scatter(0, angle_z[0])
plt.scatter(1000, angle_z[1000])
plt.scatter(2000, angle_z[2000])
plt.scatter(3000, angle_z[3000])
plt.scatter(8657, angle_z[8657])
plt.scatter(4260, angle_z[4260])
print(angle_z[8657])

plt.grid(True)
plt.show()
