import cubic_spliner
import wpt_file_reader
import matplotlib.pyplot as plt
import argparse
import sys
import os
import yaml
from yaml.loader import SafeLoader
from scipy import spatial as ss
import numpy as np
import distutils
import math


def argParser():
    parser = argparse.ArgumentParser(description='GenTrackGeoData')
    parser.add_argument('--inputYamlFilePath',
                    help='Path to the input yaml file path.')

    parser.add_argument('--visualizationFlg',
                    help='Visualization flag (Bool).')
    return parser


def readCSV(file_path_):
    path_x_list_ = []
    path_y_list_ = []

    cnt = 0
    fileHandle = open(file_path_, "r")
    for line in fileHandle:
        cnt += 1
        fields = line.split(",")
        if fields[0] != " " and fields[1] != " " and cnt != 1:
            x = float(fields[0])
            y = float(fields[1])
            path_x_list_.append(x)
            path_y_list_.append(y)
    fileHandle.close()

    # Check loaded data
    assert len(path_x_list_) == len(
        path_y_list_
    ), "path x and y length are different"
    assert len(path_x_list_) != 0, "path file empty"
    return path_x_list_, path_y_list_


def saveToFile(track_center_x, track_center_y, file_name):
    with open(file_name, 'a') as the_file:
        for i in range(len(track_center_x)):
            the_file.write(str(track_center_x[i]))
            the_file.write(',')
            the_file.write(str(track_center_y[i]))
            the_file.write('\n')
        the_file.write(str(track_center_x[0]))
        the_file.write(',')
        the_file.write(str(track_center_y[0]))
        the_file.write('\n')
        the_file.close()

    print("PROCESS DONE")


def genCenterLine(track_bound_left_x, track_bound_left_y, track_bound_right_x, track_bound_right_y):

    center_x = []
    center_y = []

    for i in range(len(track_bound_left_x)):

        dist_list = []
        pt_left_x = track_bound_left_x[i]
        pt_left_y = track_bound_left_y[i]

        for j in range(len(track_bound_right_x)):

            pt_right_x = track_bound_right_x[j]
            pt_right_y = track_bound_right_y[j]

            dist = math.sqrt(pow(pt_left_x - pt_right_x, 2) + pow(pt_left_y - pt_right_y, 2))
            dist_list.append(dist)

        min_dist_idx = dist_list.index(min(dist_list))

        center_x.append((track_bound_left_x[i] + track_bound_right_x[min_dist_idx]) / 2.0)
        center_y.append((track_bound_left_y[i] + track_bound_right_y[min_dist_idx]) / 2.0)

    return center_x, center_y


def main():  # pragma: no cover

    track_bound_left_file_path = "/home/usrg/adehome/nif/src/planning/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/right_center_line.csv"

    track_bound_left_x = []
    track_bound_left_y = []
    track_bound_left_splined_x = []
    track_bound_left_splined_y = []

    track_bound_right_file_path = "/home/usrg/adehome/nif/src/planning/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/lvms_outer_line.csv"
    track_bound_right_x = []
    track_bound_right_y = []
    track_bound_right_splined_x = []
    track_bound_right_splined_y = []

    track_center_file_name = "right_right_center_line.csv"
    track_center_x = []
    track_center_y = []

    # read files
    track_bound_left_x, track_bound_left_y = readCSV(track_bound_left_file_path)
    track_bound_right_x, track_bound_right_y = readCSV(track_bound_right_file_path)

    (track_bound_left_splined_x,
        track_bound_left_splined_y,
        _,
        _,
        _) = cubic_spliner.calcSpline([track_bound_left_x, track_bound_left_y], ds=1.0)

    (track_bound_right_splined_x,
        track_bound_right_splined_y,
        _,
        _,
        _) = cubic_spliner.calcSpline([track_bound_right_x, track_bound_right_y], ds=1.0)

    track_center_x, track_center_y = genCenterLine(track_bound_left_splined_x, track_bound_left_splined_y, track_bound_right_splined_x, track_bound_right_splined_y)

    plt.plot(track_center_x, track_center_y)
    # plt.axis('equal')
    plt.show()

    saveToFile(track_center_x, track_center_y, track_center_file_name)
    # saveToFile(track_bound_left_splined_x,track_bound_left_splined_y,"splined_left.csv")
    # saveToFile(track_bound_right_splined_x,track_bound_right_splined_y,"splined_right.csv")


if __name__ == "__main__":
    main()
