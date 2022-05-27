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


def argParser():
    parser = argparse.ArgumentParser(description='GenTrackGeoData')
    parser.add_argument('--inputYamlFilePath',
                    help='Path to the input yaml file path.')

    parser.add_argument('--visualizationFlg',
                    help='Visualization flag (Bool).')
    return parser


def main():  # pragma: no cover
    print("Start GenTrackGeoData...")
    arg_parser = argParser()
    parsed_args = arg_parser.parse_args(sys.argv[1:])

    if os.path.exists(parsed_args.inputYamlFilePath):
        print("---------------------")
        print("Input YAML file exist")
        print("---------------------")
    else:
        raise ValueError("Input YAML is not exist")

    vis_flg = parsed_args.visualizationFlg
    if vis_flg == None or False:
        vis_flg = False
    else:
        vis_flg = True

    track_name = None
    left_track_boundaries_file_path = None
    right_track_boundaries_file_path = None
    track_centerline_file_path = None
    output_directory = None
    coordination_type = None
    ref_lat = None
    ref_long = None
    spline_interval = None
    output_file_name = None

    track_left_boundary_splined_x = []
    track_left_boundary_splined_y = []
    track_left_boundary_splined_yaw = []
    track_left_boundary_splined_k = []
    track_left_boundary_splined_s = []

    track_right_boundary_splined_x = []
    track_right_boundary_splined_y = []
    track_right_boundary_splined_yaw = []
    track_right_boundary_splined_k = []
    track_right_boundary_splined_s = []

    track_centerline_splined_x = []
    track_centerline_splined_y = []
    track_centerline_splined_yaw = []
    track_centerline_splined_k = []
    track_centerline_splined_s = []

    with open(parsed_args.inputYamlFilePath) as f:
        data = yaml.load(f, Loader=SafeLoader)
        track_name = data.get('TrackName')
        output_file_name = track_name + ".csv"
        output_directory = data.get('OutputDirectory')
        coordination_type = data.get('CoordinationType')
        ref_lat = data.get('Ref_latitude')
        ref_long = data.get('Ref_longitude')
        if coordination_type == "GPS":
            if ref_lat == None:
                print("REFERENCE LATITUDE IS NOT SET. SET TO ZERO")
                ref_lat = 0.0
            if ref_long == None:
                print("REFERENCE LONGITUDE IS NOT SET. SET TO ZERO")
                ref_long = 0.0
        spline_interval = data.get('SplineInterval')
        if spline_interval == None:
            print("SPLINE INTERVAL IS NOT SET. SET TO ONE")
            spline_interval = 1.0

        for d in data.get('TrackBoundaries'):
            if next(iter(d.keys())) == 'LeftFilePath':
                left_track_boundaries_file_path = next(iter(d.values()))
        for d in data.get('TrackBoundaries'):
            if next(iter(d.keys())) == 'RightFilePath':
                right_track_boundaries_file_path = next(iter(d.values()))
        for d in data.get('TrackCenterLine'):
            if next(iter(d.keys())) == 'FilePath':
                track_centerline_file_path = next(iter(d.values()))

    print("------- SUMMURY -------")
    print("TRACK NAME : ", track_name)
    print("LEFT BOUNDARY FILE PATH : ", left_track_boundaries_file_path)
    print("RIGHT BOUNDARY FILE PATH : ", right_track_boundaries_file_path)
    print("CENTERLINE FILE PATH : ", track_centerline_file_path)
    print("COORDINATION TYPE : ", coordination_type)
    print("OUTPUT DIRECTORY : ", output_directory)
    if coordination_type == 'GPS':
        print("REF LAT AND LONG : ", ref_lat, " ", ref_long)
    print("-----------------------")

    print("START LOADING FILES...")
    # wpt_reader = wpt_file_reader.WPTFileReader(track_boundaries_file_path, track_centerline_file_path,
    #                                             coordination_type, ref_lat, ref_long)
    wpt_reader = wpt_file_reader.WPTFileReaderV2(left_track_boundaries_file_path, right_track_boundaries_file_path, track_centerline_file_path,
        coordination_type, ref_lat, ref_long)

    left_x, left_y = wpt_reader.getLeftTrackBoundaries()
    right_x, right_y = wpt_reader.getRightTrackBoundaries()
    center_x, center_y = wpt_reader.getTrackCenterline()
    print("LOADING DONE")

    print("START SPLINING...")

    (track_left_boundary_splined_x,
     track_left_boundary_splined_y,
     track_left_boundary_splined_yaw,
     track_left_boundary_splined_k,
     track_left_boundary_splined_s) = cubic_spliner.calcSpline(wpt_reader.getLeftTrackBoundaries(), ds=spline_interval)

    (track_right_boundary_splined_x,
     track_right_boundary_splined_y,
     track_right_boundary_splined_yaw,
     track_right_boundary_splined_k,
     track_right_boundary_splined_s) = cubic_spliner.calcSpline(wpt_reader.getRightTrackBoundaries(), ds=spline_interval)

    (track_centerline_splined_x,
     track_centerline_splined_y,
     track_centerline_splined_yaw,
     track_centerline_splined_k,
     track_centerline_splined_s) = cubic_spliner.calcSpline(wpt_reader.getTrackCenterline(), ds=spline_interval)

    print("SPLINING DONE")

    if(vis_flg):
        # scatter_idx = 450
        # plt.scatter(center_x[scatter_idx], center_y[scatter_idx])
        # scatter_idx = 700
        # plt.scatter(center_x[scatter_idx], center_y[scatter_idx])
        plt.plot(track_left_boundary_splined_x, track_left_boundary_splined_y, "--b", label="splined left boundary")
        plt.plot(track_right_boundary_splined_x, track_right_boundary_splined_y, "--g", label="splined right boundary")
        plt.plot(center_x, center_y, "r", label="splined centerline boundary")
        plt.legend()
        plt.axis('equal')
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.show()

    left_boundary_array = np.column_stack((track_left_boundary_splined_x, track_left_boundary_splined_y))
    right_boundary_array = np.column_stack((track_right_boundary_splined_x, track_right_boundary_splined_y))
    centerline_array = np.column_stack((track_centerline_splined_x, track_centerline_splined_y))

    left_tree = ss.cKDTree(left_boundary_array)
    right_tree = ss.cKDTree(right_boundary_array)
    min_dist_to_left, min_dist_to_left_id = left_tree.query(centerline_array)
    min_dist_to_right, min_dist_to_right_id = right_tree.query(centerline_array)

    print("START GENERATING THE FILE...")

    with open(output_directory + output_file_name, 'a') as the_file:
        the_file.write("# x_m, y_m, w_tr_right_m, w_tr_left_m")
        the_file.write('\n')
        for i in range(len(track_centerline_splined_x)):
            the_file.write(str(track_centerline_splined_x[i]))
            the_file.write(',')
            the_file.write(str(track_centerline_splined_y[i]))
            the_file.write(',')
            the_file.write(str(min_dist_to_right[i]))
            the_file.write(',')
            the_file.write(str(min_dist_to_left[i]))
            the_file.write('\n')
        the_file.close()

    print("PROCESS DONE")


if __name__ == "__main__":
    main()
