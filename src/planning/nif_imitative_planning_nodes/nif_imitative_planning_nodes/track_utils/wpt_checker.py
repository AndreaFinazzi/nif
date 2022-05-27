import sys
import pymap3d as pm
import numpy as np
import cubic_spliner
import matplotlib.pyplot as plt
import math


class WPTFileReader:
    def __init__(self, track_bound_file_path_, track_centerline_file_path_, coordination_repre_type_="NED", ref_lat_=0.0, ref_long_=0.0):
        self.track_bound_file_path = track_bound_file_path_
        self.track_centerline_file_path = track_centerline_file_path_

        coordination_type_list = ["NED", "GPS"]

        if coordination_repre_type_ not in coordination_type_list:
            raise ValueError('Coordination type is wrong.')
        self.coor_type = coordination_repre_type_

        self.ref_lat = ref_lat_
        self.ref_long = ref_long_

        # if coordination_repre_type_ == "GPS":
        #     print("YOUR REFERENCE LAT AND LONG ARE SET AS : ", self.ref_lat, " AND ", self.ref_long)
        #     print("PAY ATTENTION THAT THESE REFERENCE ARE IDENTICALLY SET IN THE FOLLOWING ALGORITHM (E.G. WPT MANAGER, PLANNER AND SO ON) ")

        self.track_bound_left_x_raw = []
        self.track_bound_left_y_raw = []
        self.track_bound_right_x_raw = []
        self.track_bound_right_y_raw = []

        self.track_centerline_x_raw = []
        self.track_centerline_y_raw = []

        self.track_centerline_x_raw, self.track_centerline_y_raw = self.readTrackCenterFile(self.track_centerline_file_path, self.coor_type)
        (self.track_bound_left_x_raw, self.track_bound_left_y_raw,
        self.track_bound_right_x_raw, self.track_bound_right_y_raw) = self.readTrackBoundaryFile(self.track_bound_file_path, self.coor_type)

    def getLeftTrackBoundaries(self):
        return self.track_bound_left_x_raw, self.track_bound_left_y_raw

    def getRightTrackBoundaries(self):
        return self.track_bound_right_x_raw, self.track_bound_right_y_raw

    def getTrackCenterline(self):
        return self.track_centerline_x_raw, self.track_centerline_y_raw

    def readTrackCenterFile(self, file_path_, coor_type_):
        path_x_list_ = []
        path_y_list_ = []

        fileHandle = open(file_path_, "r")
        if coor_type_ == "NED":
            for line in fileHandle:
                fields = line.split(",")
                if fields[0] != " " and fields[1] != " ":
                    x = float(fields[0])
                    y = float(fields[1])
                    path_x_list_.append(x)
                    path_y_list_.append(y)
            fileHandle.close()
        elif coor_type_ == "GPS":
            for line in fileHandle:
                fields = line.split(",")
                if fields[0] != " " and fields[1] != " ":
                    lat = float(fields[0])
                    long = float(fields[1])
                    x, y = self.convertFromInspva(lat, long, self.ref_lat, self.ref_long)
                    path_x_list_.append(x)
                    path_y_list_.append(y)
            fileHandle.close()
        else:
            raise ValueError('Coordination type is wrong.')

            # Check loaded data
        assert len(path_x_list_) == len(
            path_y_list_
        ), "path x and y length are different"
        assert len(path_x_list_) != 0, "path file empty"
        return path_x_list_, path_y_list_

    def readTrackBoundaryFile(self, file_path_, coor_type_):
        left_path_x_list_ = []
        left_path_y_list_ = []
        right_path_x_list_ = []
        right_path_y_list_ = []

        fileHandle = open(file_path_, "r")
        if coor_type_ == "NED":
            for line in fileHandle:
                fields = line.split(",")
                if fields[0] != " " and fields[1] != " " and fields[2] != " " and fields[3] != " ":
                    left_x = float(fields[0])
                    left_y = float(fields[1])
                    right_x = float(fields[2])
                    right_y = float(fields[3])
                    left_path_x_list_.append(left_x)
                    left_path_y_list_.append(left_y)
                    right_path_x_list_.append(right_x)
                    right_path_y_list_.append(right_y)
            fileHandle.close()
        elif coor_type_ == "GPS":
            for line in fileHandle:
                fields = line.split(",")
                if fields[0] != " " and fields[1] != " " and fields[2] != " " and fields[3] != " ":
                    left_lat = float(fields[0])
                    left_long = float(fields[1])
                    right_lat = float(fields[2])
                    right_long = float(fields[3])
                    left_x, left_y = self.convertFromInspva(left_lat, left_long, self.ref_lat, self.ref_long)
                    right_x, right_y = self.convertFromInspva(right_lat, right_long, self.ref_lat, self.ref_long)
                    left_path_x_list_.append(left_x)
                    left_path_y_list_.append(left_y)
                    right_path_x_list_.append(right_x)
                    right_path_y_list_.append(right_y)
            fileHandle.close()
        else:
            raise ValueError('Coordination type is wrong.')

        # Check loaded data
        assert len(left_path_x_list_) == len(
            left_path_y_list_
        ), "left boundary x and y length are different"
        assert len(left_path_x_list_) != 0, "left boundaries are empty"
        assert len(right_path_x_list_) == len(
            right_path_y_list_
        ), "right boundary x and y length are different"
        assert len(right_path_x_list_) != 0, "right boundaries are empty"
        return left_path_x_list_, left_path_y_list_, right_path_x_list_, right_path_y_list_

    def convertFromInspva(self, lat_, long_, ref_lat_, ref_long_):
        n, e, d = pm.geodetic2ned(lat_, long_, 0, ref_lat_, ref_long_, 0)
        position = np.asarray([[n], [e], [d]])
        return n, e


class WPTFileReaderV2:
    def __init__(self, left_track_bound_file_path_, right_track_bound_file_path_, track_centerline_file_path_, coordination_repre_type_="NED", ref_lat_=0.0, ref_long_=0.0):
        self.left_track_bound_file_path = left_track_bound_file_path_
        self.right_track_bound_file_path = right_track_bound_file_path_
        self.track_centerline_file_path = track_centerline_file_path_

        coordination_type_list = ["NED", "GPS"]

        if coordination_repre_type_ not in coordination_type_list:
            raise ValueError('Coordination type is wrong.')
        self.coor_type = coordination_repre_type_

        self.ref_lat = ref_lat_
        self.ref_long = ref_long_

        # if coordination_repre_type_ == "GPS":
        #     print("YOUR REFERENCE LAT AND LONG ARE SET AS : ", self.ref_lat, " AND ", self.ref_long)
        #     print("PAY ATTENTION THAT THESE REFERENCE ARE IDENTICALLY SET IN THE FOLLOWING ALGORITHM (E.G. WPT MANAGER, PLANNER AND SO ON) ")

        self.track_bound_left_x_raw = []
        self.track_bound_left_y_raw = []
        self.track_bound_right_x_raw = []
        self.track_bound_right_y_raw = []

        self.track_centerline_x_raw = []
        self.track_centerline_y_raw = []

        self.track_centerline_x_raw, self.track_centerline_y_raw = self.readTrackCenterFile(self.track_centerline_file_path, self.coor_type)
        self.track_bound_left_x_raw, self.track_bound_left_y_raw = self.readTrackCenterFile(self.left_track_bound_file_path, self.coor_type)
        self.track_bound_right_x_raw, self.track_bound_right_y_raw = self.readTrackCenterFile(self.right_track_bound_file_path, self.coor_type)
        # (self.track_bound_left_x_raw, self.track_bound_left_y_raw,
        # self.track_bound_right_x_raw, self.track_bound_right_y_raw) = self.readTrackBoundaryFile(self.track_bound_file_path, self.coor_type)

    def getLeftTrackBoundaries(self):
        return self.track_bound_left_x_raw, self.track_bound_left_y_raw

    def getRightTrackBoundaries(self):
        return self.track_bound_right_x_raw, self.track_bound_right_y_raw

    def getTrackCenterline(self):
        return self.track_centerline_x_raw, self.track_centerline_y_raw

    def readTrackCenterFile(self, file_path_, coor_type_):
        path_x_list_ = []
        path_y_list_ = []

        cnt = 0
        fileHandle = open(file_path_, "r")
        if coor_type_ == "NED":
            for line in fileHandle:
                cnt += 1
                fields = line.split(",")
                if fields[0] != " " and fields[1] != " ":
                    x = float(fields[0])
                    y = float(fields[1])
                    path_x_list_.append(x)
                    path_y_list_.append(y)
            fileHandle.close()
        elif coor_type_ == "GPS":
            for line in fileHandle:
                cnt += 1
                fields = line.split(",")
                if fields[0] != " " and fields[1] != " ":
                    lat = float(fields[0])
                    long = float(fields[1])
                    x, y = self.convertFromInspva(lat, long)
                    path_x_list_.append(x)
                    path_y_list_.append(y)
            fileHandle.close()
        else:
            raise ValueError('Coordination type is wrong.')

            # Check loaded data
        assert len(path_x_list_) == len(
            path_y_list_
        ), "path x and y length are different"
        assert len(path_x_list_) != 0, "path file empty"
        return path_x_list_, path_y_list_

    def readTrackBoundaryFile(self, file_path_, coor_type_):
        left_path_x_list_ = []
        left_path_y_list_ = []
        right_path_x_list_ = []
        right_path_y_list_ = []

        fileHandle = open(file_path_, "r")
        if coor_type_ == "NED":
            for line in fileHandle:
                fields = line.split(",")
                if fields[0] != " " and fields[1] != " " and fields[2] != " " and fields[3] != " ":
                    left_x = float(fields[0])
                    left_y = float(fields[1])
                    right_x = float(fields[2])
                    right_y = float(fields[3])
                    left_path_x_list_.append(left_x)
                    left_path_y_list_.append(left_y)
                    right_path_x_list_.append(right_x)
                    right_path_y_list_.append(right_y)
            fileHandle.close()
        elif coor_type_ == "GPS":
            for line in fileHandle:
                fields = line.split(",")
                if fields[0] != " " and fields[1] != " " and fields[2] != " " and fields[3] != " ":
                    left_lat = float(fields[0])
                    left_long = float(fields[1])
                    right_lat = float(fields[2])
                    right_long = float(fields[3])
                    left_x, left_y = self.convertFromInspva(left_lat, left_long, self.ref_lat, self.ref_long)
                    right_x, right_y = self.convertFromInspva(right_lat, right_long, self.ref_lat, self.ref_long)
                    left_path_x_list_.append(left_x)
                    left_path_y_list_.append(left_y)
                    right_path_x_list_.append(right_x)
                    right_path_y_list_.append(right_y)
            fileHandle.close()
        else:
            raise ValueError('Coordination type is wrong.')

        # Check loaded data
        assert len(left_path_x_list_) == len(
            left_path_y_list_
        ), "left boundary x and y length are different"
        assert len(left_path_x_list_) != 0, "left boundaries are empty"
        assert len(right_path_x_list_) == len(
            right_path_y_list_
        ), "right boundary x and y length are different"
        assert len(right_path_x_list_) != 0, "right boundaries are empty"
        return left_path_x_list_, left_path_y_list_, right_path_x_list_, right_path_y_list_

    def convertFromInspva(self, lat_, long_):
        n, e, d = pm.geodetic2ned(lat_, long_, 0, self.ref_lat, self.ref_long, 0)
        position = np.asarray([[n], [e], [d]])
        return n, e


class WPTSpliner:
    def __init__(self, track_file_path_, coordination_repre_type_="NED", ref_lat_=0.0, ref_long_=0.0, raw_file_sampling_interval_=100, splining_interval_=5.0):
        self.track_file_path = track_file_path_

        coordination_type_list = ["NED", "GPS"]

        if coordination_repre_type_ not in coordination_type_list:
            raise ValueError('Coordination type is wrong.')
        self.coor_type = coordination_repre_type_

        self.ref_lat = ref_lat_
        self.ref_long = ref_long_

        # if coordination_repre_type_ == "GPS":
        #     print("YOUR REFERENCE LAT AND LONG ARE SET AS : ", self.ref_lat, " AND ", self.ref_long)
        #     print("PAY ATTENTION THAT THESE REFERENCE ARE IDENTICALLY SET IN THE FOLLOWING ALGORITHM (E.G. WPT MANAGER, PLANNER AND SO ON) ")

        self.track_x_raw = []
        self.track_y_raw = []

        self.track_splined_x = []
        self.track_splined_y = []
        self.track_splined_yaw = []
        self.track_splined_k = []
        self.track_splined_s = []

        self.track_x_raw, self.track_y_raw = self.readTrackFile(self.track_file_path, self.coor_type, raw_file_sampling_interval_, splining_interval_)
        # (self.track_bound_left_x_raw, self.track_bound_left_y_raw,
        # self.track_bound_right_x_raw, self.track_bound_right_y_raw) = self.readTrackBoundaryFile(self.track_bound_file_path, self.coor_type)

    def getLeftTrackBoundaries(self):
        return self.track_bound_left_x_raw, self.track_bound_left_y_raw

    def getRightTrackBoundaries(self):
        return self.track_bound_right_x_raw, self.track_bound_right_y_raw

    def getTrackCenterline(self):
        return self.track_centerline_x_raw, self.track_centerline_y_raw

    def readTrackFile(self, file_path_, coor_type_, raw_file_sampling_interval_, splining_interval_):
        path_x_list_ = []
        path_y_list_ = []

        cnt = 0
        fileHandle = open(file_path_, "r")
        if coor_type_ == "NED":
            for line in fileHandle:
                if(line == 0):
                    continue
                cnt += 1
                fields = line.split(",")
                if fields[0] != " " and fields[1] != " " and cnt % raw_file_sampling_interval_ == 0:  # with sampling interval
                    x = float(fields[0])
                    y = float(fields[1])
                    path_x_list_.append(x)
                    path_y_list_.append(y)
                    cnt = 0
            # path_x_list_.append(path_x_list_[0])
            # path_y_list_.append(path_y_list_[0])
            fileHandle.close()
        elif coor_type_ == "GPS":
            for line in fileHandle:
                cnt += 1
                fields = line.split(",")
                if fields[0] != " " and fields[1] != " " and cnt % raw_file_sampling_interval_ == 0:
                    lat = float(fields[0])
                    long = float(fields[1])
                    x, y = self.convertFromInspva(lat, long)
                    path_x_list_.append(x)
                    path_y_list_.append(y)
            # path_x_list_.append(path_x_list_[0])
            # path_y_list_.append(path_y_list_[0])
            fileHandle.close()
        else:
            raise ValueError('Coordination type is wrong.')

            # Check loaded data
        assert len(path_x_list_) == len(
            path_y_list_
        ), "path x and y length are different"
        assert len(path_x_list_) != 0, "path file empty"

        (self.track_splined_x,
        self.track_splined_y,
        self.track_splined_yaw,
        self.track_splined_k,
        self.track_splined_s) = cubic_spliner.calcSpline([path_x_list_, path_y_list_], ds=splining_interval_)

        return path_x_list_, path_y_list_

    def convertFromInspva(self, lat_, long_):
        n, e, d = pm.geodetic2ned(lat_, long_, 0, self.ref_lat, self.ref_long, 0)
        position = np.asarray([[n], [e], [d]])
        return n, e

    def visualizeTrack(self):
        # plt.plot(self.track_x_raw, self.track_y_raw, "r", label="raw wpt")
        plt.plot(self.track_splined_x, self.track_splined_y, "--b", label="splined wpt")
        plt.axis('equal')

    def visualizeYaw(self):
        plt.plot(self.track_splined_yaw)

    def saveToFile(self):
        with open("/home/usrg/workspace/iac/GenTrackGeoData/output/lgsim/lvms/nif_map/trackboundary_left.csv", 'a') as the_file:
            for i in range(len(self.track_splined_x)):
                the_file.write(str(self.track_splined_x[i]))
                the_file.write(',')
                the_file.write(str(self.track_splined_y[i]))
                the_file.write('\n')
            the_file.write(str(self.track_splined_x[0]))
            the_file.write(',')
            the_file.write(str(self.track_splined_y[0]))
            the_file.write('\n')
            the_file.close()

    print("PROCESS DONE")


def main():  # pragma: no cover
    # /home/usrg/workspace/iac/iac_wpt_data/wpt_files/LVMS/center_line.csv
    # /home/usrg/workspace/iac/iac_wpt_data/wpt_files/LVMS/left_side_center.csv
    # /home/usrg/workspace/iac/iac_wpt_data/wpt_files/LVMS/raceline.csv
    # /home/usrg/workspace/iac/iac_wpt_data/wpt_files/LVMS/right_side.csv
    obj1 = WPTSpliner("/home/usrg/adehome/nif/src/planning/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/center_line.csv", raw_file_sampling_interval_=3, splining_interval_=1.0)
    obj2 = WPTSpliner("/home/usrg/adehome/nif/src/planning/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/lvms_inner_line.csv", raw_file_sampling_interval_=3, splining_interval_=1.0)
    obj3 = WPTSpliner("/home/usrg/adehome/nif/src/planning/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/lvms_outer_line.csv", raw_file_sampling_interval_=3, splining_interval_=1.0)
    obj4 = WPTSpliner("/home/usrg/adehome/nif/src/planning/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/left_center_line.csv", raw_file_sampling_interval_=3, splining_interval_=1.0)
    obj5 = WPTSpliner("/home/usrg/adehome/nif/src/planning/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/race_line.csv", raw_file_sampling_interval_=3, splining_interval_=1.0)
    obj6 = WPTSpliner("/home/usrg/adehome/nif/src/planning/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/right_center_line.csv", raw_file_sampling_interval_=3, splining_interval_=1.0)
    obj7 = WPTSpliner("/home/usrg/adehome/nif/src/planning/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/center_right_center_line.csv", raw_file_sampling_interval_=3, splining_interval_=1.0)
    obj8 = WPTSpliner("/home/usrg/adehome/nif/src/planning/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/left_center_center_line.csv", raw_file_sampling_interval_=3, splining_interval_=1.0)
    obj9 = WPTSpliner("/home/usrg/adehome/nif/src/planning/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/left_left_center_line.csv", raw_file_sampling_interval_=3, splining_interval_=1.0)
    obj10 = WPTSpliner("/home/usrg/adehome/nif/src/planning/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/right_right_center_line.csv", raw_file_sampling_interval_=3, splining_interval_=1.0)

    obj1.visualizeTrack()
    obj2.visualizeTrack()
    obj3.visualizeTrack()
    obj4.visualizeTrack()
    obj5.visualizeTrack()
    obj6.visualizeTrack()
    obj7.visualizeTrack()
    obj8.visualizeTrack()
    obj9.visualizeTrack()
    obj10.visualizeTrack()
    plt.show()

    obj1.visualizeYaw()
    obj2.visualizeYaw()
    obj3.visualizeYaw()
    obj4.visualizeYaw()
    obj5.visualizeYaw()
    obj6.visualizeYaw()
    obj7.visualizeYaw()
    obj8.visualizeYaw()
    obj9.visualizeYaw()
    obj10.visualizeYaw()
    plt.show()


if __name__ == "__main__":
    main()
