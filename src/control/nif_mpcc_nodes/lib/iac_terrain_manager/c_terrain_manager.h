#ifndef C_TERRAIN_MANAGER_H
#define C_TERRAIN_MANAGER_H

#include <algorithm> // std::min_element
#include <fstream>
#include <iostream>
#include <math.h>
#include <random>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <tuple>
#include <vector>

#include "mpcc_controller/Spline/arc_length_spline.h"
#include "mpcc_controller/types.h"

using namespace std;

class c_terrain_manager {
private:
  vector<double> m_track_inner_x;
  vector<double> m_track_inner_y;
  vector<double> m_track_inner_z;
  vector<double> m_track_outter_x;
  vector<double> m_track_outter_y;
  vector<double> m_track_outter_z;

  // wpt
  vector<double> m_wpt_m25_x;
  vector<double> m_wpt_m25_y;
  vector<double> m_wpt_m40_x;
  vector<double> m_wpt_m40_y;
  vector<double> m_wpt_magic_x;
  vector<double> m_wpt_magic_y;

  double m_track_inner_min_x = 0.0;
  double m_track_inner_min_y = 0.0;
  double m_track_inner_max_x = 0.0;
  double m_track_inner_max_y = 0.0;
  double m_track_outter_min_x = 0.0;
  double m_track_outter_min_y = 0.0;
  double m_track_outter_max_x = 0.0;
  double m_track_outter_max_y = 0.0;
  double m_track_center_x = 0.0;
  double m_track_center_y = 0.0;

  double turn1_in_x = 415.6;
  double turn1_in_y = -1129.5;
  double turn1_out_x = 141.2;
  double turn1_out_y = -870.0;

  double turn2_in_x = -62.91;
  double turn2_in_y = -875.8;
  double turn2_out_x = -315.5;
  double turn2_out_y = -1144.0;

  double turn3_in_x = -298.6;
  double turn3_in_y = -2151.2;
  double turn3_out_x = -31.4;
  double turn3_out_y = -2404.4;

  double turn4_in_x = 194.6;
  double turn4_in_y = -2396.2;
  double turn4_out_x = 434.1;
  double turn4_out_y = -2116.4;

  double turn1_x_min = std::min(turn1_in_x,turn1_out_x);
  double turn1_y_min = std::min(turn1_in_y,turn1_out_y);
  double turn1_x_max = std::max(turn1_in_x,turn1_out_x)+20.0;
  double turn1_y_max = std::max(turn1_in_y,turn1_out_y)+20.0;

  double turn2_x_min = std::min(turn2_in_x,turn2_out_x)-20.0;
  double turn2_y_min = std::min(turn2_in_y,turn2_out_y);
  double turn2_x_max = std::max(turn2_in_x,turn2_out_x);
  double turn2_y_max = std::max(turn2_in_y,turn2_out_y)+20.0;

  double turn3_x_min = std::min(turn3_in_x,turn3_out_x)-20.0;
  double turn3_y_min = std::min(turn3_in_y,turn3_out_y)-20.0;
  double turn3_x_max = std::max(turn3_in_x,turn3_out_x);
  double turn3_y_max = std::max(turn3_in_y,turn3_out_y);
    
  double turn4_x_min = std::min(turn4_in_x,turn4_out_x);
  double turn4_y_min = std::min(turn4_in_y,turn4_out_y)-20.0;
  double turn4_x_max = std::max(turn4_in_x,turn4_out_x)+20.0;
  double turn4_y_max = std::max(turn4_in_y,turn4_out_y);

  double rightBot_longStraight_x_min = std::min(turn1_in_x,turn4_out_x)-20.0;
  double rightBot_longStraight_y_min = std::min(turn1_in_y,turn4_out_y);
  double rightTop_longStraight_x_max = std::max(turn1_in_x,turn4_out_x)+20.0;
  double rightTop_longStraight_y_max = std::max(turn1_in_y,turn4_out_y);
  
  double leftBot_longStraight_x_min = std::min(turn2_out_x,turn3_in_x)-20.0;
  double leftBot_longStraight_y_min = std::min(turn2_out_y,turn3_in_y);
  double leftTop_longStraight_x_max = std::max(turn2_out_x,turn3_in_x)+20.0;
  double leftTop_longStraight_y_max = std::max(turn2_out_y,turn3_in_y);

  bool m_setTrack_flg = false;

  bool m_setBankMap_flg = false;
  bool m_setZMap_flg = false;

  int m_mapSize_x, m_mapSize_y;

  int m_trackSection_global;
  double m_initial_race_time = 2.0; // sec
  // double m_initial_race_time = 4.0; // sec
//   double m_start_center_up_x = -303.926; // for checking init left/right start position
//   double m_start_center_up_y = -1751.344;
//   double m_start_center_bot_x = -301.358;
//   double m_start_center_bot_y = -1867.877;
  // double m_start_center_up_x = -304.1; // for checking init left/right start position
  // double m_start_center_up_y = -1751.344;
  // double m_start_center_bot_x = -301.461;
  // double m_start_center_bot_y = -1902.66;
  double m_start_center_up_x = -304.1; // for checking init left/right start position
  double m_start_center_up_y = -1751.344;
  double m_start_center_bot_x = -302.00;
  double m_start_center_bot_y = -1902.66;

  // for checking raceline decision
  double turn4TorightStraight_x_min = 400.0;
  double turn4TorightStraight_x_max = 456.0;
  double turn4TorightStraight_y_min = -2060.0;
  double turn4TorightStraight_y_max = -2056.0;

  double turn2ToleftStraight_x_min = -330;
  double turn2ToleftStraight_x_max = -278;
  double turn2ToleftStraight_y_min = -1182;
  double turn2ToleftStraight_y_max = -1178;

  bool last_decision_flag = false;
  bool curr_decision_flag = false;

  // for checking raceline comback
  double rightStraightToTurn1_x_min = 400.0;
  double rightStraightToTurn1_x_max = 456.0;
  double rightStraightToTurn1_y_min = -1400;
  double rightStraightToTurn1_y_max = -1394;

  double leftStraightToTurn3_x_min = -330;
  double leftStraightToTurn3_x_max = -278;
  double leftStraightToTurn3_y_min = -1893;
  double leftStraightToTurn3_y_max = -1887;

  bool last_comeback_flag = false;
  bool curr_comeback_flag = false;

  // for raceline changing
  double rightStraightRacelineChange_x_min = 400.0;
  double rightStraightRacelineChange_x_max = 456.0;
  double rightStraightRacelineChange_y_min = -2056;
  double rightStraightRacelineChange_y_max = -1400;

  double leftStraightRacelineChange_x_min = -330;
  double leftStraightRacelineChange_x_max = -278;
  double leftStraightRacelineChange_y_min = -1887;
  double leftStraightRacelineChange_y_max = -1182;

  double raceline_change_dist = 8.0;
  double raceline_change_rel_v = 2.0/3.6;

  int m_initial_raceline = 0; // 0: not in initial pose, 1: initial left racingline, 2: initial right racingline
  bool m_done_init_raceline_check = false;
  int m_raceline_change_count = 0;
  
  double turn4_raceline_update_x_min = 400.0; 
  double turn4_raceline_update_x_max = 456.0; 
  double turn4_raceline_update_y_min = -2164.0;
  double turn4_raceline_update_y_max = -2060.0;

  string m_track_inner_file_path = "";
  string m_track_outter_file_path = "";
  string m_bankMap_file_path =
      "/home/usrg/ros2_ws/src/mpcc_ros2/lib/iac_terrain_manager/"
      "test_m_terrain_bankAngle_map.csv";
  string m_zMap_file_path = "/home/usrg/ros2_ws/src/mpcc_ros2/lib/"
                            "iac_terrain_manager/test_m_terrain_z_map.csv";

  double m_lookup_grid_res = 10.0; // in meter
  int m_mapSize_row;
  int m_mapSize_col;

  vector<vector<double>> m_terrain_z_map;         // in meter
  vector<vector<double>> m_terrain_bankAngle_map; // in rad

public:
  c_terrain_manager();
  c_terrain_manager(string track_inner_file_path,
                    string track_outter_file_path);
  c_terrain_manager(vector<double> track_inner_x, vector<double> track_inner_y,
                    vector<double> track_inner_z, vector<double> track_outter_x,
                    vector<double> track_outter_y,
                    vector<double> track_outter_z);
  vector<double> getTrack_inner_x() { return m_track_inner_x; }
  vector<double> getTrack_inner_y() { return m_track_inner_y; }
  vector<double> getTrack_inner_z() { return m_track_inner_z; }
  vector<double> getTrack_outter_x() { return m_track_outter_x; }
  vector<double> getTrack_outter_y() { return m_track_outter_y; }
  vector<double> getTrack_outter_z() { return m_track_outter_z; }

  double getInitialRaceTime() { return m_initial_race_time; }

  double calc_min_dist(double x, double y, std::vector<double> wpt_x, std::vector<double> wpt_y);

  void setTrackFilePaths(string track_inner_file_path,
                         string track_outter_file_path) {
    m_track_inner_file_path = track_inner_file_path;
    m_track_outter_file_path = track_outter_file_path;
  }

  void setTrack(vector<double> track_inner_x, vector<double> track_inner_y,
                vector<double> track_inner_z, vector<double> track_outter_x,
                vector<double> track_outter_y, vector<double> track_outter_z);

  tuple<vector<double>, vector<double>, vector<double>>
  getGeoDataFromCSV(string file_path);

  tuple<vector<double>, vector<double>>
  getGeoDataFromCSV_onlyWPT(string file_path);

  void getPreviouslyCalculatedBankNZmapFromCSV();

  void setTrack_minMax_xy() {
    m_track_inner_min_x =
        *min_element(m_track_inner_x.begin(), m_track_inner_x.end());
    m_track_inner_min_y =
        *min_element(m_track_inner_y.begin(), m_track_inner_y.end());
    m_track_inner_max_x =
        *max_element(m_track_inner_x.begin(), m_track_inner_x.end());
    m_track_inner_max_y =
        *max_element(m_track_inner_y.begin(), m_track_inner_y.end());
    m_track_outter_min_x =
        *min_element(m_track_outter_x.begin(), m_track_outter_x.end());
    m_track_outter_min_y =
        *min_element(m_track_outter_y.begin(), m_track_outter_y.end());
    m_track_outter_max_x =
        *max_element(m_track_outter_x.begin(), m_track_outter_x.end());
    m_track_outter_max_y =
        *max_element(m_track_outter_y.begin(), m_track_outter_y.end());

    m_track_center_x = (m_track_outter_min_x + m_track_outter_max_x) / 2;
    m_track_center_y = (m_track_outter_min_y + m_track_outter_max_y) / 2;
  }

  double getBankAngle_rad(double target_x, double target_y);
  double getZ_meter(double target_x, double target_y);
  bool getTrackRegion(double target_x, double target_y); // return true : in straight false : in curve
  double getTrackBoundaryModulation(double target_x, double target_y); // return the amount of the outer boundary modulation

  void setTrackArcLengthSpline(const mpcc::ArcLengthSpline &track);
  double getSectionPointProgress(double x, double y);
  int checkInitialRaceline(double ego_x, double ego_y, double ego_v, double sim_time, mpcc::State x_oppo); // 0: not in initial pose, 1: initial left racingline, 2: initial right racingline
  
  bool checkRaceLineDecisionRegion(double ego_x, double ego_y);
  bool checkRaceLineComebackRegion(double ego_x, double ego_y);

  mpcc::ArcLengthSpline track_;

  vector<double> getBankAngle_rad(vector<double> global_pos_x_vec,
                                  vector<double> global_pos_y_vec);
  ~c_terrain_manager();
};


#endif //C_TERRAIN_MANAGER_H

//////////////////////////////////////////////////////////
////////////////////// EXAMPLE USAGE /////////////////////
//////////////////////////////////////////////////////////

// 0. Include class into the ros node
// 1. Create object : c_terrain_manager obj = c_terrain_manager();
// 2. Pass global x and y [getBankAngle_rad / getZ_meter]