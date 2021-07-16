#ifndef C_WPT_H_
#define C_WPT_H_

#include "nav_msgs/msg/path.hpp"
#include "nif_utils/frenet_path_generator.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

class c_wpt { // static wpt common class
public:
  c_wpt() {}
  c_wpt(string wpt_file_path_,
        string wpt_alias_,
        string global_frame_id_,
        bool wpt_3d_file_flg_,
        bool spline_flg_,
        double spline_interval_);
  ~c_wpt() {}

  vector<tuple<double, double>>
  load2DWPTFile(string wpt_2d_file_path_); // load 2d wpt from the file
  vector<tuple<double, double, double>>
  load3DWPTFile(string wpt_3d_file_path_); // load 3d wpt from the file

  string getWPTFilePath() {
    return m_wpt_file_path;
  }
  string getWPTAlias() {
    return m_wpt_alias;
  }
  bool isWPTSplined() {
    return m_spline_flg;
  }
  bool is3DWPT() {
    return m_3d_wpt_flg;
  }
  int getWPTSize() {
    return m_wpt_size;
  }
  double getWPTLength() {
    return m_wpt_length;
  }
  double getWPTSplineInterval() {
    return m_spline_interval;
  }

  nav_msgs::msg::Path getWPTinNavPath() {
    return m_wpt_inglobal;
  }

  vector<double> getWPTX() {
    return m_wpt_raw_x;
  }
  vector<double> getWPTY() {
    return m_wpt_raw_y;
  }
  vector<double> getWPTZ() {
    return m_wpt_raw_z;
  }

  vector<double> getSplinedWPTX() {
    return m_wpt_splined_x;
  }
  vector<double> getSplinedWPTY() {
    return m_wpt_splined_y;
  }
  vector<double> getSplinedWPTZ() {
    return m_wpt_splined_z;
  }
  vector<double> getSplinedWPTYaw() {
    return m_wpt_splined_yaw;
  }
  vector<double> getSplinedWPTCurvature() {
    return m_wpt_splined_curvature;
  }

private:
  string m_wpt_file_path;
  string m_wpt_alias;
  string m_global_frame_id;
  bool m_3d_wpt_flg;
  bool m_spline_flg;
  int m_wpt_size; // in index level
  double m_wpt_length;
  double m_spline_interval;

  nav_msgs::msg::Path m_wpt_inglobal;
  vector<tuple<double, double>> m_wpt_xy;
  vector<tuple<double, double, double>> m_wpt_xyz;
  vector<double> m_wpt_raw_x, m_wpt_raw_y, m_wpt_raw_z;
  vector<double> m_wpt_splined_x, m_wpt_splined_y, m_wpt_splined_z,
      m_wpt_splined_yaw, m_wpt_splined_curvature;
};

#endif // C_WPT_H_
