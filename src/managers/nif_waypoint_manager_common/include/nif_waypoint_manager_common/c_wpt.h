#ifndef C_WPT_H_
#define C_WPT_H_

#include "nif_utils/frenet_path_generator.h"

#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Paht.h>
#include <string>
#include <vector>

using namespace std;

class c_wpt { // static wpt common class
public:
  c_wpt() {}
  c_wpt(c_wpt& wpt_);
  c_wpt(string wpt_file_path_,
        string wpt_alias_ = "",
        string global_frame_id_ = "odom",
        bool 3d_wpt_file_flg_ = true;
        bool spline_flg_ = true, double spline_interval_ = 0.5);
  ~c_wpt();

  vector<tuple<double, double>>
  load2DWPTFile(string wpt_2d_file_path_); // load 2d wpt from the file
  vector<tuple<double, double, double>>
  load3DWPTFile(string wpt_3d_file_path_); // load 3d wpt from the file

  const string getWPTFilePath() const {return m_wpt_file_path};
  const string getWPTAlias() const {return m_wpt_alias};
  const bool isWPTSplined() const {
    return m_spline_flg;
  };
  const bool is3DWPT() const {
    return m_3d_wpt_flg;
  };
  const int getWPTSize() const {return m_wpt_size};
  const double getWPTLength() const {return m_wpt_length};
  const double getWPTSplineInterval() const {return m_spline_interval};

  const nav_msgs::Path getWPTinNavPath() const {return m_wpt_inglobal};

  const vector<double> getWPTX() const {return m_wpt_raw_x};
  const vector<double> getWPTY() const {return m_wpt_raw_y};
  const vector<double> getWPTZ() const {return m_wpt_raw_z};

  const vector<double> getSplinedWPTX() const {return m_wpt_splined_x};
  const vector<double> getSplinedWPTY() const {return m_wpt_splined_y};
  const vector<double> getSplinedWPTZ() const {return m_wpt_splined_z};
  const vector<double> getSplinedWPTYaw() const {return m_wpt_splined_yaw};
  const vector<double> getSplinedWPTCurvature() const {
      return m_wpt_splined_curvature};

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
}

#endif // C_WPT_H_
