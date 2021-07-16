#include "nif_waypoint_manager_common/c_wpt.h"

c_wpt::c_wpt(string wpt_file_path_,
             string wpt_alias_ = "",
             string global_frame_id_ = "odom",
             bool wpt_3d_file_flg_ = true,
             bool spline_flg_ = true,
             double spline_interval_ = 0.5) {
  //  init
  m_wpt_raw_x.clear();
  m_wpt_splined_x.clear();
  m_wpt_raw_y.clear();
  m_wpt_splined_y.clear();
  m_wpt_raw_z.clear();
  m_wpt_splined_z.clear();

  m_wpt_file_path = wpt_file_path_;
  m_wpt_alias = wpt_alias_;
  m_global_frame_id = global_frame_id_;
  m_spline_interval = spline_interval_;
  m_spline_flg = spline_flg_;
  m_3d_wpt_flg = wpt_3d_file_flg_;

  m_wpt_inglobal.header.frame_id = m_global_frame_id;

  if (m_3d_wpt_flg) {
    m_wpt_xyz = load3DWPTFile(m_wpt_file_path);
    m_wpt_size = m_wpt_xyz.size();
    for (int wpt_idx = 0; wpt_idx < m_wpt_xyz.size(); wpt_idx++) {
      m_wpt_raw_x.push_back(get<0>(m_wpt_xyz[wpt_idx]));
      m_wpt_raw_y.push_back(get<1>(m_wpt_xyz[wpt_idx]));
      m_wpt_raw_z.push_back(get<2>(m_wpt_xyz[wpt_idx]));

      geometry_msgs::msg::PoseStamped wpt_pt;
      wpt_pt.header.frame_id = m_wpt_inglobal.header.frame_id;
      wpt_pt.pose.position.x = m_wpt_raw_x[-1];
      wpt_pt.pose.position.y = m_wpt_raw_y[-1];
      wpt_pt.pose.position.z = m_wpt_raw_z[-1];
      m_wpt_inglobal.poses.push_back(wpt_pt);
    }
    if (m_spline_flg) {
      shared_ptr<CubicSpliner2D> cubic_spliner_2D(
          new CubicSpliner2D(m_wpt_raw_x, m_wpt_raw_y));
      shared_ptr<CubicSpliner> cubic_spliner_sz_ = shared_ptr<CubicSpliner>(
          new CubicSpliner(cubic_spliner_2D->points_ss(), m_wpt_raw_z));
      double point_s = 0.0;
      double point_s_end = cubic_spliner_2D->points_s().back();
      m_wpt_length = point_s_end;

      m_wpt_inglobal.poses.clear();

      while (point_s < point_s_end) {
        std::tuple<double, double> position =
            cubic_spliner_2D->calculate_position(point_s);

        double point_x = std::get<0>(position);
        double point_y = std::get<1>(position);
        double point_z =
            cubic_spliner_sz_->calculate_zeroth_derivative(point_s);
        double yaw = cubic_spliner_2D->calculate_yaw(point_s);
        double curvature = cubic_spliner_2D->calculate_curvature(point_s);

        m_wpt_splined_x.push_back(point_x);
        m_wpt_splined_y.push_back(point_y);
        m_wpt_splined_z.push_back(point_z);
        m_wpt_splined_yaw.push_back(yaw);
        m_wpt_splined_curvature.push_back(curvature);

        geometry_msgs::msg::PoseStamped wpt_pt;
        wpt_pt.header.frame_id = m_wpt_inglobal.header.frame_id;
        wpt_pt.pose.position.x = point_x;
        wpt_pt.pose.position.y = point_y;
        wpt_pt.pose.position.z = point_z;
        m_wpt_inglobal.poses.push_back(wpt_pt);

        point_s += abs(m_spline_interval);
      }
    }
  } else {
    m_wpt_xy = load2DWPTFile(m_wpt_file_path);
    m_wpt_size = m_wpt_xy.size();
    for (int wpt_idx = 0; wpt_idx < m_wpt_xy.size(); wpt_idx++) {
      m_wpt_raw_x.push_back(get<0>(m_wpt_xy[wpt_idx]));
      m_wpt_raw_y.push_back(get<1>(m_wpt_xy[wpt_idx]));

      geometry_msgs::msg::PoseStamped wpt_pt;
      wpt_pt.header.frame_id = m_wpt_inglobal.header.frame_id;
      wpt_pt.pose.position.x = m_wpt_raw_x[-1];
      wpt_pt.pose.position.y = m_wpt_raw_y[-1];
      wpt_pt.pose.position.z = 0.0;
      m_wpt_inglobal.poses.push_back(wpt_pt);
    }
    if (m_spline_flg) {
      shared_ptr<CubicSpliner2D> cubic_spliner_2D =
        make_shared<CubicSpliner2D>(m_wpt_raw_x, m_wpt_raw_y);
      double point_s = 0.0;
      double point_s_end = cubic_spliner_2D->points_s().back();
      m_wpt_length = point_s_end;

      m_wpt_inglobal.poses.clear();

      while (point_s < point_s_end) {
        std::tuple<double, double> position =
            cubic_spliner_2D->calculate_position(point_s);

        double point_x = std::get<0>(position);
        double point_y = std::get<1>(position);
        double yaw = cubic_spliner_2D->calculate_yaw(point_s);
        double curvature = cubic_spliner_2D->calculate_curvature(point_s);

        m_wpt_splined_x.push_back(point_x);
        m_wpt_splined_y.push_back(point_y);
        m_wpt_splined_yaw.push_back(yaw);
        m_wpt_splined_curvature.push_back(curvature);

        geometry_msgs::msg::PoseStamped wpt_pt;
        wpt_pt.header.frame_id = m_wpt_inglobal.header.frame_id;
        wpt_pt.pose.position.x = point_x;
        wpt_pt.pose.position.y = point_y;
        wpt_pt.pose.position.z = 0.0;
        m_wpt_inglobal.poses.push_back(wpt_pt);

        point_s += abs(m_spline_interval);
      }
    }
  }
}

vector<tuple<double, double>> c_wpt::load2DWPTFile(string wpt_2d_file_path_) {
  vector<tuple<double, double>> data;
  ifstream inputFile(wpt_2d_file_path_);
  int l = 0;
  while (inputFile) {
      if (l % 1000 == 0) {
          string s;
          if (!getline(inputFile, s))
              break;
          if (s[0] != '#') {
              istringstream ss(s);
              tuple<double, double> record(0.0, 0.0);
              int cnt = 0;
              bool nan_flg = false;
              while (ss) {
                  string line;
                  if (!getline(ss, line, ','))
                      break;
                  try {
                      if (cnt == 0)
                          get<0>(record) = stof(line);
                      else if (cnt == 1)
                          get<1>(record) = stof(line);
                  } catch (const invalid_argument e) {
                      cout << "NaN found in file " << wpt_2d_file_path_ << " line " << l
                           << endl;
                      e.what();
                      nan_flg = true;
                  }
                  cnt++;
              }
              if (nan_flg == false && l % 1000 == 0)
                  data.push_back(record);
          }
      }
      l++;
  }
  if (!inputFile.eof()) {
    cerr << "Could not read file " << wpt_2d_file_path_ << "\n";
    __throw_invalid_argument("File not found.");
  }
  return data;
}

vector<tuple<double, double, double>>
c_wpt::load3DWPTFile(string wpt_3d_file_path_) {
  vector<tuple<double, double, double>> data;
  ifstream inputFile(wpt_3d_file_path_);
  int l = 0;
  while (inputFile) {
      if (l % 100 == 0) {
          string s;
          if (!getline(inputFile, s))
              break;
          if (s[0] != '#') {
              istringstream ss(s);
              tuple<double, double, double> record(0.0, 0.0, 0.0);

              int cnt = 0;
              bool nan_flg = false;
              while (ss) {
                  string line;
                  if (!getline(ss, line, ','))
                      break;
                  try {
                      if (cnt == 0)
                          get<0>(record) = stof(line);
                      else if (cnt == 1)
                          get<1>(record) = stof(line);
                      else if (cnt == 2)
                          get<2>(record) = stof(line);
                  } catch (const invalid_argument e) {
                      cout << "NaN found in file " << wpt_3d_file_path_ << " line " << l
                           << endl;
                      e.what();
                      nan_flg = true;
                  }
                  cnt++;
              }
              if (nan_flg == false)
                  data.push_back(record);
          }
      }
  l++;
  }

  if (!inputFile.eof()) {
    cerr << "Could not read file " << wpt_3d_file_path_ << "\n";
    __throw_invalid_argument("File not found.");
  }

  return data;
}