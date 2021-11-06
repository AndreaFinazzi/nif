#include "nif_waypoint_manager_common/c_wpt.h"

c_wpt::c_wpt(string wpt_file_path_, string wpt_alias_ = "",
             string global_frame_id_ = "map", bool wpt_3d_file_flg_ = false,
             bool spline_flg_ = false, double spline_interval_ = 0.5) {
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
  m_spline_flg = true;
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
      wpt_pt.pose.position.x = m_wpt_raw_x[wpt_idx];
      wpt_pt.pose.position.y = m_wpt_raw_y[wpt_idx];
      wpt_pt.pose.position.z = m_wpt_raw_z[wpt_idx];

      m_wpt_inglobal.poses.push_back(wpt_pt);
    }
    if (m_spline_flg) {
      shared_ptr<CubicSpliner2D> cubic_spliner_2D =
          std::make_shared<CubicSpliner2D>(m_wpt_raw_x, m_wpt_raw_y);
      shared_ptr<CubicSpliner> cubic_spliner_sz_ =
          std::make_shared<CubicSpliner>(cubic_spliner_2D->points_ss(),
                                         m_wpt_raw_z);
      double point_s = 0.0;
      double point_s_end = cubic_spliner_2D->points_s().back();
      m_wpt_length = point_s_end;

      m_wpt_inglobal.poses.clear();

      if (point_s_end < nif::common::constants::planner::WPT_MINIMUM_LEN) {
        throw std::domain_error("Length of the waypoint is abnormally short.");
        return;
      }

      while (point_s < point_s_end) {
        std::tuple<double, double> position =
            cubic_spliner_2D->calculate_position(point_s);

        double point_x = std::get<0>(position);
        double point_y = std::get<1>(position);
        double point_z =
            cubic_spliner_sz_->calculate_zeroth_derivative(point_s);
        double yaw = cubic_spliner_2D->calculate_yaw(point_s);
        double curvature = cubic_spliner_2D->calculate_curvature(point_s);

        if (std::isnan(point_x) || std::isnan(point_y) || std::isnan(point_z) ||
            std::isnan(yaw) || std::isnan(curvature)) {
          throw std::domain_error("Nan value is found in the splining result.");
          return;
        }

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

        wpt_pt.pose.orientation.x = 0.;
        wpt_pt.pose.orientation.y = 0.;
        wpt_pt.pose.orientation.z = std::sin(yaw / 2.);
        wpt_pt.pose.orientation.w = std::cos(yaw / 2.);

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
      wpt_pt.pose.position.x = m_wpt_raw_x[wpt_idx];
      wpt_pt.pose.position.y = m_wpt_raw_y[wpt_idx];
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

      if (point_s_end < nif::common::constants::planner::WPT_MINIMUM_LEN) {
        throw std::domain_error("Lenth of the waypoint is abnormally short.");
        return;
      }

      while (point_s < point_s_end) {
        std::tuple<double, double> position =
            cubic_spliner_2D->calculate_position(point_s);

        double point_x = std::get<0>(position);
        double point_y = std::get<1>(position);
        double yaw = cubic_spliner_2D->calculate_yaw(point_s);
        double curvature = cubic_spliner_2D->calculate_curvature(point_s);

        if (std::isnan(point_x) || std::isnan(point_y) || std::isnan(yaw) ||
            std::isnan(curvature)) {
          throw std::domain_error("Nan value is found in the splining result.");
          return;
        }

        m_wpt_splined_x.push_back(point_x);
        m_wpt_splined_y.push_back(point_y);
        m_wpt_splined_yaw.push_back(yaw);
        m_wpt_splined_curvature.push_back(curvature);

        geometry_msgs::msg::PoseStamped wpt_pt;
        wpt_pt.header.frame_id = m_wpt_inglobal.header.frame_id;
        wpt_pt.pose.position.x = point_x;
        wpt_pt.pose.position.y = point_y;
        wpt_pt.pose.position.z = 0.0;

        wpt_pt.pose.orientation.x = 0.;
        wpt_pt.pose.orientation.y = 0.;
        wpt_pt.pose.orientation.z = std::sin(yaw / 2.);
        wpt_pt.pose.orientation.w = std::cos(yaw / 2.);
        m_wpt_inglobal.poses.push_back(wpt_pt);

        point_s += abs(m_spline_interval);
      }
    }
  }
}

vector<tuple<double, double>> c_wpt::load2DWPTFile(string wpt_2d_file_path_,
                                                   int wpt_sampling_interval_) {
  vector<tuple<double, double>> data;
  ifstream inputFile(wpt_2d_file_path_);
  int l = 0;
  while (inputFile) {
    l++;
    string s;
    if (!getline(inputFile, s))
      break;
    if (s[0] != '#') {
      if (l % wpt_sampling_interval_ == 1) {
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
        if (nan_flg == false)
          data.push_back(record);
      }
    }
  }
  if (!inputFile.eof()) {
    cerr << "Could not read file " << wpt_2d_file_path_ << "\n";
    __throw_invalid_argument("File not found.");
  }
  return data;
}

vector<tuple<double, double, double>>
c_wpt::load3DWPTFile(string wpt_3d_file_path_, int wpt_sampling_interval_) {
  vector<tuple<double, double, double>> data;
  ifstream inputFile(wpt_3d_file_path_);
  int l = 0;
  while (inputFile) {
    string s;
    if (!getline(inputFile, s))
      break;
    if (s[0] != '#') {
      if (l % wpt_sampling_interval_ == 1) {
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