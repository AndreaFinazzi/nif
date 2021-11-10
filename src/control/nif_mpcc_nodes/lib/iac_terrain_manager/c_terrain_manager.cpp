#include "c_terrain_manager.h"

using namespace std;

c_terrain_manager::c_terrain_manager() {
  // std::cout << "Load previously calculated Bank and Z map" << std::endl;

  string outter_track_file_path =
      "/home/sw/ros2_ws/src/mpcc_ros2/lib/iac_terrain_manager/"
      "terrain_data/"
      "VehiclePlayerExport_outter_path_dataLog_v2.csv";

  string m25_wpt_file_path =
      "/home/sw/ros2_ws/src/mpcc_ros2/src/mpcc_controller/mpcc_controller/Params/m25_wpt.csv";
  string m40_wpt_file_path =
      "/home/sw/ros2_ws/src/mpcc_ros2/src/mpcc_controller/mpcc_controller/Params/m40_wpt.csv";
  string magic_wpt_file_path =
      "/home/sw/ros2_ws/src/mpcc_ros2/src/mpcc_controller/mpcc_controller/Params/magic_wpt.csv";

  auto outterTrackGeoData = getGeoDataFromCSV(outter_track_file_path);

  auto m25WPTGeoData = getGeoDataFromCSV_onlyWPT(m25_wpt_file_path);
  auto m40WPTGeoData = getGeoDataFromCSV_onlyWPT(m40_wpt_file_path);
  auto magicWPTGeoData = getGeoDataFromCSV_onlyWPT(magic_wpt_file_path);

  m_wpt_m25_x = get<0>(m25WPTGeoData);
  m_wpt_m25_y = get<1>(m25WPTGeoData);
  m_wpt_m40_x = get<0>(m40WPTGeoData);
  m_wpt_m40_y = get<1>(m40WPTGeoData);
  m_wpt_magic_x = get<0>(magicWPTGeoData);
  m_wpt_magic_y = get<1>(magicWPTGeoData);

  m_track_outter_x = get<0>(outterTrackGeoData);
  m_track_outter_y = get<1>(outterTrackGeoData);

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

  std::cout << "Bank and Z map resolution : " << m_lookup_grid_res << " m"
            << std::endl;
  getPreviouslyCalculatedBankNZmapFromCSV();
}

void c_terrain_manager::setTrackArcLengthSpline(
    const mpcc::ArcLengthSpline &track) {
  track_ = track;
}

double c_terrain_manager::getSectionPointProgress(double x, double y) {
  mpcc::State x_section_point;
  x_section_point.X = x;
  x_section_point.Y = y;
  double progress = track_.projectOnSpline(x_section_point);
  return progress;
}

double c_terrain_manager::getTrackBoundaryModulation(double target_x,
                                                     double target_y) {
  double amount_modulation = 0.0; // delta

  int trackSection = -1;
  double headingTo_x;
  double headingTo_y;
  double dist;

  double modulation_max = 6.2;
  // double lookAhead_dist = 60.0;
  double lookAhead_dist = 45.0;

  enum TrackSection {
    in_right_straight,
    in_turn1,
    in_top_straight,
    in_turn2,
    in_left_straight,
    in_turn3,
    in_bottom_straight,
    in_turn4,
  };

  if (target_y >= rightBot_longStraight_y_min &&
      target_y <= rightTop_longStraight_y_max && target_x >= turn1_x_min) {
    trackSection = in_right_straight;
    headingTo_x = turn1_in_x;
    headingTo_y = turn1_in_y;
  } else if (target_y >= turn1_y_min && target_y <= turn1_y_max &&
             target_x >= turn1_x_min && target_x <= turn1_x_max) { // In turn 1
    trackSection = in_turn1;
    headingTo_x = turn1_out_x;
    headingTo_y = turn1_out_y;
  } else if (target_x >= turn2_x_max && target_x <= turn1_x_min &&
             target_y >= turn1_y_min) { // In top straight
    trackSection = in_top_straight;
    headingTo_x = turn2_in_x;
    headingTo_y = turn2_in_y;
  } else if (target_y >= turn2_y_min && target_y <= turn2_y_max &&
             target_x >= turn2_x_min && target_x <= turn2_x_max) { // In turn 2
    trackSection = in_turn2;
    headingTo_x = turn2_out_x;
    headingTo_y = turn2_out_y;
  } else if (target_y >= leftBot_longStraight_y_min &&
             target_y <= leftTop_longStraight_y_max &&
             target_x <= turn3_x_max) {
    trackSection = in_left_straight;
    headingTo_x = turn3_in_x;
    headingTo_y = turn3_in_y;
  } else if (target_y >= turn3_y_min && target_y <= turn3_y_max &&
             target_x >= turn3_x_min && target_x <= turn3_x_max) { // In turn 3
    trackSection = in_turn3;
    headingTo_x = turn3_out_x;
    headingTo_y = turn3_out_y;
  } else if (target_x >= turn3_x_max && target_x <= turn4_x_min &&
             target_y <= turn4_y_max) { // In bottom straight
    trackSection = in_bottom_straight;
    headingTo_x = turn4_in_x;
    headingTo_y = turn4_in_y;
  } else if (target_y >= turn4_y_min && target_y <= turn4_y_max &&
             target_x >= turn4_x_min && target_x <= turn4_x_max) { // In turn 4
    trackSection = in_turn4;
    headingTo_x = turn4_out_x;
    headingTo_y = turn4_out_y;
  }

  // Update m_trackSection_global for checking initial race position
  m_trackSection_global = trackSection;

  // dist = sqrt(pow(target_x-headingTo_x,2)+pow(target_y-headingTo_y,2));

  // Compare progress between current target pose and section waypoint
  double headingTo_progress = getSectionPointProgress(headingTo_x, headingTo_y);
  double target_progress = getSectionPointProgress(target_x, target_y);

  dist = headingTo_progress - target_progress;
  if (dist < -1500) {
    dist += track_.total_progress_;
  }

  if (dist < lookAhead_dist) {
    if (trackSection == in_top_straight || trackSection == in_left_straight ||
        trackSection == in_bottom_straight ||
        trackSection == in_right_straight) {
      // increasing modulation
      return modulation_max * (1 - (dist / lookAhead_dist));
    } else {
      // decreasing modulation
      return modulation_max * ((dist / lookAhead_dist));
    }
  } else {
    if (trackSection == in_top_straight || trackSection == in_left_straight ||
        trackSection == in_bottom_straight ||
        trackSection == in_right_straight) {
      return 0.0;
    } else {
      return modulation_max;
    }
  }
}

bool c_terrain_manager::getTrackRegion(double target_x, double target_y) {
  // return true : in straight (includes short straight)
  // return false : in curve

  if (target_y >= rightBot_longStraight_y_min &&
      target_y <= rightTop_longStraight_y_max) {
    return true;
  } else if (target_y >= leftBot_longStraight_y_min &&
             target_y <= leftTop_longStraight_y_max) {
    return true;
  } else if (target_y >= turn1_y_min && target_y <= turn1_y_max &&
             target_x >= turn1_x_min && target_x <= turn1_x_max) { // In turn 1
    return false;
  } else if (target_y >= turn2_y_min && target_y <= turn2_y_max &&
             target_x >= turn2_x_min && target_x <= turn2_x_max) { // In turn 2
    return false;
  } else if (target_y >= turn3_y_min && target_y <= turn1_y_max &&
             target_x >= turn3_x_min && target_x <= turn1_x_max) { // In turn 3
    return false;
  } else if (target_y >= turn4_y_min && target_y <= turn1_y_max &&
             target_x >= turn4_x_min && target_x <= turn1_x_max) { // In turn 4
    return false;
  } else {
    return true;
  }
}

double c_terrain_manager::getBankAngle_rad(double target_x, double target_y) {

  if (m_setBankMap_flg) {
    int grid_x = int((target_x - m_track_outter_min_x) / m_lookup_grid_res);
    int grid_y = int((target_y - m_track_outter_min_y) / m_lookup_grid_res);

    if (grid_y >= m_mapSize_col || grid_x >= m_mapSize_row || grid_y < 0 ||
        grid_x < 0) {
      // std::cout << " GRID SIZE AND DISCRETIZATION ERROR...RETURN ZERO"
      //           << std::endl;
      // std::cout << " preCalculated grid map size x : " << m_mapSize_row  <<
      // std::endl; std::cout << " preCalculated grid map size y : " <<
      // m_mapSize_col  << std::endl;

      // std::cout << " access grid map loc x : " << grid_x  << std::endl;
      // std::cout << " access grid map loc y : " << grid_y  << std::endl;

      // std::cout << " access target x : " << target_x  << std::endl;
      // std::cout << " access target y : " << target_y  << std::endl;

      return 0;
    } else {
      return m_terrain_bankAngle_map[grid_x][grid_y];
    }
  } else {
    std::cout << "Bank map is not loaded yet...Load it first" << std::endl;
    return 0;
  }
}

double c_terrain_manager::getZ_meter(double target_x, double target_y) {

  if (m_setZMap_flg) {
    int grid_x = int((target_x - m_track_outter_min_x) / m_lookup_grid_res);
    int grid_y = int((target_y - m_track_outter_min_y) / m_lookup_grid_res);

    if (grid_y >= m_mapSize_col || grid_x >= m_mapSize_row) {
      std::cout << " GRID SIZE AND DISCRETIZATION ERROR...RETURN ZERO"
                << std::endl;
      return 0;
    } else {
      return m_terrain_z_map[grid_x][grid_y];
    }
  } else {
    std::cout << "Z map is not loaded yet...Load it first" << std::endl;
    return 0;
  }
}

void c_terrain_manager::getPreviouslyCalculatedBankNZmapFromCSV() {

  //   read from file
  std::ifstream file_z(m_zMap_file_path);
  std::vector<std::vector<double>> matrix_z;
  std::vector<double> row_z;
  std::string line_z;
  std::string cell_z;

  while (file_z) {
    std::getline(file_z, line_z);
    std::stringstream lineStream(line_z);
    row_z.clear();
    while (std::getline(lineStream, cell_z, ',')) {
      row_z.push_back(stod(cell_z));
    }
    if (!row_z.empty())
      matrix_z.push_back(row_z);
  }
  m_terrain_z_map = matrix_z;

  std::ifstream file_b(m_bankMap_file_path);
  std::vector<std::vector<double>> matrix_b;
  std::vector<double> row_b;
  std::string line_b;
  std::string cell_b;

  while (file_b) {
    std::getline(file_b, line_b);
    std::stringstream lineStream(line_b);
    row_b.clear();
    while (std::getline(lineStream, cell_b, ',')) {
      row_b.push_back(stod(cell_b));
    }
    if (!row_b.empty())
      matrix_b.push_back(row_b);
  }
  m_terrain_bankAngle_map = matrix_b;

  m_mapSize_row = m_terrain_bankAngle_map.size();
  m_mapSize_col = m_terrain_bankAngle_map[0].size();

  m_setBankMap_flg = true;
  m_setZMap_flg = true;
}

int c_terrain_manager::checkInitialRaceline(double ego_x, double ego_y,
                                            double ego_v, double sim_time,
                                            mpcc::State x_oppo) {
  // Check initial race position & Decide initial raceline
  int initial_raceline = 0; // 0: not in initial pose, 
                            // 1: initial left racingline,
                            // 2: initial right racingline
                            // 3: raceline m25
                            // 4: raceline m40
  double track_width = 17.0;

  // Update m_trackSection_global ()
  // 0: in_right_straight
  // 1: in_turn1
  // 2: in_top_straight
  // 3: in_turn2
  // 4: in_left_straight
  // 5: in_turn3
  // 6: in_bottom_straight
  // 7: in_turn4
  getTrackBoundaryModulation(ego_x, ego_y);

  // check initial raceline
  if (!m_done_init_raceline_check) {
    if (sim_time < m_initial_race_time) {
      if (m_trackSection_global == 4) {
        // check left/right initial race position
        double start_center_x =
            (m_start_center_bot_x - m_start_center_up_x) /
                (m_start_center_bot_y - m_start_center_up_y) *
                (ego_y - m_start_center_up_y) +
            m_start_center_up_x;
        if (ego_x > start_center_x) {
          // in initial left race position
          initial_raceline = 1;
        } else {
          // in initial right race position
          initial_raceline = 2;
        }
        // TODO: consider adding a new case for initial_raceline = 0 w.r.t. dist
        // to each raceline
      }
      else {
        // not in initial race line
        initial_raceline = m_initial_raceline;
      }
      m_initial_raceline = initial_raceline;
    }

    // update to original raceline when arrive in>out phase of Turn4.
    // if (m_trackSection_global == 7)
    if (ego_x > turn4_raceline_update_x_min &&
        ego_x < turn4_raceline_update_x_max &&
        ego_y > turn4_raceline_update_y_min &&
        ego_y < turn4_raceline_update_y_max) {
      initial_raceline = 0;
      m_initial_raceline = initial_raceline;
      m_done_init_raceline_check = true;
    }
  }
  
  // check decision region (after finishing initial raceline checking)
  else {
    curr_decision_flag = checkRaceLineDecisionRegion(ego_x, ego_y);
    curr_comeback_flag = checkRaceLineComebackRegion(ego_x, ego_y);

    // when inside of the decision region
    if (curr_decision_flag == false && last_decision_flag == true){
      if (x_oppo.X * x_oppo.Y == 0.0){
        initial_raceline = 0;
      }
      else{
        // Check distance to oppo
        double dist_to_oppo = sqrt(pow(x_oppo.X - ego_x, 2.0) + pow(x_oppo.Y - ego_y, 2.0));

        double dist_btw_oppo_m25  = 100000.0;
        double dist_btw_oppo_m40  = 100000.0;
        double dist_btw_oppo_magic = 100000.0;
        double dist_btw_ego_m25   = 100000.0;
        double dist_btw_ego_m40   = 100000.0;
        double dist_btw_ego_magic = 100000.0;

        std::vector<double> dist_oppo_vec_tmp;
        // std::cout << "here wpt size m_wpt_m25_x : " << m_wpt_m25_x.size() << std::endl;
        // std::cout << "here wpt size m_wpt_m40_x : " << m_wpt_m40_x.size() << std::endl;
        // std::cout << "here wpt size m_wpt_magic_x : " << m_wpt_magic_x.size() << std::endl;
        dist_btw_oppo_m25 = calc_min_dist(x_oppo.X, x_oppo.Y, m_wpt_m25_x, m_wpt_m25_y);
        dist_btw_oppo_m40 = calc_min_dist(x_oppo.X, x_oppo.Y, m_wpt_m40_x, m_wpt_m40_y);
        dist_btw_oppo_magic = calc_min_dist(x_oppo.X, x_oppo.Y, m_wpt_magic_x, m_wpt_magic_y);

        dist_oppo_vec_tmp.push_back(dist_btw_oppo_magic);
        dist_oppo_vec_tmp.push_back(100000.0);
        dist_oppo_vec_tmp.push_back(100000.0);
        dist_oppo_vec_tmp.push_back(dist_btw_oppo_m25);
        dist_oppo_vec_tmp.push_back(dist_btw_oppo_m40);

        int raceLine_idx_close_oppo = std::min_element(dist_oppo_vec_tmp.begin(),dist_oppo_vec_tmp.end()) - dist_oppo_vec_tmp.begin();

        std::vector<double> dist_ego_vec_tmp;
        dist_btw_ego_m25 = calc_min_dist(ego_x, ego_y, m_wpt_m25_x, m_wpt_m25_y);
        dist_btw_ego_m40 = calc_min_dist(ego_x, ego_y, m_wpt_m40_x, m_wpt_m40_y);
        dist_btw_ego_magic = calc_min_dist(ego_x, ego_y, m_wpt_magic_x, m_wpt_magic_y);

        dist_ego_vec_tmp.push_back(dist_btw_ego_magic);
        dist_ego_vec_tmp.push_back(100000.0);
        dist_ego_vec_tmp.push_back(100000.0);
        dist_ego_vec_tmp.push_back(dist_btw_ego_m25);
        dist_ego_vec_tmp.push_back(dist_btw_ego_m40);
        
        if (dist_to_oppo > 50.0){
          // follow raceline magic
          initial_raceline = 0;
        }
        else if (dist_to_oppo > 20 && dist_to_oppo <= 50.0){
          // follow raceline which is the closest path with oppo
          initial_raceline = dist_oppo_vec_tmp[raceLine_idx_close_oppo];
        }
        else {
          // Find the closest path with ego except the path closest with oppo
          dist_ego_vec_tmp[raceLine_idx_close_oppo] = 100000.0;
          double raceLine_idx_close_ego = std::min_element(dist_ego_vec_tmp.begin(),dist_ego_vec_tmp.end()) - dist_ego_vec_tmp.begin();
          initial_raceline = dist_ego_vec_tmp[raceLine_idx_close_ego];
        }
      }

      std::string raceLine_name = "";
      if (m_initial_raceline==0){
        raceLine_name = "Original";
      }
      else if(m_initial_raceline==3){
        raceLine_name = "M25";
      }
      else if(m_initial_raceline==4){
        raceLine_name = "M40";
      }
      else{
        raceLine_name = "Don't care";
      }
      
      std::cout << "TURN TO STRAIGHT : Raceline Decision update before : " << raceLine_name << std::endl;
      
      m_initial_raceline = initial_raceline;

      if (m_initial_raceline==0){
        raceLine_name = "Original";
      }
      else if(m_initial_raceline==3){
        raceLine_name = "M25";
      }
      else if(m_initial_raceline==4){
        raceLine_name = "M40";
      }
      else{
        raceLine_name = "Don't care";
      }
      
      std::cout << "TURN TO STRAIGHT : Raceline Decision update after  : " << raceLine_name << std::endl;
    }
    // when outside of the decision region
    else{
      // No raceline update 
    }

    // when inside of the comeback region
    if (curr_comeback_flag == false && last_comeback_flag == true){
      if (initial_raceline == 4){
        initial_raceline = 3;
      }
      else{
        // no raceline comeback
      }

      std::string raceLine_name = "";
      if (m_initial_raceline==0){
        raceLine_name = "Original";
      }
      else if(m_initial_raceline==3){
        raceLine_name = "M25";
      }
      else if(m_initial_raceline==4){
        raceLine_name = "M40";
      }
      else{
        raceLine_name = "Don't care";
      }
      
      std::cout << "STRAIGHT TO TURN : Raceline Comeback update before : " << raceLine_name << std::endl;
      
      m_initial_raceline = initial_raceline;

      if (m_initial_raceline==0){
        raceLine_name = "Original";
      }
      else if(m_initial_raceline==3){
        raceLine_name = "M25";
      }
      else if(m_initial_raceline==4){
        raceLine_name = "M40";
      }
      else{
        raceLine_name = "Don't care";
      }
      
      std::cout << "STRAIGHT TO TURN : Raceline Comeback update after  : " << raceLine_name << std::endl;


    }
    // when outside of the comback region
    else{
      // No raceline update
    }

    // update last decision and comback flags
    last_decision_flag = curr_decision_flag;
    last_comeback_flag = curr_comeback_flag;

    bool is_raceline_change_region = ((ego_x > rightStraightRacelineChange_x_min &&
                                      ego_x < rightStraightRacelineChange_x_max &&
                                      ego_y > rightStraightRacelineChange_y_min &&
                                      ego_y < rightStraightRacelineChange_y_max) ||
                                      (ego_x > leftStraightRacelineChange_x_min &&
                                      ego_x < leftStraightRacelineChange_x_max &&
                                      ego_y > leftStraightRacelineChange_y_min &&
                                      ego_y < leftStraightRacelineChange_y_max));
    // when oppo is too close && slower than ego (2km/h) && is_straight && 1 second past last raceline change
    if (is_raceline_change_region && m_raceline_change_count > 10) {
      double oppo_dist = sqrt(pow(x_oppo.X-ego_x, 2.0) + pow(x_oppo.Y-ego_y, 2.0));
      double oppo_rel_v = x_oppo.vx - ego_v;
      bool is_straight = (m_trackSection_global == 0 || m_trackSection_global == 4);

      if (oppo_dist < raceline_change_dist && oppo_rel_v < -raceline_change_rel_v && is_straight){
        // find a path close with oppo
        double dist_btw_oppo_m25  = 100000.0;
        double dist_btw_oppo_m40  = 100000.0;
        double dist_btw_oppo_magic = 100000.0;

        std::vector<double> dist_oppo_vec_tmp;
        dist_btw_oppo_m25 = calc_min_dist(x_oppo.X, x_oppo.Y, m_wpt_m25_x, m_wpt_m25_y);
        dist_btw_oppo_m40 = calc_min_dist(x_oppo.X, x_oppo.Y, m_wpt_m40_x, m_wpt_m40_y);
        dist_btw_oppo_magic = calc_min_dist(x_oppo.X, x_oppo.Y, m_wpt_magic_x, m_wpt_magic_y);

        dist_oppo_vec_tmp.push_back(dist_btw_oppo_magic);
        dist_oppo_vec_tmp.push_back(100000.0);
        dist_oppo_vec_tmp.push_back(100000.0);
        dist_oppo_vec_tmp.push_back(dist_btw_oppo_m25);
        dist_oppo_vec_tmp.push_back(dist_btw_oppo_m40);

        int raceLine_idx_close_oppo = std::min_element(dist_oppo_vec_tmp.begin(),dist_oppo_vec_tmp.end()) - dist_oppo_vec_tmp.begin();

        // choose raceline where no oppo && close to ego
        double dist_btw_ego_m25  = 100000.0;
        double dist_btw_ego_m40  = 100000.0;
        double dist_btw_ego_magic  = 100000.0;
        std::vector<double> dist_ego_vec_tmp;
        dist_btw_ego_m25 = calc_min_dist(ego_x, ego_y, m_wpt_m25_x, m_wpt_m25_y);
        dist_btw_ego_m40 = calc_min_dist(ego_x, ego_y, m_wpt_m40_x, m_wpt_m40_y);
        dist_btw_ego_magic = calc_min_dist(ego_x, ego_y, m_wpt_magic_x, m_wpt_magic_y);

        dist_ego_vec_tmp.push_back(dist_btw_ego_magic);
        dist_ego_vec_tmp.push_back(100000.0);
        dist_ego_vec_tmp.push_back(100000.0);
        dist_ego_vec_tmp.push_back(dist_btw_ego_m25);
        dist_ego_vec_tmp.push_back(dist_btw_ego_m40);
        
        dist_ego_vec_tmp[raceLine_idx_close_oppo] = 100000.0; // do not choose oppo raceline

        int raceLine_idx_close_ego = std::min_element(dist_ego_vec_tmp.begin(),dist_ego_vec_tmp.end()) - dist_ego_vec_tmp.begin();
        initial_raceline = raceLine_idx_close_ego;
        m_initial_raceline = initial_raceline;
        m_raceline_change_count = 0;
      }
    }
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    std::cout << "RACELINE CHANGE from: " << m_initial_raceline << " to: " << initial_raceline << " count: " << m_raceline_change_count << std::endl;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    m_raceline_change_count = std::min(500, m_raceline_change_count+1);
  }
  return m_initial_raceline;
}

double c_terrain_manager::calc_min_dist(double x, double y, std::vector<double> wpt_x, std::vector<double> wpt_y) {
  double min_dist = 100000.0;
  for (int wpt_idx = 0;  wpt_idx < wpt_x.size(); wpt_idx++){
    double dist = sqrt(pow(x - wpt_x[wpt_idx], 2.0) + pow(y - wpt_y[wpt_idx], 2.0));
    if(dist < min_dist){
      min_dist = dist;
    }
  }
  return min_dist;
}

bool c_terrain_manager::checkRaceLineDecisionRegion(double ego_x, double ego_y){
    // Region (in Turn 4 -> Right Straight) or (in Region in Turn 2 -> Left Straight)
    if ((ego_x > turn4TorightStraight_x_min &&
        ego_x < turn4TorightStraight_x_max &&
        ego_y > turn4TorightStraight_y_min &&
        ego_y < turn4TorightStraight_y_max) ||
        (ego_x > turn2ToleftStraight_x_min &&
        ego_x < turn2ToleftStraight_x_max &&
        ego_y > turn2ToleftStraight_y_min &&
        ego_y < turn2ToleftStraight_y_max) 
        ) {
          return true;
    }
    else{
      return false;
    }
}

bool c_terrain_manager::checkRaceLineComebackRegion(double ego_x, double ego_y){
    // Region (in Right Straight -> Turn 1) or (in Left Straight -> Turn 2)
    if ((ego_x > rightStraightToTurn1_x_min &&
        ego_x < rightStraightToTurn1_x_max &&
        ego_y > rightStraightToTurn1_y_min &&
        ego_y < rightStraightToTurn1_y_max) ||
        (ego_x > leftStraightToTurn3_x_min &&
        ego_x < leftStraightToTurn3_x_max &&
        ego_y > leftStraightToTurn3_y_min &&
        ego_y < leftStraightToTurn3_y_max) 
        ) {
          return true;
    }
    else{
      return false;
    }
}

tuple<vector<double>, vector<double>, vector<double>>
c_terrain_manager::getGeoDataFromCSV(string file_path) {
  vector<double> track_x;
  vector<double> track_y;
  vector<double> track_z;
  //   read from file
  std::ifstream file(file_path);
  std::vector<std::vector<std::string>> matrix;
  std::vector<std::string> row;
  std::string line;
  std::string cell;

  int cnt = 0;
  while (file) {
    if (cnt % 10 == 0) {
      std::getline(file, line);
      std::stringstream lineStream(line);
      row.clear();

      while (std::getline(lineStream, cell, ';')) {
        row.push_back(cell);
        //   cout << "cell : " << cell << endl;
      }

      if (!row.empty())
        matrix.push_back(row);
    }
    cnt++;
  }

  for (int i = 2; i < int(matrix.size()) - 10; i++) {
    track_x.push_back(stod(matrix[i][1]));
    track_y.push_back(stod(matrix[i][2]));
    track_z.push_back(stod(matrix[i][3]));
  }

  return make_tuple(track_x, track_y, track_z);
}

tuple<vector<double>, vector<double>>
c_terrain_manager::getGeoDataFromCSV_onlyWPT(string file_path) {
  vector<double> track_x;
  vector<double> track_y;
  //   read from file
  std::ifstream file(file_path);
  std::vector<std::vector<std::string>> matrix;
  std::vector<std::string> row;
  std::string line;
  std::string cell;

  int cnt = 0;
  while (file) {
    if (cnt % 10 == 0) {
      std::getline(file, line);
      std::stringstream lineStream(line);
      row.clear();

      while (std::getline(lineStream, cell, ';')) {
        row.push_back(cell);
        //   cout << "cell : " << cell << endl;
      }

      if (!row.empty())
        matrix.push_back(row);
    }
    cnt++;
  }

  for (int i = 0; i < int(matrix.size()) - 10; i++) {
    track_x.push_back(stod(matrix[i][0]));
    track_y.push_back(stod(matrix[i][1]));
  }

  return make_tuple(track_x, track_y);
}

c_terrain_manager::c_terrain_manager(string track_inner_file_path,
                                     string track_outter_file_path) {

  cout << "Constructor(with file path) init" << endl;
  setTrackFilePaths(track_inner_file_path, track_outter_file_path);

  //   cout << "file path 1 : " << track_inner_file_path << endl;
  //   cout << "file path 2 : " << track_outter_file_path << endl;
  auto innerTrackGeoData = getGeoDataFromCSV(track_inner_file_path);
  auto outterTrackGeoData = getGeoDataFromCSV(track_outter_file_path);

  setTrack(get<0>(innerTrackGeoData), get<1>(innerTrackGeoData),
           get<2>(innerTrackGeoData), get<0>(outterTrackGeoData),
           get<1>(outterTrackGeoData), get<2>(outterTrackGeoData));
}
c_terrain_manager::c_terrain_manager(vector<double> track_inner_x,
                                     vector<double> track_inner_y,
                                     vector<double> track_inner_z,
                                     vector<double> track_outter_x,
                                     vector<double> track_outter_y,
                                     vector<double> track_outter_z) {
  cout << "Constructor(with terrain data) init" << endl;

  setTrack(track_inner_x, track_inner_y, track_inner_z, track_outter_x,
           track_outter_y, track_outter_z);
}

void c_terrain_manager::setTrack(vector<double> track_inner_x,
                                 vector<double> track_inner_y,
                                 vector<double> track_inner_z,
                                 vector<double> track_outter_x,
                                 vector<double> track_outter_y,
                                 vector<double> track_outter_z) {


  m_track_inner_x = track_inner_x;
  m_track_inner_y = track_inner_y;
  m_track_inner_z = track_inner_z;
  m_track_outter_x = track_outter_x;
  m_track_outter_y = track_outter_y;
  m_track_outter_z = track_outter_z;

  m_setTrack_flg = true;

  vector<double> track_mid_v1_x;
  vector<double> track_mid_v1_y;
  vector<double> track_mid_v1_z;
  vector<double> track_mid_v2_x;
  vector<double> track_mid_v2_y;
  vector<double> track_mid_v2_z;

  string track_mid_v1_filePath =
      "/home/usrg/ros2_ws/src/mpcc_ros2/lib/iac_terrain_manager/terrain_data/"
      "VehiclePlayerExport_middle_path_dataLog_v1.csv";
  string track_mid_v2_filePath =
      "/home/usrg/ros2_ws/src/mpcc_ros2/lib/iac_terrain_manager/terrain_data/"
      "VehiclePlayerExport_middle_path_dataLog_v2.csv";

  auto v1_TrackGeoData = getGeoDataFromCSV(track_mid_v1_filePath);
  auto v2_TrackGeoData = getGeoDataFromCSV(track_mid_v2_filePath);

  track_mid_v1_x = get<0>(v1_TrackGeoData);
  track_mid_v1_y = get<1>(v1_TrackGeoData);
  track_mid_v1_z = get<2>(v1_TrackGeoData);
  track_mid_v2_x = get<0>(v2_TrackGeoData);
  track_mid_v2_y = get<1>(v2_TrackGeoData);
  track_mid_v2_z = get<2>(v2_TrackGeoData);

  setTrack_minMax_xy();
  m_mapSize_row =
      int((m_track_outter_max_y - m_track_outter_min_y) / m_lookup_grid_res) +
      1;
  m_mapSize_col =
      int((m_track_outter_max_x - m_track_outter_min_x) / m_lookup_grid_res) +
      1;

  m_terrain_z_map.resize(m_mapSize_row);
  m_terrain_bankAngle_map.resize(m_mapSize_row);

  for (int t = 0; t < m_mapSize_row; t++) {
    m_terrain_z_map[t].resize(m_mapSize_col);
    m_terrain_bankAngle_map[t].resize(m_mapSize_col);
  }

  //   skip splining & make lookup table at the first time
  for (int i = 0; i < m_mapSize_row; i++) {
    for (int j = 0; j < m_mapSize_col; j++) {
      double x = i * m_lookup_grid_res + m_track_outter_min_x;
      double y = j * m_lookup_grid_res + m_track_outter_min_y;
      double x_randAdded_1 =
          x + (((rand() % 50) - 25) * m_lookup_grid_res) / 50.0;
      double x_randAdded_2 =
          x + (((rand() % 60) - 30) * m_lookup_grid_res) / 60.0;
      double y_randAdded_1 =
          y + (((rand() % 50) - 25) * m_lookup_grid_res) / 50.0;
      double y_randAdded_2 =
          y + (((rand() % 60) - 30) * m_lookup_grid_res) / 60.0;

      vector<double> dist_to_inner_vec;
      vector<double> dist_to_outter_vec;
      vector<double> dist_to_mid_v1_vec;
      vector<double> dist_to_mid_v2_vec;

      int closest_idx_to_inner = 0;
      int closest_idx_to_outter = 0;
      int closest_idx_to_mid_v1 = 0;
      int closest_idx_to_mid_v2 = 0;
      double closest_dist_to_inner = 0.0;
      double closest_dist_to_outter = 0.0;
      double closest_dist_to_mid_v1 = 0.0;
      double closest_dist_to_mid_v2 = 0.0;

      //   inner
      for (int in_idx = 0; in_idx < m_track_inner_x.size(); in_idx++) {

        double dist_to_inner = sqrt(pow(x - m_track_inner_x[in_idx], 2) +
                                    pow(y - m_track_inner_y[in_idx], 2));
        dist_to_inner_vec.push_back(dist_to_inner);
      }
      //   outter
      for (int out_idx = 0; out_idx < m_track_outter_x.size(); out_idx++) {
        double dist_to_outter = sqrt(pow(x - m_track_outter_x[out_idx], 2) +
                                     pow(y - m_track_outter_y[out_idx], 2));
        dist_to_outter_vec.push_back(dist_to_outter);
      }
      //   mid v1
      for (int mid_v1_idx = 0; mid_v1_idx < track_mid_v1_x.size();
           mid_v1_idx++) {
        double dist_to_mid_v1 =
            sqrt(pow(x_randAdded_1 - track_mid_v1_x[mid_v1_idx], 2) +
                 pow(y_randAdded_1 - track_mid_v1_y[mid_v1_idx], 2));
        dist_to_mid_v1_vec.push_back(dist_to_mid_v1);
      }
      //   mid v2
      for (int mid_v2_idx = 0; mid_v2_idx < track_mid_v2_x.size();
           mid_v2_idx++) {
        double dist_to_mid_v2 =
            sqrt(pow(x_randAdded_2 - track_mid_v2_x[mid_v2_idx], 2) +
                 pow(y_randAdded_2 - track_mid_v2_y[mid_v2_idx], 2));
        dist_to_mid_v2_vec.push_back(dist_to_mid_v2);
      }

      closest_dist_to_inner =
          *min_element(dist_to_inner_vec.begin(), dist_to_inner_vec.end());
      closest_dist_to_outter =
          *min_element(dist_to_outter_vec.begin(), dist_to_outter_vec.end());
      closest_dist_to_mid_v1 =
          *min_element(dist_to_mid_v1_vec.begin(), dist_to_mid_v1_vec.end());
      closest_dist_to_mid_v2 =
          *min_element(dist_to_mid_v2_vec.begin(), dist_to_mid_v2_vec.end());

      closest_idx_to_inner =
          min_element(dist_to_inner_vec.begin(), dist_to_inner_vec.end()) -
          dist_to_inner_vec.begin();
      closest_idx_to_outter =
          min_element(dist_to_outter_vec.begin(), dist_to_outter_vec.end()) -
          dist_to_outter_vec.begin();
      closest_idx_to_mid_v1 =
          min_element(dist_to_mid_v1_vec.begin(), dist_to_mid_v1_vec.end()) -
          dist_to_mid_v1_vec.begin();
      closest_idx_to_mid_v2 =
          min_element(dist_to_mid_v2_vec.begin(), dist_to_mid_v2_vec.end()) -
          dist_to_mid_v2_vec.begin();

      float plane_coeff_a = 0.0;
      float plane_coeff_b = 0.0;
      float plane_coeff_c = 0.0;
      float plane_coeff_d = 0.0;

      double grid_z = 0.0;

      if (closest_dist_to_inner < closest_dist_to_outter) {

        double pt1_x = m_track_inner_x[closest_idx_to_inner];
        double pt1_y = m_track_inner_y[closest_idx_to_inner];
        double pt1_z = m_track_inner_z[closest_idx_to_inner];

        double pt2_x = track_mid_v1_x[closest_idx_to_mid_v1];
        double pt2_y = track_mid_v1_y[closest_idx_to_mid_v1];
        double pt2_z = track_mid_v1_z[closest_idx_to_mid_v1];

        double pt3_x = track_mid_v2_x[closest_idx_to_mid_v2];
        double pt3_y = track_mid_v2_y[closest_idx_to_mid_v2];
        double pt3_z = track_mid_v2_z[closest_idx_to_mid_v2];

        vector<double> dist_vec, z_vec;
        dist_vec.push_back(sqrt(pow(x - pt1_x, 2) + pow(y - pt1_y, 2)));
        dist_vec.push_back(sqrt(pow(x - pt2_x, 2) + pow(y - pt2_y, 2)));
        dist_vec.push_back(sqrt(pow(x - pt3_x, 2) + pow(y - pt3_y, 2)));
        z_vec.push_back(pt1_z);
        z_vec.push_back(pt2_z);
        z_vec.push_back(pt3_z);

        int min_idx =
            min_element(dist_vec.begin(), dist_vec.end()) - dist_vec.begin();
        grid_z = z_vec[min_idx];

        float a1 = pt2_x - pt1_x;
        float b1 = pt2_y - pt1_y;
        float c1 = pt2_z - pt1_z;
        float a2 = pt3_x - pt1_x;
        float b2 = pt3_y - pt1_y;
        float c2 = pt3_z - pt1_z;
        plane_coeff_a = b1 * c2 - b2 * c1;
        plane_coeff_b = a2 * c1 - a1 * c2;
        plane_coeff_c = a1 * b2 - b1 * a2;
        plane_coeff_d = (-plane_coeff_a * pt1_x - plane_coeff_b * pt1_y -
                         plane_coeff_c * pt1_z);
        // equation of plane : ax + by + cz + d = 0
        // normal vector of plane : (a,b,c)
      } else {
        double pt1_x = m_track_outter_x[closest_idx_to_outter];
        double pt1_y = m_track_outter_y[closest_idx_to_outter];
        double pt1_z = m_track_outter_z[closest_idx_to_outter];

        double pt2_x = track_mid_v1_x[closest_idx_to_mid_v1];
        double pt2_y = track_mid_v1_y[closest_idx_to_mid_v1];
        double pt2_z = track_mid_v1_z[closest_idx_to_mid_v1];

        double pt3_x = track_mid_v2_x[closest_idx_to_mid_v2];
        double pt3_y = track_mid_v2_y[closest_idx_to_mid_v2];
        double pt3_z = track_mid_v2_z[closest_idx_to_mid_v2];

        vector<double> dist_vec, z_vec;
        dist_vec.push_back(sqrt(pow(x - pt1_x, 2) + pow(y - pt1_y, 2)));
        dist_vec.push_back(sqrt(pow(x - pt2_x, 2) + pow(y - pt2_y, 2)));
        dist_vec.push_back(sqrt(pow(x - pt3_x, 2) + pow(y - pt3_y, 2)));
        z_vec.push_back(pt1_z);
        z_vec.push_back(pt2_z);
        z_vec.push_back(pt3_z);

        int min_idx =
            min_element(dist_vec.begin(), dist_vec.end()) - dist_vec.begin();
        grid_z = z_vec[min_idx];

        float a1 = pt2_x - pt1_x;
        float b1 = pt2_y - pt1_y;
        float c1 = pt2_z - pt1_z;
        float a2 = pt3_x - pt1_x;
        float b2 = pt3_y - pt1_y;
        float c2 = pt3_z - pt1_z;
        plane_coeff_a = b1 * c2 - b2 * c1;
        plane_coeff_b = a2 * c1 - a1 * c2;
        plane_coeff_c = a1 * b2 - b1 * a2;
        plane_coeff_d = (-plane_coeff_a * pt1_x - plane_coeff_b * pt1_y -
                         plane_coeff_c * pt1_z);
        // equation of plane : ax + by + cz + d = 0
        // normal vector of plane : (a,b,c)
      }

      //   cout << "error" << endl;

      double normal_vec_mag =
          sqrt(plane_coeff_a * plane_coeff_a + plane_coeff_b * plane_coeff_b +
               plane_coeff_c * plane_coeff_c);
      double bankAngle_rad = acos(plane_coeff_c / normal_vec_mag);

      if (abs(3.14159265359 - bankAngle_rad) < bankAngle_rad) {
        bankAngle_rad = 3.14159265359 - bankAngle_rad;
      }
      //   cout << "error solved 1" << endl;

      m_terrain_z_map[i][j] = grid_z;
      m_terrain_bankAngle_map[i][j] = bankAngle_rad;

      //   cout << "error solved 2" << endl;
      // cout << "i : " << i << endl;
      // cout << "j : " << j << endl;
    }
    // cout << " m_mapSize_row : " << m_mapSize_row << " / " << i
    //      << endl;
    // cout << " m_mapSize_col : " << m_mapSize_col << " / " << i
    //      << endl;
    // cout << "hup" << std::endl;
  }
  // cout << "Done, save variable for testing" << endl;
  std::ofstream out_m_terrain_z_map("test_m_terrain_z_map.csv");
  std::ofstream out_m_terrain_bankAngle_map("test_m_terrain_bankAngle_map.csv");

  for (int i = 0; i < m_mapSize_row; i++) {
    for (int j = 0; j < m_mapSize_col; j++) {
      out_m_terrain_z_map << m_terrain_z_map[i][j] << ",";
      out_m_terrain_bankAngle_map << m_terrain_bankAngle_map[i][j] << ",";
    }
    out_m_terrain_z_map << '\n';
    out_m_terrain_bankAngle_map << '\n';
  }

  m_setBankMap_flg = true;
  m_setZMap_flg = true;
}

c_terrain_manager::~c_terrain_manager() {}

//////////////////////////////////////////////////////////
////////////////////// EXAMPLE USAGE /////////////////////
//////////////////////////////////////////////////////////

// 0. Include class into the ros node
// 1. Create object : c_terrain_manager obj = c_terrain_manager();
// 2. Pass global x and y [getBankAngle_rad / getZ_meter]