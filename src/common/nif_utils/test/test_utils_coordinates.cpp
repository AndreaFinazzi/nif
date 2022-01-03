//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/27/21.
//

#include "nif_utils/utils.h"
#include "gtest/gtest.h"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

const double DISTANCE_ABS_ERROR = 0.0001;
const double ORIENTATION_ABS_ERROR = 0.000001;

using namespace nif::common::utils::coordination;

class UtilsCoordinatesTest : public ::testing::Test {
protected:
  /**
   * Called by GTest at the beginning of the test
   */
  void SetUp() override {
    odom_rotated.pose.pose.orientation.x = 0.0;
    odom_rotated.pose.pose.orientation.y = 0.0;
    odom_rotated.pose.pose.orientation.z = 0.3826834;
    odom_rotated.pose.pose.orientation.w = 0.9238795;

    pose_a_in_global.pose.position.x = 1;
    pose_a_in_global.pose.position.y = 1;

    pose_c_in_body.pose.position.x = 1;
    pose_c_in_body.pose.position.y = 1;

    odom_trans.pose.pose.position.x = -2.0;
    odom_trans.pose.pose.position.y = -2.0;
    odom_trans.pose.pose.position.z = 0.0;

    odom_trans_rotated.pose.pose.orientation = odom_rotated.pose.pose.orientation;
    odom_trans_rotated.pose.pose.position = odom_trans.pose.pose.position;

  }

  /**
   * Called by GTest at the end of the test
   */
  void TearDown() override {}

  nav_msgs::msg::Odometry odom_rotated, odom_trans, odom_trans_rotated;
  geometry_msgs::msg::PoseStamped pose_a_in_global, pose_b_in_global;
  geometry_msgs::msg::PoseStamped pose_c_in_body, pose_d_in_body;
};

TEST_F(UtilsCoordinatesTest, Rotation) {
  geometry_msgs::msg::PoseStamped pose_a_in_body = getPtGlobaltoBody(
    odom_rotated,
    pose_a_in_global
  );
  ASSERT_NEAR(pose_a_in_body.pose.position.x, sqrt(2), DISTANCE_ABS_ERROR);
  ASSERT_NEAR(pose_a_in_body.pose.position.y, 0.0, DISTANCE_ABS_ERROR);

  geometry_msgs::msg::PoseStamped pose_c_in_global = getPtBodytoGlobal(
    odom_rotated,
    pose_c_in_body
  );
  ASSERT_NEAR(pose_c_in_global.pose.position.x, 0.0, DISTANCE_ABS_ERROR);
  ASSERT_NEAR(pose_c_in_global.pose.position.y, sqrt(2), DISTANCE_ABS_ERROR);
}

TEST_F(UtilsCoordinatesTest, Translation) {

  geometry_msgs::msg::PoseStamped pose_a_in_body = getPtGlobaltoBody(
    odom_trans,
    pose_a_in_global
  );
  ASSERT_DOUBLE_EQ(pose_a_in_body.pose.position.x, 3.0);
  ASSERT_DOUBLE_EQ(pose_a_in_body.pose.position.y, 3.0);

  geometry_msgs::msg::PoseStamped pose_c_in_global = getPtBodytoGlobal(
    odom_trans,
    pose_c_in_body
  );
  ASSERT_DOUBLE_EQ(pose_c_in_global.pose.position.x, -1.0);
  ASSERT_DOUBLE_EQ(pose_c_in_global.pose.position.y, -1.0); 

}

TEST_F(UtilsCoordinatesTest, RotoTranslation) {

  geometry_msgs::msg::PoseStamped pose_a_in_body = getPtGlobaltoBody(
    odom_trans_rotated,
    pose_a_in_global
  );
  ASSERT_NEAR(pose_a_in_body.pose.position.x, 3.0 * sqrt(2.0), DISTANCE_ABS_ERROR);
  ASSERT_NEAR(pose_a_in_body.pose.position.y, 0.0, DISTANCE_ABS_ERROR);

  geometry_msgs::msg::PoseStamped pose_c_in_global = getPtBodytoGlobal(
    odom_trans_rotated,
    pose_c_in_body
  );
  ASSERT_NEAR(pose_c_in_global.pose.position.x, -2.0, DISTANCE_ABS_ERROR);
  ASSERT_NEAR(pose_c_in_global.pose.position.y, -2.0 + sqrt(2.0) , DISTANCE_ABS_ERROR); 
  ASSERT_NEAR(pose_c_in_global.pose.orientation.w, odom_trans_rotated.pose.pose.orientation.w, ORIENTATION_ABS_ERROR);
  ASSERT_NEAR(pose_c_in_global.pose.orientation.z, odom_trans_rotated.pose.pose.orientation.z, ORIENTATION_ABS_ERROR);
}
