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
const double VELOCITY_ABS_ERROR = 0.0005;

class UtilsAlgebraCrossProductTest : public ::testing::Test {
protected:
  /**
   * Called by GTest at the beginning of the test
   */
  void SetUp() override {
    point_a.x = 2.0;
    point_a.y = 3.0;
    point_a.z = 4.0;

    point_b.x = 5.0;
    point_b.y = 6.0;
    point_b.z = 7.0;


    point_2d_a.x = 2.0;
    point_2d_a.y = 3.0;
    point_2d_a.z = 0.0;

    point_2d_b.x = 5.0;
    point_2d_b.y = 6.0;
    point_2d_b.z = 0.0;

  }

  /**
   * Called by GTest at the end of the test
   */
  void TearDown() override {}

  geometry_msgs::msg::Point point_a, point_b;
  geometry_msgs::msg::Point point_2d_a, point_2d_b;
};

TEST_F(UtilsAlgebraCrossProductTest, CrossProductVector) {
    const geometry_msgs::msg::Point& vector_c = nif::common::utils::algebra::calCrossProduct(
      point_a, point_b);
  ASSERT_EQ(vector_c.x, -3.0);
  ASSERT_EQ(vector_c.y, 6.0);
  ASSERT_EQ(vector_c.z, -3.0);
}

TEST_F(UtilsAlgebraCrossProductTest, CrossProductSign) {
    const geometry_msgs::msg::Point& vector_c = nif::common::utils::algebra::calCrossProduct(
      point_2d_a, point_2d_b);
  ASSERT_EQ(vector_c.x, 0.0);
  ASSERT_EQ(vector_c.y, 0.0);
  ASSERT_EQ(vector_c.z, -3.0);

  ASSERT_LT(vector_c.z, 0); // redundant
}

