//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/28/21.
//

//
// Created by usrg on 6/27/21.
//

#include "nif_utils/utils.h"
#include "gtest/gtest.h"
#include <chrono>
#include "rclcpp/rclcpp.hpp"

TEST(UtilsTimeSecs, TimeDurationConversion) {
  {
    {
      int ns = 1000000000;
      auto res = nif::common::utils::time::secs(rclcpp::Time(ns));
      ASSERT_DOUBLE_EQ(res, 1.0);
    }

    {
      int s = 3;
      int ns = 500000000;
      auto res = nif::common::utils::time::secs(rclcpp::Time(s, ns));
      ASSERT_DOUBLE_EQ(res, 3.5);
    }

    {
      int s = 0;
      int ns = 500000000;
      auto res = nif::common::utils::time::secs(rclcpp::Duration(s, ns));
      ASSERT_DOUBLE_EQ(res, 0.5);
    }
  }
}