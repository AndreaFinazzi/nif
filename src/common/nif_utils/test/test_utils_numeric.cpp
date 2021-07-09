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

TEST(UtilsNumericClipTest, ClipTestMin) {
  {
    double min = 0.1, max = 1.1, target = 0.005;
    auto res = nif::common::utils::numeric::clip<double>(min, max, target);
    ASSERT_DOUBLE_EQ(res, 0.1);
  }
}

TEST(UtilsNumericClipTest, ClipTestMax) {
  {
    double min = 0.1, max = 1.1, target = 0.005;
    auto res = nif::common::utils::numeric::clip<double>(min, max, target);
    ASSERT_DOUBLE_EQ(res, 1.1);
  }
}

TEST(UtilsNumericClipTest, ClipTestPass) {
  {
    {
      double min = 0.1, max = 1.1, target = 0.505;
      auto res = nif::common::utils::numeric::clip<double>(min, max, target);
      ASSERT_DOUBLE_EQ(res, 0.505);
    }

    {
      const std::chrono::microseconds min(1000), max(12000),
          target(10000); //  10ms
      auto res = nif::common::utils::numeric::clip<std::chrono::microseconds>(min, max, target);
      ASSERT_EQ(res, std::chrono::microseconds(10000));
    }
  }
}