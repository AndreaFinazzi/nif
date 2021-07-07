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

TEST(UtilsNumericClipTest, ClipTestMin) {
  {
    double a = 0.1, b = 1.1, c = 0.005;
    auto res = nif::common::utils::numeric::clip<double>(a, b, c);
    ASSERT_DOUBLE_EQ(res, 0.1);
  }
}

TEST(UtilsNumericClipTest, ClipTestMax) {
  {
    double a = 0.1, b = 1.1, c = 0.005;
    auto res = nif::common::utils::numeric::clip<double>(a, b, c);
    ASSERT_DOUBLE_EQ(res, 1.1);
  }
}

TEST(UtilsNumericClipTest, ClipTestPass) {
  {
    double a = 0.1, b = 1.1, c = 0.505;
    auto res = nif::common::utils::numeric::clip<double>(a, b, c);
    ASSERT_DOUBLE_EQ(res, 0.505);
  }
}