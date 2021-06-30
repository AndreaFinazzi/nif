//
// Created by usrg on 6/28/21.
//

//
// Created by usrg on 6/27/21.
//

#include "gtest/gtest.h"
#include "nif_utils/utils.h"


TEST(UtilsNumericClipTest, ClipTestMin) {
    {
        double res = nif::common::utils::numeric::clip(0.1, 1.1, 0.005);
        ASSERT_DOUBLE_EQ(res, 0.1);
    }
}

TEST(UtilsNumericClipTest, ClipTestMax) {
    {
        double res = nif::common::utils::numeric::clip(0.1, 1.1, 2.005);
        ASSERT_DOUBLE_EQ(res, 1.1);
    }
}

TEST(UtilsNumericClipTest, ClipTestPass) {
    {
        double res = nif::common::utils::numeric::clip(0.1, 1.1, 0.505);
        ASSERT_DOUBLE_EQ(res, 0.505);
    }
}