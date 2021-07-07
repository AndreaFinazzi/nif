//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/18/21.
//

#include "nif_common_nodes/i_base_node.h"
#include "gtest/gtest.h"

class BaseNodeTest : public ::testing::Test {
protected:
  void SetUp() override {}
};

TEST_F(BaseNodeTest, MockTestOne) { ASSERT_EQ(3, 3); }

TEST_F(BaseNodeTest, MockTestTwo) { ASSERT_EQ(3, 3); }