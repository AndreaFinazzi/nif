//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/18/21.
//

#include "base_node_test_node.h"
#include "gtest/gtest.h"


TEST(BaseNodeTest, MockTestOne)
{
  TestNode node;
  node.stateReport();
}

//TEST_F(BaseNodeTest, MockTestTwo) { ASSERT_EQ(3, 3); }