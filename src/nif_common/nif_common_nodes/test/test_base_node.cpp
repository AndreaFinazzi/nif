//
// Created by usrg on 6/18/21.
//

#include "gtest/gtest.h"
#include "base_node.h"

class BaseNodeTest : public ::testing::Test {
protected:
  void SetUp() override {

  }

//  IBaseNode node();
};

TEST_F(BaseNodeTest, MockTestOne) {
  ASSERT_EQ(3, 3)
}

TEST_F(BaseNodeTest, MockTestTwo) {
ASSERT_EQ(3, 3)
}