//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/18/21.
//

#include "csl_node_test_node.h"
#include "nif_control_safety_layer_nodes/control_safety_layer_node.h"
#include "gtest/gtest.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>

class sanity_check : public ::testing::Test
{
public:
  virtual ~sanity_check() {}
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown()
  {
    (void)rclcpp::shutdown();
  }
};  // sanity_check


TEST_F(sanity_check, testone) {
  using namespace nif::common::constants;
  using nif::control::ControlSafetyLayerNode;

  std::shared_ptr<MockControlNode> nd_t;
  rclcpp::Node::SharedPtr nd_csl;

  try {
    RCLCPP_INFO(
        rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
        "Instantiating MockControlNode with name: %s;",
        "control_test_node");

    nd_t = std::make_shared<MockControlNode>("control_test_node");
    nd_csl = std::make_shared<ControlSafetyLayerNode>("control_safety_layer_node", nif::common::constants::SYNC_PERIOD_DEFAULT_US);

  } catch (std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s",
                 e.what());
    return;
  }

  {
    int i = 0;
    rclcpp::executors::MultiThreadedExecutor multi_threaded_executor(rclcpp::ExecutorOptions{}, 2);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(nd_t);
//    exec.add_node(nd_csl);
    while (i != 1000000) {
      exec.spin_some(std::chrono::milliseconds(10LL));
      i++;
    }
    // spin one more time for good measure
    exec.spin_some(std::chrono::milliseconds(100LL));
    
  }



  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [MockControlNode]", "control_test_node");

}

// TEST_F(BaseNodeTest, MockTestTwo) { ASSERT_EQ(3, 3); }