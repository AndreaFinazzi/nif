#include "rclcpp/rclcpp.hpp"
#include "nif_common/constants.h"


/**
 * Node responsible for global parameters. Each parameter 'declared' for this node, is globally availabel to the others.
 */ 
class GlobalParameterNode : public rclcpp::Node

{

public:
    GlobalParameterNode(const std::string & node_name) 
    : Node(node_name,
           rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)) {}
private:

};

int32_t main(int32_t argc, char** argv) {
  rclcpp::init(argc, argv);

  using namespace nif::common::constants;

  const std::string & node_name = nif::common::constants::parameters::GLOBAL_PARAMETERS_NODE_NAME;

  rclcpp::Node::SharedPtr nd;

  try {
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating GlobalParameterNode with name: %s",
                node_name.c_str());
    nd = std::make_shared<GlobalParameterNode>(node_name);

  } catch (std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s",
                 e.what());
    return -1;
  }

  rclcpp::spin(nd);
  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [GlobalParameterNode]",
              node_name.c_str());

  return 0;
}