//
// Created by usrg on 6/23/21.
//

#include "nif_control_safety_layer_nodes/control_safety_layer_node.h"
#include <memory>

int32_t main(int32_t argc, char ** argv)
{
    rclcpp::init(argc, argv);

    using motion::control::mpc_controller_nodes::MpcControllerNode;
    const auto nd = std::make_shared<MpcControllerNode>("mpc_controller", "");

    rclcpp::spin(nd);

    rclcpp::shutdown();

    return 0;
}
