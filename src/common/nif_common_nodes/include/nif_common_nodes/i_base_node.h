//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/18/21.
//

#ifndef NIF_COMMON_NODES_BASENODE_H
#define NIF_COMMON_NODES_BASENODE_H

#include <string>

#include "nif_common/types.h"
#include "nif_common_nodes/node_status_manager.h"
#include "nif_utils/utils.h"

#include "tf2_ros/transform_broadcaster.h"
#include <rclcpp/rclcpp.hpp>

namespace nif {
namespace common {

class IBaseNode : public rclcpp::Node {
public:
protected:
  // DEPRECATED, will be removed!
  explicit IBaseNode(const std::string &);

  IBaseNode(const std::string &, const NodeType,
            const rclcpp::NodeOptions & = rclcpp::NodeOptions{});

  virtual ~IBaseNode() {
    this->node_status_manager.update(NodeStatusCode::NODE_DEAD);
  }

  virtual void initParameters(){};
  virtual void getParameters(){};

  const std::string &getBodyFrameId() const;
  const std::string &getGlobalFrameId() const;
  const rclcpp::Time &getGclockNodeInit() const;
  const msgs::Odometry &getEgoOdometry() const;
  const msgs::PowertrainState &getEgoPowertrainState() const;
  const msgs::SystemStatus &getSystemState() const;
  const msgs::RaceControlState &getRaceControlState() const;

  nif::common::NodeStatusManager node_status_manager;

  rclcpp::SyncParametersClient::SharedPtr global_parameters_client;

  template <class T>
  T get_global_parameter(const std::string &param_name) {
    if (this->global_parameters_client->service_is_ready()) {
      if (this->global_parameters_client->has_parameter(param_name)) {
        // Try to get parameters
        return static_cast<T>(
            this->global_parameters_client->get_parameter<T>(param_name));

      } else {
        throw rclcpp::exceptions::ParameterNotDeclaredException("Global parameter " + param_name + " has not been declared.");
      }
    } else {
      //  TODO define specific exception type
      throw std::runtime_error("GlobalParametersNode unavailable, can't "
                               "retrieve global parameters.");
    }
  }

  template <class T>
  void set_global_parameter(
      const std::string & param_name, const T & value) {
    if (this->global_parameters_client->service_is_ready()) {

        RCLCPP_INFO(this->get_logger(), "Setting global parameter %s",
                    param_name.c_str());
        this->global_parameters_client->set_parameters({
            rclcpp::Parameter(param_name, value)
        });

    } else {

      throw std::runtime_error("GlobalParametersNode unavailable, can't "
                               "retrieve global parameters.");
    }
  }

private:
  /**
   * The default constructor is hidden from the outside to prevent unnamed
   * nodes.
   */
  IBaseNode();

  std::string body_frame_id;

  //  TODO define precisely which frame is considered global
  std::string global_frame_id;

  /**
   * Initialization time
   **/
  rclcpp::Time gclock_node_init;

  nif::common::msgs::Odometry ego_odometry;

  nif::common::msgs::PowertrainState ego_powertrain_state;

  // TODO : finalize SystemState class
  nif::common::msgs::SystemStatus system_state;

  // TODO : finalize RaceControlState class
  nif::common::msgs::RaceControlState race_control_state;

  rclcpp::Subscription<nif::common::msgs::Odometry>::SharedPtr ego_odometry_sub;

  rclcpp::Subscription<nif::common::msgs::PowertrainState>::SharedPtr
      ego_powertrain_state_sub;

  rclcpp::Subscription<nif::common::msgs::SystemStatus>::SharedPtr
      system_state_sub;

  rclcpp::Subscription<nif::common::msgs::RaceControlState>::SharedPtr
      race_control_state_sub;

  void egoOdometryCallback(const nif::common::msgs::Odometry::SharedPtr msg);

  void egoPowertrainCallback(
      const nif::common::msgs::PowertrainState::SharedPtr msg);

  void
  systemStateCallback(const nif::common::msgs::SystemStatus::SharedPtr msg);

  void raceControlStateCallback(
      const nif::common::msgs::RaceControlState::SharedPtr msg);
};

} // namespace common
} // namespace nif
#endif // NIF_COMMON_NODES_BASENODE_H
