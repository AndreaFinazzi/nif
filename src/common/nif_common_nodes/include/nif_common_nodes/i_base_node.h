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
#include "nif_utils/utils.h"

#include "tf2_ros/transform_broadcaster.h"
#include <rclcpp/rclcpp.hpp>

namespace nif {
namespace common {

class IBaseNode : public rclcpp::Node {
public:
protected:
  IBaseNode(const std::string& node_name, const rclcpp::NodeOptions& options);
  explicit IBaseNode(const std::string& node_name);

virtual ~IBaseNode() {}
private:
  /**
   * The default constructor is hidden from the outside to prevent unnamed
   * nodes.
   */
  IBaseNode();

  std::string body_frame_id;

//  TODO define precisely which frame is considered global
  std::string global_frame_id;

  rclcpp::Time gclock_node_init;

public:
  const std::string &getBodyFrameId() const;
  const std::string &getGlobalFrameId() const;
  const rclcpp::Time &getGclockNodeInit() const;
  const msgs::Odometry &getEgoOdometry() const;
  const msgs::PowertrainState &getEgoPowertrainState() const;
  const msgs::SystemState &getSystemState() const;
  const msgs::RaceControlState &getRaceControlState() const;

private:
  nif::common::msgs::Odometry ego_odometry;

  nif::common::msgs::PowertrainState ego_powertrain_state;

  // TODO : finalize SystemState class
  nif::common::msgs::SystemState system_state;

  // TODO : finalize RaceControlState class
  nif::common::msgs::RaceControlState race_control_state;

    rclcpp::Subscription<nif::common::msgs::Odometry>::SharedPtr ego_odometry_sub;

  rclcpp::Subscription<nif::common::msgs::PowertrainState>::SharedPtr
      ego_powertrain_state_sub;

  rclcpp::Subscription<nif::common::msgs::SystemState>::SharedPtr
      system_state_sub;

  rclcpp::Subscription<nif::common::msgs::RaceControlState>::SharedPtr
      race_control_state_sub;

  virtual void initParameters() = 0;
  virtual void getParameters() = 0;

  void egoOdometryCallback(const nif::common::msgs::Odometry::SharedPtr msg);

  void egoPowertrainCallback(
      const nif::common::msgs::PowertrainState::SharedPtr msg);

  void systemStateCallback(const nif::common::msgs::SystemState::SharedPtr msg);

  void raceControlStateCallback(
      const nif::common::msgs::RaceControlState::SharedPtr msg);
};

} // namespace common
} // namespace nif
#endif // NIF_COMMON_NODES_BASENODE_H
