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
#include "nif_msgs/srv/register_node_status.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include <rclcpp/rclcpp.hpp>

namespace nif {
namespace common {

using nif_msgs::msg::MissionStatus;

class IBaseNode : public rclcpp::Node {
public:
  inline std::string getNodeStatusTopicName() {
    return this->get_global_parameter<std::string>(
           nif::common::constants::parameters::names::TOPIC_ID_PREFIX_NODE_STATUS) +
           std::string(this->get_name());
  }

protected:
  // DEPRECATED, will be removed!
  __attribute_deprecated__
  explicit IBaseNode(const std::string &);

  IBaseNode(const std::string &, const NodeType,
            const rclcpp::NodeOptions & = rclcpp::NodeOptions{},
            const bool register_to_system_manager = true);

  virtual ~IBaseNode() {
    this->setNodeStatus(NodeStatusCode::NODE_DEAD);
  }

  __attribute_deprecated__
  virtual void initParameters() {};

  __attribute_deprecated__
  virtual void getParameters() {};

  const std::string &getBodyFrameId() const;
  const std::string &getGlobalFrameId() const;
  const rclcpp::Time &getGclockNodeInit() const;
  const msgs::Odometry &getEgoOdometry() const;
  const msgs::PowertrainState &getEgoPowertrainState() const;

  bool hasEgoOdometry() const;
  bool hasEgoPowertrainState() const;
  bool hasSystemStatus() const;

  const msgs::SystemStatus &getSystemStatus() const;
  const rclcpp::Time &getEgoOdometryUpdateTime() const;
  const rclcpp::Time &getEgoPowertrainStateUpdateTime() const;
  const rclcpp::Time &getSystemStatusUpdateTime() const;

  rclcpp::SyncParametersClient::SharedPtr global_parameters_client;

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle;

  void setNodeStatus(NodeStatusCode status_code) noexcept;

  /**
   * Gets a global parameter through the global parameter client.
   * Note that the getter mentioned above throws `rclcpp::exceptions::ParameterNotDeclaredException` if the global parameter is not defined, and `std::runtime_exception` if the server is unavailable, thus the caller should define a proper strategy to handle miscalls.
   * @tparam T Parameter type.
   * @param param_name Parameter name.
   * @return Parameter value, if defined.
   */
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

  /**
   * Sets a global parameter through the global parameter client.
   * Note that the getter mentioned above throws `rclcpp::exceptions::ParameterNotDeclaredException` if the global parameter is not defined, and `std::runtime_exception` if the server is unavailable, thus the caller should define a proper strategy to handle miscalls.
   * @tparam T Parameter's value type.
   * @param param_name Parameter's name.
   * @param value Parameter's value.
   */
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


  bool missionIs(MissionStatus::_mission_status_code_type mission) {
    return this->getSystemStatus().mission_status.mission_status_code == mission;
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

  /**
   * Ego odometry from the localization layer.
   * It's automatically stored by its callback along with ego_odometry_update_time.
   */
  nif::common::msgs::Odometry ego_odometry;

  bool has_ego_odometry = false;

  /**
  * Ego powertrain status from the vehicle interface.
  * It's automatically stored by its callback along with ego_powertrain_state_update_time.
  */
  nif::common::msgs::PowertrainState ego_powertrain_state;

  bool has_ego_powertrain_state = false;

  // TODO : finalize SystemState class
  /**
  * System status from the system status manager.
  * It's automatically stored by its callback along with system_status_update_time.
  */
  nif::common::msgs::SystemStatus system_status;

  bool has_system_status = false;

  bool has_race_control_status = false;

  /**
 * Ego odometry last update time.
 */
  rclcpp::Time ego_odometry_update_time;

  /**
  * Ego powertrain last update time.
  */
  rclcpp::Time ego_powertrain_state_update_time;

  // TODO : finalize SystemState class
  /**
  * System status last update time.
  */
  rclcpp::Time system_status_update_time;


  rclcpp::Subscription<nif::common::msgs::Odometry>::SharedPtr
      ego_odometry_sub;
  rclcpp::Subscription<nif::common::msgs::PowertrainState>::SharedPtr
      ego_powertrain_state_sub;

  rclcpp::Subscription<nif::common::msgs::SystemStatus>::SharedPtr
      system_status_sub;

  void egoOdometryCallback(
      const nif::common::msgs::Odometry::SharedPtr msg);

  void egoPowertrainCallback(
      const nif::common::msgs::PowertrainState::SharedPtr msg);

  void systemStatusCallback(
      const nif::common::msgs::SystemStatus::SharedPtr msg);


  /**
  * It's called at the end of egoOdometryCallback(...)
  * and it can be customized.
  */
  virtual
      void afterEgoOdometryCallback() {}

  /**
  * It's called at the end of egoPowertrainCallback(...)
  * and it can be customized.
  */
  virtual
      void afterEgoPowertrainCallback() {}

  /**
  * It's called at the end of systemStatusCallback(...)
  * and it can be customized.
  */
  virtual
      void afterSystemStatusCallback() {}

  virtual
      rcl_interfaces::msg::SetParametersResult
  parametersSetCallback(const std::vector<rclcpp::Parameter> &vector)
  {
    return rcl_interfaces::msg::SetParametersResult{};
  }
//  ### NODE STATUS COMPONENTS
  rclcpp::TimerBase::SharedPtr node_status_timer;

  nif::common::types::t_clock_period_us node_status_timer_period_us;

  rclcpp::Publisher<nif_msgs::msg::NodeStatus>::SharedPtr node_status_pub;

  nif::common::NodeStatusManager node_status_manager;

  rclcpp::Client<nif_msgs::srv::RegisterNodeStatus>::SharedPtr register_node_service_client;

  void nodeStatusTimerCallback();

};

} // namespace common
} // namespace nif
#endif // NIF_COMMON_NODES_BASENODE_H
