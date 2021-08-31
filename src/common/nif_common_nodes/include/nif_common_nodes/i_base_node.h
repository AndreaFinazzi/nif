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

  __attribute_deprecated__
  virtual void initParameters() {};

  __attribute_deprecated__
  virtual void getParameters() {};

  const std::string &getBodyFrameId() const;
  const std::string &getGlobalFrameId() const;
  const rclcpp::Time &getGclockNodeInit() const;
  const msgs::Odometry &getEgoOdometry() const;
  const msgs::PowertrainState &getEgoPowertrainState() const;
  const msgs::SystemStatus &getSystemState() const;
  const msgs::RaceControlStatus &getRaceControlState() const;

  nif::common::NodeStatusManager node_status_manager;

  rclcpp::SyncParametersClient::SharedPtr global_parameters_client;

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

  // TODO : finalize RaceControlState class
  /**
  * Race Control input from the race control interface.
  * It's automatically stored by its callback along with race_control_status_update_time.
  */
  nif::common::msgs::RaceControlStatus race_control_status;

  bool has_race_control_status = false;

public:
  bool hasEgoOdometry() const;
  bool hasEgoPowertrainState() const;
  bool hasSystemStatus() const;
  bool hasRaceControlStatus() const;

private:
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

  // TODO : finalize RaceControlState class
  /**
  * Race Control input last update time.
  */
  rclcpp::Time race_control_status_update_time;


  rclcpp::Subscription<nif::common::msgs::Odometry>::SharedPtr
      ego_odometry_sub;
  rclcpp::Subscription<nif::common::msgs::PowertrainState>::SharedPtr
      ego_powertrain_state_sub;

public:
  const msgs::SystemStatus &getSystemStatus() const;
  const msgs::RaceControlStatus &getRaceControlStatus() const;
  const rclcpp::Time &getEgoOdometryUpdateTime() const;
  const rclcpp::Time &getEgoPowertrainStateUpdateTime() const;
  const rclcpp::Time &getSystemStatusUpdateTime() const;
  const rclcpp::Time &getRaceControlStatusUpdateTime() const;

private:
  rclcpp::Subscription<nif::common::msgs::SystemStatus>::SharedPtr
      system_status_sub;
  rclcpp::Subscription<nif::common::msgs::RaceControlStatus>::SharedPtr
      race_control_status_sub;

  void egoOdometryCallback(
      const nif::common::msgs::Odometry::SharedPtr msg);

  void egoPowertrainCallback(
      const nif::common::msgs::PowertrainState::SharedPtr msg);

  void systemStatusCallback(
      const nif::common::msgs::SystemStatus::SharedPtr msg);

  void raceControlStatusCallback(
      const nif::common::msgs::RaceControlStatus::SharedPtr msg);

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

  /**
  * It's called at the end of raceControlStatusCallback(...)
  * and it can be customized.
  */
  virtual
      void afterRaceControlStatusCallback() {}

};

} // namespace common
} // namespace nif
#endif // NIF_COMMON_NODES_BASENODE_H
