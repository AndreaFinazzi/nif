/**
 * @file   tire_manager_node.cpp
 * @author Hyunki Seong
 * @date   2021-08-22
 * @brief  ROS2 node implementation for testing load transfer based on vehicle
 * dynamics
 * @arg    input      : lateral acceleration, longitudinal acceleration
 *         output     : wheel load: Fz_LF, Fz_RF, Fz_LR, Fz_RR
 *         parameters : ccccc
 * @param
 * @note   ***IMPORTANT ASSUMPTION***
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "vehicle_dynamics_manager/tire_manager.hpp"

//! Update Rate (in Hz)
const double update_rate_hz = 100.;

class TireManagerNode : public rclcpp::Node {
public:
  TireManagerNode() : Node("tire_manager_node") {
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/novatel_bottom/imu/data", 10, // or /raptor_dbw_interface/imu/data_raw
        std::bind(&TireManagerNode::callback_imu, this, std::placeholders::_1));

    //! Timer to publish at specific Hz
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000. / update_rate_hz)),
        std::bind(&TireManagerNode::publishTireLoad, this));
  };

  void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "I heard imu: '%f'", msg->data);
    m_a_lon = msg->linear_acceleration.x;
    m_a_lat = msg->linear_acceleration.y;
  }

  void publishTireLoad() {
    //
    std::vector<double> tire_load;
    tire_load = tire_load_manager.ComputeLoadTransfer(m_a_lon, m_a_lat);
    std::cout << "--------------------------------" << std::endl;
    std::cout << "Accel lon, lat : " << m_a_lon << " " << m_a_lat;
    std::cout << "\nTire load LF : " << tire_load[0];
    std::cout << "\nTire load RF : " << tire_load[1];
    std::cout << "\nTire load LR : " << tire_load[2];
    std::cout << "\nTire load RR : " << tire_load[3] << std::endl;

    tire_load_manager.Compute4WheelLateralForceLimit(m_a_lon, m_a_lat);

    // // for test
    // std::cout << "Fz_LF[0] : " << tire_load_manager.m_tire_fz_fy_LF[0][0];
    // std::cout << "\nFyMax_LF[0] : " <<
    // tire_load_manager.m_tire_fz_fy_LF[0][1]; std::cout << "\nFyMin_LF[0] : "
    // << tire_load_manager.m_tire_fz_fy_LF[0][2]
    //           << std::endl;
  }

private:
  //! Functions
  TireManager tire_load_manager;

  //! Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  //! Update Timers
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Variables
  double m_a_lon = 0.0;
  double m_a_lat = 0.0;

}; /* class TireManagerNode */

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TireManagerNode>());
  rclcpp::shutdown();
  return 0;
}
