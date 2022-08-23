#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <nif_msgs/msg/autonomy_status.hpp>
#include <nif_msgs/msg/system_health_status.hpp>
#include <nif_msgs/msg/mission_status.hpp>
#include <nif_msgs/msg/system_status.hpp>
#include <nif_common/types.h>
#include "nif_msgs/srv/register_node_status.hpp"


using namespace std::chrono_literals;

class FakeSystemStatus : public rclcpp::Node
{
  public:
    FakeSystemStatus()
    : Node("FakeSystemStatus_node")
    {

        m_fake_system_status_pub = this->create_publisher<nif_msgs::msg::SystemStatus>("/system/status", 10);
    // timer
        timer_ = this->create_wall_timer(
        10ms, std::bind(&FakeSystemStatus::timer_callback, this));

        m_fake_autonomy_status.longitudinal_autonomy_enabled = true;
        m_fake_autonomy_status.lateral_autonomy_enabled = true;
        m_fake_system_health_status.system_failure = false;
        m_fake_system_health_status.communication_failure = false;
        m_fake_system_health_status.localization_failure = false;
        m_fake_system_health_status.system_status_code = 0; //(SYSTEM_OK)
        
        m_fake_mission_status.stamp_last_update = rclcpp::Clock().now();
        m_fake_mission_status.track_flag = nif::common::msgs::RCFlagSummary::TRACK_FLAG_GREEN;
        m_fake_mission_status.veh_flag = nif::common::msgs::RCFlagSummary::VEH_FLAG_DEFENDER;
        m_fake_mission_status.mission_status_code = nif_msgs::msg::MissionStatus::MISSION_PIT_INIT;
        m_fake_mission_status.max_velocity_mps = 5.0;
        m_fake_mission_status.zone_status.zone_type = nif_msgs::msg::ZoneStatus::ZONE_TYPE_PIT;
        m_fake_mission_status.zone_status.long_acceleration_max = 2.0;
        m_fake_mission_status.zone_status.long_acceleration_min = 3.0;

        m_fake_system_status_msg.autonomy_status = m_fake_autonomy_status;
        m_fake_system_status_msg.health_status = m_fake_system_health_status;
        m_fake_system_status_msg.mission_status = m_fake_mission_status;

        this->register_node_service = this->create_service<nif_msgs::srv::RegisterNodeStatus>(
            "/system_status_manager/register",
            std::bind(
                    &FakeSystemStatus::registerNodeServiceHandler,
                    this,
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    );
  }
  ~FakeSystemStatus()
  {
    
  }

  private:
    rclcpp::Publisher<nif_msgs::msg::SystemStatus>::SharedPtr m_fake_system_status_pub;
    nif_msgs::msg::AutonomyStatus m_fake_autonomy_status;
    nif_msgs::msg::SystemHealthStatus m_fake_system_health_status;
    nif_msgs::msg::MissionStatus m_fake_mission_status;

    rclcpp::TimerBase::SharedPtr timer_;
    nif_msgs::msg::SystemStatus m_fake_system_status_msg;
    rclcpp::Service<nif_msgs::srv::RegisterNodeStatus>::SharedPtr register_node_service;
    
    void timer_callback()
    {
        m_fake_system_status_msg.header.stamp = rclcpp::Clock().now();                

        m_fake_system_status_pub->publish( m_fake_system_status_msg);
    }

    void registerNodeServiceHandler(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const nif_msgs::srv::RegisterNodeStatus::Request::SharedPtr request,
        nif_msgs::srv::RegisterNodeStatus::Response::SharedPtr response) {
//  Request data check
    // if (!request->node_name.empty() &&
        // isNodeTypeInRange(static_cast<NodeType>(request->node_type)) &&
        // !request->status_topic_name.empty()) {
        
        // if (this->status_index_by_name.find(request->node_name) != this->status_index_by_name.end())
            // Process only if not already subscribed
            // RCLCPP_ERROR(this->get_logger(), "DUPLICATE NODE DETECTED.");

        // auto node_id = this->newStatusRecord(request);

        // if (node_id > 0) {
            // response->node_id = node_id;
            // response->success = true;
            // response->message = "Node " + request->node_name + " hase been registered in SSM.";
            // return;
        // }
    
      response->success = true;
      return;
    }
   
    // response->node_id = 0;
    // response->success = false;
    // response->message = "ERROR: Bad service request.";
};





int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeSystemStatus>());
  rclcpp::shutdown();
  return 0;
}