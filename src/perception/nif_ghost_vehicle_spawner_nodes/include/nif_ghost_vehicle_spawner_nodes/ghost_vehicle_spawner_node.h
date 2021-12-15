//
// Created by usrg on 8/22/21.
//

#ifndef ROS2MASTER_GHOST_VEHICLE_SPAWNER_NODE_H
#define ROS2MASTER_GHOST_VEHICLE_SPAWNER_NODE_H

/**
 * @brief Ghost vehicle spawner node.
 **/

#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nif_msgs/msg/perception3_d.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nif_msgs/srv/ghost_vehicle_create.hpp"
#include "nif_msgs/srv/ghost_vehicle_get.hpp"
#include "nif_msgs/srv/ghost_vehicle_destroy.hpp"
#include "nif_msgs/srv/ghost_vehicle_update_maptrack.hpp"
#include "nif_msgs/srv/ghost_vehicle_update_relative_velocity.hpp"
#include "nif_msgs/srv/ghost_vehicle_update_pose.hpp"

#include "nif_common/types.h"
#include "nif_utils/utils.h"
#include "nif_utils/polygon_helper.h"
#include "nif_waypoint_manager_minimal/waypoint_manager_minimal.h"
#include "nif_common_nodes/i_base_synchronized_node.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <cstdlib>
#include <map>

auto    TIMER_PERIOD   = 40ms;
double  TIMER_PERIOD_S = 0.04;

namespace nif {
namespace perception {

struct GhostVehicleState {
    unsigned int id;
    nav_msgs::msg::Odometry odometry;
    geometry_msgs::msg::Pose pose_in_ego_body;
    double velocity_u_vector_in_ego_body[3];
    double relative_velocity_mps;
    // Speed vector?
    std::string maptrack_id;
    unsigned int waypoint_index;
    rclcpp::Time t_prev;
};

class GhostVehicleSpawnerNode : public nif::common::IBaseSynchronizedNode {
public:
    explicit GhostVehicleSpawnerNode(const std::string &node_name)
        : IBaseSynchronizedNode(node_name, nif::common::NodeType::TOOL, TIMER_PERIOD, rclcpp::NodeOptions{}, false)
    {
        std::string map_package_share_directory = "";
        try {
        // Get maps package (to avoid copyiong in multiple locations)
        map_package_share_directory = ament_index_cpp::get_package_share_directory(
            "nif_waypoint_manager_nodes");
        map_package_share_directory = map_package_share_directory.append("/");
        
        } catch (std::exception e) {
            RCLCPP_FATAL(this->get_logger(), "Can't get map_package_share_directory");
        }
        std::vector<std::string> file_ids_list_default = {"map"};
        std::vector<std::string> file_path_list_default = {"maps/map.csv"};

        this->declare_parameter("file_ids_list", file_ids_list_default);
        this->declare_parameter("file_path_list", file_path_list_default);
        
        this->file_ids_list = 
            this->get_parameter("file_ids_list").as_string_array();
        this->file_path_list =
            this->get_parameter("file_path_list").as_string_array();

        if (this->file_ids_list.size() == this->file_path_list.size()) {

            for (unsigned int i = 0; i < file_ids_list.size(); i++) {
                // Update wpt path with package prefix
                file_path_list[i].insert(0, map_package_share_directory);
                
                std::vector<std::string> maptrack_path_vector { file_path_list[i] }; // Pass single path
                // Populate map <maptrack_id, wpt_manager>
                wpt_manager_by_id.insert({
                    file_ids_list[i], std::make_unique<WaypointManagerMinimal>(
                        maptrack_path_vector,
                        this->getBodyFrameId(),
                        this->getGlobalFrameId(),
                        this->spline_interval)
                });
            }
        } else {
            throw std::invalid_argument("file_ids_list and file_path_list must have the same number of elements.");
        }

        // RCLCPP objects
        this->perception_result_pub = this->create_publisher<nif_msgs::msg::Perception3D>(
            "out_perception_result",
            nif::common::constants::QOS_SENSOR_DATA);

        this->marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
            "out_marker",
            nif::common::constants::QOS_SENSOR_DATA);

        this->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
            "out_odometry",
            nif::common::constants::QOS_SENSOR_DATA);

        // SERVICES 
        this->service_create_vehicle = this->create_service<nif_msgs::srv::GhostVehicleCreate>(
            "/ghost_spawner/create_vehicle", 
            std::bind(
                &GhostVehicleSpawnerNode::service_handler_create_vehicle,
                this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            
        this->service_get_vehicle = this->create_service<nif_msgs::srv::GhostVehicleGet>(
            "/ghost_spawner/get_vehicle", 
            std::bind(
                &GhostVehicleSpawnerNode::service_handler_get_vehicle,
                this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            
        this->service_update_vehicle_relative_velocity = this->create_service<nif_msgs::srv::GhostVehicleUpdateRelativeVelocity>(
            "/ghost_spawner/update_vehicle_relative_velocity", 
            std::bind(
                &GhostVehicleSpawnerNode::service_handler_update_vehicle_relative_velocity,
                this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            
        this->service_update_vehicle_pose = this->create_service<nif_msgs::srv::GhostVehicleUpdatePose>(
            "/ghost_spawner/update_vehicle_pose", 
            std::bind(
                &GhostVehicleSpawnerNode::service_handler_update_vehicle_pose,
                this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            
        this->service_update_vehicle_maptrack = this->create_service<nif_msgs::srv::GhostVehicleUpdateMaptrack>(
            "/ghost_spawner/update_vehicle_maptrack", 
            std::bind(
                &GhostVehicleSpawnerNode::service_handler_update_vehicle_maptrack,
                this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            
        this->service_destroy_vehicle = this->create_service<nif_msgs::srv::GhostVehicleDestroy>(
            "/ghost_spawner/destroy_vehicle", 
            std::bind(
                &GhostVehicleSpawnerNode::service_handler_destroy_vehicle,
                this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            
    }

    //  Main timer callback
    //  Foreach vehicle:
    //      vehicle.pose_in_ego_body.x = vehicle.pose_in_ego_body.x + (dt * vehicle.velocity.x);
    //      vehicle.pose_in_ego_body.x = vehicle.pose_in_ego_body.y + (dt * vehicle.velocity.y);
    //      vehicle.update_velocity_vector();
    //      perception_result = ...
    //      Inject random noise to solution
    //      pub->publish(perception_result);
    void run() override {
        auto now = this->now();

        for (auto const& [vehicle_id, vehicle_state] : vehicle_state_by_id) // C++17 
        {
            rclcpp::Duration dt = now - vehicle_state->t_prev;
    //      vehicle.pose_in_ego_body.x = vehicle.pose_in_ego_body.x + (dt * vehicle.velocity.x);
    //      vehicle.pose_in_ego_body.x = vehicle.pose_in_ego_body.y + (dt * vehicle.velocity.y);
            vehicle_state->pose_in_ego_body.position.x = vehicle_state->pose_in_ego_body.position.x 
                + ( TIMER_PERIOD_S * vehicle_state->velocity_u_vector_in_ego_body[0] * vehicle_state->relative_velocity_mps);

            vehicle_state->pose_in_ego_body.position.y = vehicle_state->pose_in_ego_body.position.y 
                + ( TIMER_PERIOD_S * vehicle_state->velocity_u_vector_in_ego_body[1] * vehicle_state->relative_velocity_mps);

            // Convert to global
            vehicle_state->odometry.header.stamp = now;
            vehicle_state->odometry.pose.pose = nif::common::utils::coordination::convertToFrame(
                this->getEgoOdometry().pose.pose,
                vehicle_state->pose_in_ego_body
            ).pose;

            // TODO update odometry linear twist
    //      vehicle.update_velocity_vector();
            auto& wpt_manager = this->wpt_manager_by_id[vehicle_state->maptrack_id];
            wpt_manager->setCurrentOdometry(vehicle_state->odometry);
            auto& maptrack_in_ghost_body = wpt_manager->getDesiredMapTrackInBody();
            auto& maptrack_in_global = wpt_manager->getDesiredMapTrackInGlobal();

            auto& next_wpt_in_ghost_body = maptrack_in_ghost_body.poses[10].pose;
            
            geometry_msgs::msg::Pose next_wpt_in_ego_body = nif::common::utils::coordination::convertToFrame(
                vehicle_state->pose_in_ego_body,
                next_wpt_in_ghost_body
            ).pose;

            double d_x = (next_wpt_in_ego_body.position.x - vehicle_state->pose_in_ego_body.position.x);
            double d_y = (next_wpt_in_ego_body.position.y - vehicle_state->pose_in_ego_body.position.y);
            double mag = sqrt(d_x * d_x + d_y * d_y);
            vehicle_state->velocity_u_vector_in_ego_body[0] = d_x / mag; 
            vehicle_state->velocity_u_vector_in_ego_body[1] = d_y / mag; 

            // Update orientation
            double theta_rad_ing_ego_body = atan(
                vehicle_state->velocity_u_vector_in_ego_body[0] / 
                vehicle_state->velocity_u_vector_in_ego_body[1]);
            vehicle_state->pose_in_ego_body.orientation.z = sin(theta_rad_ing_ego_body / 2.0);
            vehicle_state->pose_in_ego_body.orientation.w = cos(theta_rad_ing_ego_body / 2.0);

            vehicle_state->t_prev = now;

    //      perception_result = ...
            nif_msgs::msg::Perception3D perception_out_msg{};
            perception_out_msg.header.frame_id = this->getBodyFrameId();
            perception_out_msg.header.stamp = now;
            perception_out_msg.id = vehicle_id;
            perception_out_msg.obj_velocity_in_local.linear;
            perception_out_msg.detection_result_3d.center = vehicle_state->pose_in_ego_body;
            perception_out_msg.detection_result_3d.size.x = 4.0;
            perception_out_msg.detection_result_3d.size.y = 2.0;
            perception_out_msg.detection_result_3d.size.z = 1.0;

            visualization_msgs::msg::Marker marker_out_msg{};
            marker_out_msg.pose = vehicle_state->pose_in_ego_body;
            marker_out_msg.header.frame_id = this->getBodyFrameId();
            marker_out_msg.header.stamp = now;
            marker_out_msg.id = vehicle_id;
            marker_out_msg.scale.x = 4.0;
            marker_out_msg.scale.y = 2.0;
            marker_out_msg.scale.z = 1.0;
            
            marker_out_msg.color.a = 1.0;
            marker_out_msg.color.r = 0.0;
            marker_out_msg.color.g = 0.0;
            marker_out_msg.color.b = 1.0;


    //      Inject random noise to solution
    //      pub->publish(perception_result);
            this->perception_result_pub->publish(perception_out_msg);
            this->marker_pub->publish(marker_out_msg);
            this->odom_pub->publish(vehicle_state->odometry);
        }
    }

    // ---- CRUD ----

    /*
    * Register a new vehicle.
    */
    unsigned int create_vehicle(
        std::string&& maptrack_id, \
        const double relatvie_velocity_mps = 0, 
        unsigned int waypoint_index = 0) 
        {
            unsigned int vehicle_id = next_vehicle_id++;
            geometry_msgs::msg::Pose pose_in_ego_body;
            nav_msgs::msg::Odometry odometry;

            odometry.header.frame_id = this->getGlobalFrameId();

            auto& wpt_manager = this->wpt_manager_by_id[maptrack_id];
            odometry.pose.pose = wpt_manager->getPoseStampedAtIndex(0).pose;
            pose_in_ego_body = nif::common::utils::coordination::convertToFrame(
                this->getEgoOdometry().pose.pose,
                odometry.pose.pose).pose;

            // Populate map <vehicle_id, vehicle_state>
            vehicle_state_by_id[vehicle_id] = std::make_unique<GhostVehicleState>(GhostVehicleState{
                vehicle_id,
                odometry,
                pose_in_ego_body,
                {0.0, 0.0, 0.0}, // Velocity vector
                relatvie_velocity_mps,
                std::forward<std::string>(maptrack_id), // TODO verify this one
                waypoint_index,
                this->now()
            });

            return vehicle_id;
        }
    
    
    GhostVehicleState& get_vehicle_by_id(const unsigned int vehicle_id);
    void update_vehicle_relative_velocity(const unsigned int vehicle_id, const double relative_velocity_mps);
    
    void update_vehicle_pose(const unsigned int vehicle_id, const geometry_msgs::msg::Pose pose_in_ego_body);
    void update_vehicle_pose(const unsigned int vehicle_id, const nif::utils::geometry::Point2D point_in_body);
    
    void update_vehicle_maptrack(const unsigned int vehicle_id, const std::string& maptrack_id);
    
    void destroy_vehicle(const unsigned int vehicle_id)
    {
        vehicle_state_by_id.erase(vehicle_id);
    }

    // Service handlers
    void service_handler_create_vehicle(        
        const std::shared_ptr<rmw_request_id_t> request_header,
        const nif_msgs::srv::GhostVehicleCreate::Request::SharedPtr request,
        nif_msgs::srv::GhostVehicleCreate::Response::SharedPtr response)
        {
            auto vehicle_id = this->create_vehicle(
                std::move(request->maptrack_id), 
                request->relative_velocity_mps, 
                request->waypoint_index);

            if (vehicle_state_by_id.find(vehicle_id) != vehicle_state_by_id.end())
            {
                response->success = true;
                response->vehicle_id = vehicle_id;
                response->message = "OK: Ghost vehicle spawned successfully.";
            } else {
                response->success = false;
                response->message = "ERROR: Ghost vehicle could not be spawned.";
            }


        }

    void service_handler_get_vehicle(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const nif_msgs::srv::GhostVehicleGet::Request::SharedPtr request,
        nif_msgs::srv::GhostVehicleGet::Response::SharedPtr response
    ) {}
    
    void service_handler_update_vehicle_relative_velocity(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const nif_msgs::srv::GhostVehicleUpdateRelativeVelocity::Request::SharedPtr request,
        nif_msgs::srv::GhostVehicleUpdateRelativeVelocity::Response::SharedPtr response
    ) {}
    
    void service_handler_update_vehicle_pose(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const nif_msgs::srv::GhostVehicleUpdatePose::Request::SharedPtr request,
        nif_msgs::srv::GhostVehicleUpdatePose::Response::SharedPtr response
    ) {}
    
    void service_handler_update_vehicle_maptrack(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const nif_msgs::srv::GhostVehicleUpdateMaptrack::Request::SharedPtr request,
        nif_msgs::srv::GhostVehicleUpdateMaptrack::Response::SharedPtr response
    ) {}
    
    void service_handler_destroy_vehicle(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const nif_msgs::srv::GhostVehicleDestroy::Request::SharedPtr request,
        nif_msgs::srv::GhostVehicleDestroy::Response::SharedPtr response
    ) {}
    


private:
    std::vector<std::string> file_ids_list{};
    std::vector<std::string> file_path_list{};

    std::map<std::string, std::unique_ptr<WaypointManagerMinimal>> wpt_manager_by_id;
    std::map<unsigned int, std::unique_ptr<GhostVehicleState>> vehicle_state_by_id;

    std::string frame_id_body = "base_link";
    std::string frame_id_global = "odom";

    int spline_interval = 1;

    unsigned int next_vehicle_id = 0;

    rclcpp::Time t_prev;

    rclcpp::Publisher<nif_msgs::msg::Perception3D>::SharedPtr perception_result_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    rclcpp::Service<nif_msgs::srv::GhostVehicleCreate>::SharedPtr service_create_vehicle;
    rclcpp::Service<nif_msgs::srv::GhostVehicleGet>::SharedPtr service_get_vehicle;
    rclcpp::Service<nif_msgs::srv::GhostVehicleUpdateRelativeVelocity>::SharedPtr service_update_vehicle_relative_velocity;
    rclcpp::Service<nif_msgs::srv::GhostVehicleUpdatePose>::SharedPtr service_update_vehicle_pose;
    rclcpp::Service<nif_msgs::srv::GhostVehicleUpdateMaptrack>::SharedPtr service_update_vehicle_maptrack;
    rclcpp::Service<nif_msgs::srv::GhostVehicleDestroy>::SharedPtr service_destroy_vehicle;


}; /* class GhostVehicleSpawnerNode */

} // namespace perception
} // namespace nif

#endif // ROS2MASTER_GHOST_VEHICLE_SPAWNER_NODE_H
