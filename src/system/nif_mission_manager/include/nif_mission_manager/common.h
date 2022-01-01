//
// Created by usrg on 9/30/21.
//

#ifndef ROS2MASTER_COMMON_H
#define ROS2MASTER_COMMON_H

#include "yaml-cpp/yaml.h"
#include "nif_common/types.h"
#include "nif_utils/polygon_helper.h"

#include <unordered_map>

#define ASSERT(x) if( !(x) ) throw std::range_error("Assertion failed in yaml_cpp_adapter");

using nif_msgs::msg::MissionStatus;
using nif::utils::geometry::Point2D;

namespace nif {
namespace system {
    using track_zone_id_t = uint16_t;
    using track_zone_type_t = uint16_t;

    constexpr const char* ID_MISSIONS_LIST = "missions";
    constexpr const char* ID_MISSION_CODE = "mission_code";
    constexpr const char* ID_CONDITION_ACTIVE = "active";

    constexpr const char* ID_ACTIVATION_AREA = "activation_area";
    constexpr const char* ID_ACTIVATION_AREA_BOUNDING_BOXES = "bboxes";
    constexpr const char* ID_ACTIVATION_AREA_POLYGONS = "polygons";

    constexpr const char* ID_ACTIVATION_VELOCITY = "activation_velocity";
    constexpr const char* ID_ACTIVATION_VELOCITY_RANGE = "range_mps";

    constexpr const char* ID_ALLOWED_TRANSITIONS = "allowed_transitions";
    constexpr const char* ID_ALLOWED_TRANSITIONS_FROM = "from";

    constexpr const char* ID_TIMEOUT = "timeout";
    constexpr const char* ID_TIMEOUT_DURATION = "duration_ms";

    constexpr const char* ID_FALLBACK = "fallback";
    constexpr const char* ID_FALLBACK_MISSION_CODE = "mission_code";

    constexpr const char* ID_ZONES_LIST = "zones";
    constexpr const char* ID_ZONE_ID = "id";
    constexpr const char* ID_ZONE_TYPE = "type";
    constexpr const char* ID_ZONE_BBOX = "bbox";
    constexpr const char* ID_ZONE_POLYGON = "polygon";

    constexpr const char* ID_ZONE_DYNAMICS = "dynamics";
    constexpr const char* ID_ZONE_DYNAMICS_LONG_ACCELERATION_MAX = "long_acceleration_max";
    constexpr const char* ID_ZONE_DYNAMICS_LONG_ACCELERATION_MIN = "long_acceleration_min";

    struct MissionCondition {
        bool active = false;

        virtual 
        bool isValid(
            const MissionStatus::_mission_status_code_type &current_mission,
            const MissionStatus::_mission_status_code_type &next_mission,
            const nif::common::msgs::Odometry &ego_odom,
            const MissionStatus::_max_velocity_mps_type &ego_velocity_mps) = 0;
    };

    struct Polygon {
        std::vector<Point2D> points;

        bool isValid(
            const MissionStatus::_mission_status_code_type &current_mission,
            const MissionStatus::_mission_status_code_type &next_mission,
            const nif::common::msgs::Odometry &ego_odom,
            const MissionStatus::_max_velocity_mps_type &ego_velocity_mps)
            {
                return (
                    nif::utils::geometry::poly::isInside(
                        points, 
                        points.size(), 
                        {ego_odom.pose.pose.position.x, ego_odom.pose.pose.position.y})
                );
            }
    };


    struct BBox {
        double x_min;
        double y_min;
        double x_max;
        double y_max;
 
        bool isValid(
            const MissionStatus::_mission_status_code_type &current_mission,
            const MissionStatus::_mission_status_code_type &next_mission,
            const nif::common::msgs::Odometry &ego_odom,
            const MissionStatus::_max_velocity_mps_type &ego_velocity_mps)
            {
                return (
                    ego_odom.pose.pose.position.x >= x_min &&
                    ego_odom.pose.pose.position.y >= y_min &&
                    ego_odom.pose.pose.position.x <= x_max &&
                    ego_odom.pose.pose.position.y <= y_max
                );
            }
    };

    struct MissionActivationArea : MissionCondition {
        std::vector<BBox> bounding_boxes;
        std::vector<Polygon> polygons;

        virtual 
        bool isValid(
            const MissionStatus::_mission_status_code_type &current_mission,
            const MissionStatus::_mission_status_code_type &next_mission,
            const nif::common::msgs::Odometry &ego_odom,
            const MissionStatus::_max_velocity_mps_type &ego_velocity_mps)
            {
                if (!this->active) return true;

                bool is_valid = false;
                for (auto &&bbox : bounding_boxes)
                {
                    is_valid = is_valid || bbox.isValid(
                        current_mission, next_mission, ego_odom, ego_velocity_mps);
                }
                
                for (auto &&polygon : polygons)
                {
                    is_valid = is_valid || polygon.isValid(
                        current_mission, next_mission, ego_odom, ego_velocity_mps);
                }
                return is_valid;
            }
    };

    struct MissionActivationVelocity : MissionCondition {
        double range_min_mps;
        double range_max_mps;

        bool isValid(
            const MissionStatus::_mission_status_code_type &current_mission,
            const MissionStatus::_mission_status_code_type &next_mission,
            const nif::common::msgs::Odometry &ego_odom,
            const MissionStatus::_max_velocity_mps_type &ego_velocity_mps)
            {
                if (!this->active) return true;

                return (
                    this->active &&
                    ego_velocity_mps >= this->range_min_mps &&
                    ego_velocity_mps <= this->range_max_mps);
            }
    };

    struct MissionAllowedTransitions : MissionCondition {
        std::vector<MissionStatus::_mission_status_code_type> from;
        std::vector<MissionStatus::_mission_status_code_type> to; // Unused

        bool isValid(
            const MissionStatus::_mission_status_code_type &current_mission,
            const MissionStatus::_mission_status_code_type &next_mission,
            const nif::common::msgs::Odometry &ego_odom,
            const MissionStatus::_max_velocity_mps_type &ego_velocity_mps)
            {
                if (!this->active) return true;

                return
                    this->active &&
                    std::find(this->from.begin(), this->from.end(), current_mission) != this->from.end();
            }
    };

    struct MissionTimeout {
        bool active = false;
        long int duration_ms;
    };

    struct MissionFallback {
        bool active = false;
        MissionStatus::_mission_status_code_type mission_code;
        
        bool isActive()
            {
                return (this->active);
            }
    };
    
    struct MissionNode : MissionCondition {
        // TODO collect MissionConditions in a dinamic collection, for flexibility
        MissionStatus::_mission_status_code_type mission_code;
        MissionFallback fallback;
        MissionActivationArea activation_area;
        MissionActivationVelocity activation_velocity;
        MissionAllowedTransitions allowed_transitions;
        MissionTimeout timeout;

        bool isValid(
            const MissionStatus::_mission_status_code_type &current_mission,
            const MissionStatus::_mission_status_code_type &next_mission,
            const nif::common::msgs::Odometry &ego_odom,
            const MissionStatus::_max_velocity_mps_type &ego_velocity_mps)
            {
                if (!this->active) return true;

                return (this->active &&
                    this->activation_area.isValid(current_mission, next_mission, ego_odom, ego_velocity_mps) &&
                    this->activation_velocity.isValid(current_mission, next_mission, ego_odom, ego_velocity_mps) &&
                    this->allowed_transitions.isValid(current_mission, next_mission, ego_odom, ego_velocity_mps)
                );
            }
    };

    struct TrackZoneDynamics {
        double long_acceleration_max = 0;
        double long_acceleration_min = 0;
    };

    struct TrackZone {
        // TODO collect MissionConditions in a dinamic collection, for flexibility
        track_zone_id_t id;
        
        track_zone_type_t type;

        bool has_bbox = false;
        BBox bbox;

        bool has_polygon = false;
        Polygon polygon;

        TrackZoneDynamics dynamics;

        bool isValid(
            const MissionStatus::_mission_status_code_type &current_mission,
            const MissionStatus::_mission_status_code_type &next_mission,
            const nif::common::msgs::Odometry &ego_odom,
            const MissionStatus::_max_velocity_mps_type &ego_velocity_mps)
            {
                bool is_valid = 
                    (has_bbox && this->bbox.isValid(current_mission, next_mission, ego_odom, ego_velocity_mps)) || 
                    (has_polygon && this->polygon.isValid(current_mission, next_mission, ego_odom, ego_velocity_mps));
                
                return is_valid;
            }
    };


    using MissionsDescription = std::unordered_map<
            MissionStatus::_mission_status_code_type, MissionNode>;
            
    using TrackZonesDescription = std::unordered_map<
            track_zone_id_t, TrackZone>;
            
} // namespace system
} // namespace nif

#endif //ROS2MASTER_COMMON_H
