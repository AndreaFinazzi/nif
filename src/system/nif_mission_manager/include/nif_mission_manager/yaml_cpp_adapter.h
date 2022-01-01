//
// Created by usrg on 9/30/21.
//

#ifndef ROS2MASTER_YAML_CPP_ADAPTER_H
#define ROS2MASTER_YAML_CPP_ADAPTER_H

#include "nif_mission_manager/common.h"
#include <rclcpp/rclcpp.hpp>

using namespace nif::system;
const rclcpp::Logger LOGGER = rclcpp::get_logger("nif_mission_manager::yaml-cpp");

namespace YAML {
//        static Node encode(const Vec3& in) {
//            Node node;
//            node.push_back(in.x);
//            node.push_back(in.y);
//            node.push_back(in.z);
//            return node;
//        }
// template<>
// struct convert<nif_msgs::msg::MissionStatus::_mission_status_code_type> {
//     /**
//      * @param node
//      * @param out
//      * @return parsed mission code
//      */
//     static bool decode(const Node &node, nif_msgs::msg::MissionStatus::_mission_status_code_type &out) {
//         ASSERT(node.isScalar());

//         auto value = node.as<int>();
//         ASSERT(value <= std::numeric_limits<
//                 nif_msgs::msg::MissionStatus::_mission_status_code_type>::max() );
//         ASSERT(value >= std::numeric_limits<
//             nif_msgs::msg::MissionStatus::_mission_status_code_type>::min() );
//         ASSERT(nif::common::msgs::isMissionCodeInRange(value));
//         return true;
//     }
// };

template<>
struct convert<Point2D> {
    /**
     * x: 0.0  # x_1
     * y: 0.0  # y_1
     * 
     * @param node
     * @param out
     * @return parsed Point2D
     */
    static bool decode(const Node &node, Point2D &out) {
        ASSERT(node.size() == 2);
        ASSERT(node["x"]);
        ASSERT(node["y"]);

        out.x = node["x"].as<double>();
        out.y = node["y"].as<double>();

        RCLCPP_INFO(LOGGER, "Loaded Point.");
        return true;
    }
};

template<>
struct convert<BBox> {
    /**
     * - - 0.0 # x_min
     *   - 0.0 # y_min
     *   - 1.0 # x_max
     *   - 1.0 # y_max
     * @param node
     * @param out
     * @return parsed bounding box
     */
    static bool decode(const Node &node, BBox &out) {
        ASSERT(node.size() == 4);
        ASSERT(node.IsSequence());

        out.x_min = node[0].as<double>();
        out.y_min = node[1].as<double>();
        out.x_max = node[2].as<double>();
        out.y_max = node[3].as<double>();

        ASSERT(out.x_min < out.x_max);
        ASSERT(out.y_min < out.y_max);

        RCLCPP_INFO(LOGGER, "Loaded BBox.");
        return true;
    }
};

template<>
struct convert<Polygon> {
    /**
     * - - x: 0.0  # x_1
     *     y: 0.0  # y_1
     *   - x: -1.0 # x_2
     *     y: 1.0  # y_2
     *   - x: 0.0  # x_3
     *     y: 2.0  # y_3
     *   - x: 1.0  # x_4
     *     y: 1.0  # y_4
     * @param node
     * @param out
     * @return parsed bounding box
     */
    static bool decode(const Node &node, Polygon &out) {
        ASSERT(node.size() >= 3);  // 3 verteces or more
        ASSERT(node.IsSequence());

        for (auto&& vertex : node)
        {
            out.points.push_back(vertex.as<Point2D>());
        }

        // TODO: check for convexity
        // ASSERT(); 

        RCLCPP_INFO(LOGGER, "Loaded Polygon.");
        return true;
    }
};

template<>
struct convert<MissionActivationArea> {
    /**
     * activation_area:
     *   active: true
     *   bboxes:
     *     - ...
     *     - ...
     * @param node
     * @param out
     * @return
     */
    static bool decode(const Node &node, MissionActivationArea &out) {
        if (node[ID_CONDITION_ACTIVE]) {
            out.active = node[ID_CONDITION_ACTIVE].as<bool>();
        } else {
            RCLCPP_ERROR(LOGGER, "Listed condition must have the 'active' specifier.");
            return false;
        }
        
        if (out.active) {
            ASSERT(node.size() >= 2); // active and (bboxes and/or polygon)
            ASSERT(node.size() <= 3); // active and (bboxes and/or polygon)

            // active = true, everything else must be ok
            auto bboxes = node[ID_ACTIVATION_AREA_BOUNDING_BOXES];
            auto polygons = node[ID_ACTIVATION_AREA_POLYGONS];

            ASSERT(bboxes || polygons);
            
            if (bboxes) {
                ASSERT(bboxes.IsSequence());
                ASSERT(bboxes.size() > 0);

                for (auto &&bbox : bboxes) {
                    out.bounding_boxes.push_back(bbox.as<BBox>());
                }
            }

            if (polygons) {
                ASSERT(polygons.IsSequence());
                ASSERT(polygons.size() > 0);

                for (auto &&polygon : polygons) {
                    out.polygons.push_back(polygon.as<Polygon>());
                }
            }

            RCLCPP_INFO(LOGGER, "Loaded MissionActivationArea.bounding_boxes.size(): %ld", out.bounding_boxes.size());
        }
        return true;
    }
};

template<>
struct convert<MissionActivationVelocity> {
    /**
     * activation_velocity:
     *   active: true
     *   range_mps:
     *     - 0.0 # min m/s
     *     - 8.0 # max m/s
     * @param node
     * @param out
     * @return
     */
    static bool decode(const Node &node, MissionActivationVelocity &out) {
        if (node[ID_CONDITION_ACTIVE]) {
            out.active = node[ID_CONDITION_ACTIVE].as<bool>();
        } else {
            RCLCPP_ERROR(LOGGER, "Listed condition must have the 'active' specifier.");
            return false;
        }
        if (out.active) {
            ASSERT(node.size() == 2); // active and range_mps

            auto range_mps = node[ID_ACTIVATION_VELOCITY_RANGE];
            ASSERT(range_mps);
            ASSERT(range_mps.IsSequence());
            ASSERT(range_mps.size() == 2);

            out.range_min_mps = range_mps[0].as<double>();
            out.range_max_mps = range_mps[1].as<double>();

            RCLCPP_INFO(LOGGER, "Loaded MissionActivationVelocity.range_min_mps: %f", out.range_min_mps);
            RCLCPP_INFO(LOGGER, "Loaded MissionActivationVelocity.range_max_mps: %f", out.range_max_mps);
        }
        return true;
    }
};

template<>
struct convert<MissionAllowedTransitions> {
    /**
     * allowed_transitions:
     *   active: false # ignore
     *   from:
     *     - 0 # RACE
     *     - 50 # STANDBY
     * @param node
     * @param out
     * @return
     */
    static bool decode(const Node &node, MissionAllowedTransitions &out) {
        if (node[ID_CONDITION_ACTIVE]) {
            out.active = node[ID_CONDITION_ACTIVE].as<bool>();
        } else {
            RCLCPP_ERROR(LOGGER, "Listed condition must have the 'active' specifier.");
            return false;
        }
        if (out.active) {
            ASSERT(node.size() == 2); // active and from

            auto from = node[ID_ALLOWED_TRANSITIONS_FROM];
            ASSERT(from);
            ASSERT(from.IsSequence());
            ASSERT(from.size() > 0);

            out.from = from.as<std::vector<
                    nif_msgs::msg::MissionStatus::_mission_status_code_type>>();
            for (auto &&mission_code : out.from) {
                ASSERT(nif::common::msgs::isMissionCodeInRange(mission_code));
            }

            RCLCPP_INFO(LOGGER, "Loaded MissionAllowedTransitions.from.size(): %ld", out.from.size());
        }
        
        return true;
    }
};

template<>
struct convert<MissionTimeout> {
    /**
     * timeout:
     *   active: false # ignored
     *   duration_ms: 0
     * @param node
     * @param out
     * @return
     */
    static

    bool decode(const Node &node, MissionTimeout &out) {
        if (node[ID_CONDITION_ACTIVE]) {
            out.active = node[ID_CONDITION_ACTIVE].as<bool>();
        } else {
            RCLCPP_ERROR(LOGGER, "Listed condition must have the 'active' specifier.");
            return false;
        }
        if (out.active) {
            ASSERT(node.size() == 2); // active and duration_ms

            auto duration_ms = node[ID_TIMEOUT_DURATION];
            ASSERT(duration_ms);
            ASSERT(duration_ms.as<long int>() > 0);

            out.duration_ms = duration_ms.as<long int>();

            RCLCPP_INFO(LOGGER, "Loaded MissionTimeout.duration_ms: %ld", out.duration_ms);
        }
        return true;
    }
};

template<>
struct convert<MissionFallback> {
    /**
     * timeout:
     *   active: false # ignored
     *   mission_code: 50
     * @param node
     * @param out
     * @return
     */
    static

    bool decode(const Node &node, MissionFallback &out) {
        if (node[ID_CONDITION_ACTIVE]) {
            out.active = node[ID_CONDITION_ACTIVE].as<bool>();
        } else {
            RCLCPP_ERROR(LOGGER, "Listed condition must have the 'active' specifier.");
            return false;
        }
        if (out.active) {
            ASSERT(node.size() == 2); // active and mission_code

            auto mission_code_node = node[ID_FALLBACK_MISSION_CODE];
            ASSERT(mission_code_node);
            
            auto mission_code = mission_code_node.as<
                nif_msgs::msg::MissionStatus::_mission_status_code_type>();
            ASSERT(nif::common::msgs::isMissionCodeInRange(mission_code));

            out.mission_code = mission_code;

            RCLCPP_INFO(LOGGER, "Loaded MissionFallback.mission_code: %d", out.mission_code);
        }
        return true;
    }
};

template<>
struct convert<MissionNode> {
    /**
     * - mission_code: 60
     *   active: true
     *   activation_area:
     *      ...
     *   activation_velocity:
     *      ...
     *   allowed_transitions:
     *      ...
     *   timeout:
     *      ...
     * @param node
     * @param out
     * @return
     */
    static bool decode(const Node &node, MissionNode &out) {
        if (node[nif::system::ID_MISSION_CODE] && node[nif::system::ID_CONDITION_ACTIVE]) {
            out.active = node[nif::system::ID_CONDITION_ACTIVE].as<bool>();
            out.mission_code = node[nif::system::ID_MISSION_CODE].as<nif_msgs::msg::MissionStatus::_mission_status_code_type>();
        } else {
            RCLCPP_ERROR(LOGGER, "Listed mission must have the 'active' specifier.");
            return false;
        }

        if (out.active) {
            ASSERT(nif::common::msgs::isMissionCodeInRange(out.mission_code));
            ASSERT(node.size() >= 2); // mission_code and active
            if (node[nif::system::ID_ACTIVATION_AREA]) {
                out.activation_area = node[nif::system::ID_ACTIVATION_AREA].as<MissionActivationArea>();
            }
            if (node[nif::system::ID_ACTIVATION_VELOCITY]) {
                out.activation_velocity = node[nif::system::ID_ACTIVATION_VELOCITY].as<MissionActivationVelocity>();
            }
            if (node[nif::system::ID_ALLOWED_TRANSITIONS]) {
                out.allowed_transitions = node[nif::system::ID_ALLOWED_TRANSITIONS].as<MissionAllowedTransitions>();
            }
            if (node[nif::system::ID_TIMEOUT]) {
                out.timeout = node[nif::system::ID_TIMEOUT].as<MissionTimeout>();
            }
            if (node[nif::system::ID_FALLBACK]) {
                out.fallback = node[nif::system::ID_FALLBACK].as<MissionFallback>();
            }
            RCLCPP_INFO(LOGGER, "Loaded MissionNode.mission_code: %d", out.mission_code);
        }
        return true;
    }
};

template<>
struct convert<MissionsDescription> {
    /**
     * missions:
     *   - ...
     * @param node
     * @param out
     * @return
     */
    static bool decode(const Node &node, MissionsDescription &out) 
    {
        auto missions = node[ID_MISSIONS_LIST];

        ASSERT(missions);
        ASSERT(missions.size() > 0);

        for (auto &&mission : missions)
        {
            auto mission_node = mission.as<MissionNode>();
            // Assert unique mission_code
            ASSERT(out.find(mission_node.mission_code) == out.end());

            out.insert({ mission_node.mission_code, mission_node });
        }

        RCLCPP_INFO(LOGGER, "Loaded MissionsDescription.");
        return true;
    }
};

template<>
struct convert<TrackZoneDynamics> {
    /**
     * long_acceleration_max: 
     * long_acceleration_min: 
     * - ...
     * @param node
     * @param out
     * @return
     */
    static bool decode(const Node &node, TrackZoneDynamics &out) 
    {
        if (node[ID_ZONE_DYNAMICS_LONG_ACCELERATION_MAX] && node[ID_ZONE_DYNAMICS_LONG_ACCELERATION_MIN]) {
            out.long_acceleration_max = node[ID_ZONE_DYNAMICS_LONG_ACCELERATION_MAX].as<double>();
            out.long_acceleration_min = node[ID_ZONE_DYNAMICS_LONG_ACCELERATION_MIN].as<double>();
        } else {
            RCLCPP_ERROR(LOGGER, "long_acceleration_max and long_acceleration_min must be specified in TrackZoneDynamics.");
            return false;
        }

        RCLCPP_INFO(LOGGER, "Loaded TrackZoneDynamics");
        return true;
    }
};

template<>
struct convert<TrackZone> {
    /**
     * zone_id:
     * bbox:
     * - ...
     * @param node
     * @param out
     * @return
     */
    static bool decode(const Node &node, TrackZone &out) 
    {
        if (node[nif::system::ID_ZONE_ID] && node[ID_ZONE_TYPE]) {
            out.id = node[nif::system::ID_ZONE_ID].as<track_zone_id_t>();
            out.type = node[nif::system::ID_ZONE_TYPE].as<track_zone_type_t>();
        } else {
            RCLCPP_ERROR(LOGGER, "Listed zone must have the 'id' and 'type' specifiers.");
            return false;
        }

        ASSERT(node[ID_ZONE_BBOX] || node[ID_ZONE_POLYGON]);
        // Prioritize polygon over bbox, but keep only one of the two.
        if (node[ID_ZONE_POLYGON]) {
            out.polygon = node[ID_ZONE_POLYGON].as<Polygon>();
            out.has_polygon = true;

        } else if (node[ID_ZONE_BBOX]) {
            out.bbox = node[ID_ZONE_BBOX].as<BBox>();
            out.has_bbox = true;
        }
        
        if (node[ID_ZONE_DYNAMICS]) {
            out.dynamics = node[ID_ZONE_DYNAMICS].as<TrackZoneDynamics>();
        }

        RCLCPP_INFO(LOGGER, "Loaded TrackZone.id: %d", out.id);
        return true;
    }
};

template<>
struct convert<TrackZonesDescription> {
    /**
     * zones:
     *   - ...
     * @param node
     * @param out
     * @return
     */
    static bool decode(const Node &node, TrackZonesDescription &out) 
    {
        auto zones = node[ID_ZONES_LIST];

        ASSERT(zones);
        ASSERT(zones.IsSequence());
        ASSERT(zones.size() > 0);

        for (auto &&zone : zones)
        {
            auto zone_node = zone.as<TrackZone>();
            // Assert unique zone_id
            ASSERT(out.find(zone_node.id) == out.end());

            out.insert({ zone_node.id, zone_node });
        }

        RCLCPP_INFO(LOGGER, "Loaded ZonesDescription.");
        return true;
    }
};

} // namespace YAML

#endif //ROS2MASTER_YAML_CPP_ADAPTER_H
