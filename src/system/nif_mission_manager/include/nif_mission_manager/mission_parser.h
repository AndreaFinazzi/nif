//
// Created by usrg on 9/30/21.
//

#ifndef ROS2MASTER_MISSION_PARSER_H
#define ROS2MASTER_MISSION_PARSER_H

#include "nif_mission_manager/yaml_cpp_adapter.h"

/**
 * Yaml template
 *
 */

namespace nif {
namespace system {
namespace MissionParser {

void loadMissionsDescription(const std::string & file_name, MissionsDescription & out) {
    YAML::Node config = YAML::LoadFile(file_name);

    ASSERT(config);
    out = config.as<MissionsDescription>();
    
}

void loadZonesDescription(const std::string & file_name, TrackZonesDescription & out) {
    YAML::Node config = YAML::LoadFile(file_name);

    ASSERT(config);
    out = config.as<TrackZonesDescription>();
    
}


} // namespace MisisonParser
} // namespace system
} // namespace nif

#endif //ROS2MASTER_MISSION_PARSER_H
