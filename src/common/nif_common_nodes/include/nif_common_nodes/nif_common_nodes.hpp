//
// Created by usrg on 7/28/21.
//

// NECESSARY TO SOLVE CIRCULAR DEPENDENCIES

#ifndef NIF_COMMON_NODES_H
#define NIF_COMMON_NODES_H

#include <rclcpp/rclcpp.hpp>

namespace nif {
    namespace common
    {

        // enum NodeStatusCode : std::uint8_t {
        //     OK = 0,
        //     INITIALIZED = 1,
        //     NOT_INITIALIZED = 200,
        //     FATAL_ERROR = 254,
        //     DEAD = 255
        // };

        // enum NodeType : std::int8_t {
        //     PERCEPTION,
        //     PLANNING,
        //     PREDICTION,
        //     CONTROL,
        //     TOOL,
        //     SYSTEM
        // };
        
        class IBaseNode;
        class NodeStatus;
        class NodeStatusManager;
    } // namespace common
}


#include "nif_common_nodes/i_base_node.h"
#include "nif_common_nodes/i_base_synchronized_node.h"
#include "nif_common_nodes/node_status.h"
#include "nif_common_nodes/node_status_manager.h"

#endif // NIF_COMMON_NODES_H
