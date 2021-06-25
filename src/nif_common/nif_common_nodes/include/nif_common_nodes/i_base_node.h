//
// Created by usrg on 6/18/21.
//

#ifndef NIF_COMMON_NODES_BASENODE_H
#define NIF_COMMON_NODES_BASENODE_H

#include <rclcpp/rclcpp.hpp>
#include <string>
#include "nif_utils/utils_geometry.h"

namespace nif {
    namespace common {

        class IBaseNode : rclcpp::Node {
        public:

        protected:
            IBaseNode(const std::string &node_name, const rclcpp::NodeOptions &options)
                    : Node(node_name, options) {
                gclock_node_init = this->now();
            }

            /// Expose time to children

            rclcpp::Time gclock_node_init;

            nif::common::VehicleState ego_vehicle_state;
            nif::common::SystemState system_state;
            nif::common::RaceControlState race_control_state;

//  Reference to utils not needed, as it'll be everything static (probably)
//  nif::common::utils:: utils;

        private:
            /**
             * The default constructor is hidden from the outside to prevent unnamed nodes.
             */


            IBaseNode() : Node("no_name_node") {
                throw std::invalid_argument("Cannot construct IBaseNode without specifying node_name. Creating empty node.");
            }


            rclcpp::Subscription<VehicleState>::SharedPtr vehicle_state_sub;
            rclcpp::Subscription<SystemState>::SharedPtr system_state_sub;
            rclcpp::Subscription<RaceControlState>::SharedPtr race_control_state_sub;

//  virtual void declareParameters() = 0;
//  virtual void getParameters() = 0;

            void vehicleStateCallback(const EgoVehicleState::SharedPtr & msg);
            void systemStateCallback(const SystemState::SharedPtr & msg);
            void raceControlStateCallback(const RaceControlState::SharedPtr & msg);


        };

    }
}
#endif // NIF_COMMON_NODES_BASENODE_H
