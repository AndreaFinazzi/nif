//
// Created by usrg on 6/18/21.
//

#ifndef NIF_COMMON_NODES_BASENODE_H
#define NIF_COMMON_NODES_BASENODE_H

#include <string>

#include "nif_common/types.h"
#include "nif_utils/utils.h"
#include "../../../nif_common/include/nif_common/types.h"

#include <rclcpp/rclcpp.hpp>

namespace nif {
    namespace common {

        class IBaseNode : rclcpp::Node {
        public:

        protected:
            IBaseNode(const std::string &node_name, const rclcpp::NodeOptions &options);

            /// Expose time to children

            rclcpp::Time gclock_node_init;

            nif::common::msgs::VehicleKinematicState ego_vehicle_state;

//            TODO : finalize SystemState class
            nif::common::msgs::SystemState system_state;

//          TODO : finalize RaceControlState class
            nif::common::msgs::RaceControlState race_control_state;

//  Reference to utils not needed, as it'll be everything static (probably)
//  nif::common::utils:: utils;

        private:
            /**
             * The default constructor is hidden from the outside to prevent unnamed nodes.
             */
            IBaseNode();

            rclcpp::Subscription<nif::common::msgs::VehicleKinematicState>::SharedPtr ego_vehicle_state_sub;
            rclcpp::Subscription<nif::common::msgs::SystemState>::SharedPtr system_state_sub;
            rclcpp::Subscription<nif::common::msgs::RaceControlState>::SharedPtr race_control_state_sub;

            virtual void declareParameters() = 0;
            virtual void getParameters() = 0;

            void egoVehicleStateCallback(const nif::common::msgs::VehicleKinematicState::SharedPtr msg);
            void systemStateCallback(const nif::common::msgs::SystemState::SharedPtr msg);
            void raceControlStateCallback(const nif::common::msgs::RaceControlState::SharedPtr msg);


        };

    }
}
#endif // NIF_COMMON_NODES_BASENODE_H