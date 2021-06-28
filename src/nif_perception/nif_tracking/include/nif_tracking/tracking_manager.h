//
// Created by usrg on 6/23/21.
//

#ifndef ROS2MASTER_TRACKING_MANAGER_H
#define ROS2MASTER_TRACKING_MANAGER_H

#include <array>
#include "nif_common/types.h"

namespace nif {
    namespace perception {


/**
 * TODO: define precisely its behavior
 * TrackingManager is responsible for assigning an ID to each detected object (from PerceptionNode);
 */
class TrackingManager {
public:

protected:

private:
    /**
     *
     * @param oppo_vehicle_states
     */
    void solve(nif::common::types::t_oppo_collection_states & oppo_vehicle_states);

};


    }
}
#endif //ROS2MASTER_TRACKING_MANAGER_H
