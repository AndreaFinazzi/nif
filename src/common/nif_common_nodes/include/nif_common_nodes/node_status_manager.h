//
// Created by usrg on 7/28/21.
//

#ifndef NIFNODESTATUSMANAGER_SYSTEM_STATUS_MANAGER_H
#define NIFNODESTATUSMANAGER_SYSTEM_STATUS_MANAGER_H

#include "nif_common/types.h"
#include "nif_common_nodes/node_status.h"

#include <memory>
#include <vector>

namespace nif {
namespace common {

class NodeStatusManager {
public:
  /**
   * Register the node in the static register. If destroyed, set node status to NodeStatusCode::DEAD.
   */
  explicit NodeStatusManager(const nif::common::IBaseNode&, const nif::common::NodeType);

  void update(nif::common::NodeStatusCode status_code);
  
  ~NodeStatusManager();

//  STATIC

  static const std::unordered_map<nif::common::types::t_node_id,
                                  nif::common::NodeStatus &> &
  getNodeStatusesMap();
  //  NOT A SINGLETON ANYMORE
//  static const NodeStatusManager* getInstance();

private:
  NodeStatusManager();

  nif::common::NodeStatus node_status;

  /**
   * Static list of node statuses, it's the common register for all node statuses.
   */
  static std::unordered_map<nif::common::types::t_node_id, nif::common::NodeStatus&> node_statuses_map;

//  NOT A SINGLETON ANYMORE
//  static const NodeStatusManager* instance;
};

}
}

#endif // NIFNODESTATUSMANAGER_SYSTEM_STATUS_MANAGER_H
