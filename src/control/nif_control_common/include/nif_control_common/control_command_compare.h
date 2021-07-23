//
// Created by usrg on 7/18/21.
//

#ifndef ROS2MASTER_CONTROL_COMMAND_COMPARE_H
#define ROS2MASTER_CONTROL_COMMAND_COMPARE_H

#include <nif_common/types.h>

using nif::common::msgs::ControlCmd;

namespace nif {
namespace control {

/**
 * Compare ControlCmd based on order value. In case of `(a.order == b.order)`, return true (i.e. `a` comes before `b`)
 */
class ControlCommandCompare {
public:
  ControlCommandCompare(){};
  ~ControlCommandCompare(){};

  bool operator()(ControlCmd &o1, ControlCmd &o2) const;
  bool operator()(const ControlCmd &o1, const ControlCmd &o2) const;

  bool operator()(ControlCmd::SharedPtr &o1, ControlCmd::SharedPtr &o2) const;
  bool operator()(const ControlCmd::SharedPtr &o1,
                  const ControlCmd::SharedPtr &o2) const;
};

}
}
#endif // ROS2MASTER_CONTROL_COMMAND_COMPARE_H
