
#include "nif_control_common/control_command_compare.h"

using nif::control::ControlCommandCompare;

bool ControlCommandCompare::operator()(ControlCmd &a, ControlCmd &b) const {

  // There can be any condition
  // implemented as per the need
  // of the problem statement
  return (a.order <= b.order);
}

bool ControlCommandCompare::operator()(const ControlCmd &a,
                                       const ControlCmd &b) const {

  return (a.order <= b.order);
}
bool ControlCommandCompare::operator()(
    nif_msgs::msg::ControlCommand::SharedPtr &a,
    nif_msgs::msg::ControlCommand::SharedPtr &b) const {
  return (a->order <= b->order);
}
bool ControlCommandCompare::operator()(
    const nif_msgs::msg::ControlCommand::SharedPtr &a,
    const nif_msgs::msg::ControlCommand::SharedPtr &b) const {
  return (a->order <= b->order);
}
