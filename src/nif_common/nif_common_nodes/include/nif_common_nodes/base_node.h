//
// Created by usrg on 6/18/21.
//

#ifndef NIF_COMMON_NODES_BASENODE_H
#define NIF_COMMON_NODES_BASENODE_H

#include <rclcpp/rclcpp.h>
#include <string>

class BaseNode {
public:
  BaseNode(std::string node_name);

  void paramCallback(Collection<k> ps)
  void vehicleStateCallback(EgoVehicleState s);
  void systemStateCallback(SystemState s);
  void raceControlStateCallback(RaceControlState s);

};

#endif // NIF_COMMON_NODES_BASENODE_H
