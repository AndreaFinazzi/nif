#include "nif_opponent_prediction_nodes/quaternion.h"

#include <math.h>

Quaternion::Quaternion(double yaw, double pitch, double roll) {
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  w_ = cy * cp * cr + sy * sp * sr;
  x_ = cy * cp * sr - sy * sp * cr;
  y_ = sy * cp * sr + cy * sp * cr;
  z_ = sy * cp * cr - cy * sp * sr;
}