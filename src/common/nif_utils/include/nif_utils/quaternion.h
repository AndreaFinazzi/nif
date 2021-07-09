#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <math.h>

class Quaternion {
public:
  // roll (X), pitch (Y), yaw (Z)
  Quaternion(double yaw, double pitch, double roll);
  Quaternion() {}

  double x() {
    return x_;
  }
  double y() {
    return y_;
  }
  double z() {
    return z_;
  }
  double w() {
    return w_;
  }

private:
  double x_;
  double y_;
  double z_;
  double w_;
};

#endif // QUATERNION_H_