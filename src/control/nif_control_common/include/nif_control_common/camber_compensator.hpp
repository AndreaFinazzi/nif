#ifndef CAMBER_COMPENSATOR_H_
#define CAMBER_COMPENSATOR_H_

#include <iostream>

namespace nif {
namespace control {

/// @enum CAMBERCOMPESATORMODE
/// @brief A camber angle setup compensator for AV-21(INDY car).
enum CAMBERCOMPESATORMODE {
  CONSTANT_BIAS, ///< Add constant bias to the steering cmd
  FIRST_ORDER,   ///< Add bias which is a first order function of vehicle speed
                 ///< to the steering cmd
  EXPONENTIAL    ///< Add bias which is a exponential shape function of vehicle
              ///< speed (The shape depends on the vehicle hardware setup. -->
              ///< can be logarithmic shape)
};

class CamberCompensator {
public:
private:
}
} // namespace control
} // namespace nif

#endif
