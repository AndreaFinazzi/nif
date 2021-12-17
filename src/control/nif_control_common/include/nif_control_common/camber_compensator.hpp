#ifndef CAMBER_COMPENSATOR_H_
#define CAMBER_COMPENSATOR_H_

// Commented at Dec 10th (always right side of the waypoint in the low-speed range)
// #define DEFAULT_MAX_COMPENSATE_MAG_DEG 4.0
// #define DEFAULT_CONST_COMPENSATE_MAG_DEG 2.0
#define DEFAULT_MAX_COMPENSATE_MAG_DEG 3.0
#define DEFAULT_MIN_COMPENSATE_MAG_DEG 1.0
#define DEFAULT_CONST_COMPENSATE_MAG_DEG 1.0
#define DEFAULT_MAX_SPEED_MPS 60.0

#include <iostream>
#include <limits>

namespace nif
{
  namespace control
  {

    /// @enum CAMBERCOMPESATORMODE
    /// @brief A camber angle setup compensator for AV-21(INDY car).
    enum CAMBERCOMPESATORMODE
    {
      CONSTANT_BIAS, ///< Add constant bias to the steering cmd
      FIRST_ORDER,   ///< Add bias which is a first order function of vehicle speed
                     ///< to the steering cmd
      EXPONENTIAL    ///< Add bias which is a exponential shape function of vehicle
                     ///< speed (The shape depends on the vehicle hardware setup. -->
                     ///< can be logarithmic shape)
    };

    class CamberCompensator
    {
    public:
      /// @brief This is origianlly designed for Indy AV-21 which has a left camber
      /// setup. --> when steering cmd is zero, the car goes to left side.
      CamberCompensator(
          CAMBERCOMPESATORMODE mode_,
          bool sign_flip_); // default is "CONSTANT_BIAS" mode / sign_flip == false

      void setVehSpeed(double &veh_speed_mps_);
      void setBankAngle(double &bank_angle_rad_);
      void setSignFlip(bool &is_flip_);
      void setCompMode(CAMBERCOMPESATORMODE &mode_);
      double getCamberCompensation();

    private:
      CAMBERCOMPESATORMODE m_compesator_mode;
      double m_veh_speed; // [mps]
      bool m_sign_flip;
      int m_sign_multiplier;
      bool m_bank_angle_rad;

      double m_max_compensate_mag; // [deg] : must be positive
      double m_compensated_result; // [deg]
    };
  } // namespace control
} // namespace nif

#endif
