#include "nif_control_common/camber_compensator.hpp"

using namespace nif::control;

template <typename T>
int sgn(T val) { return (T(0) < val) - (val < T(0)); }

CamberCompensator::CamberCompensator(CAMBERCOMPESATORMODE mode_ = CAMBERCOMPESATORMODE::CONSTANT_BIAS,
                                     bool sign_flip_ = false)
    : m_compesator_mode(mode_), m_sign_flip(sign_flip_)
{
  m_sign_multiplier = 1;
  if (m_sign_flip == true)
  {
    m_sign_multiplier = -1;
  }

  m_max_compensate_mag = DEFAULT_MAX_COMPENSATE_MAG_DEG;
}

void CamberCompensator::setVehSpeed(double &veh_speed_mps_)
{
  m_veh_speed = veh_speed_mps_;
}
void CamberCompensator::setBankAngle(double &bank_angle_rad_)
{
  m_bank_angle_rad = bank_angle_rad_;
}
void CamberCompensator::setSignFlip(bool &is_flip_)
{
  // if sign flip is true, output value always negative
  // In AV-21 case, whehn the steering angle is positive, the vehicle goes to left.
  m_sign_flip = is_flip_;
  m_sign_multiplier = 1;
  if (m_sign_flip == true)
  {
    m_sign_multiplier = -1;
  }
}
void CamberCompensator::setCompMode(CAMBERCOMPESATORMODE &mode_)
{
  if (mode_ == CAMBERCOMPESATORMODE::CONSTANT_BIAS ||
      mode_ == CAMBERCOMPESATORMODE::FIRST_ORDER ||
      mode_ == CAMBERCOMPESATORMODE::EXPONENTIAL)
  {
    m_compesator_mode = mode_;
  }
  else
  {
    std::cout << "Invalid compesator mode. Set to Defult : CONSTANT_BIAS mode"
              << std::endl;
    m_compesator_mode = CAMBERCOMPESATORMODE::CONSTANT_BIAS;
  }
}
double CamberCompensator::getCamberCompensation()
{

  // TODO : do something
  switch (m_compesator_mode)
  {
  case CAMBERCOMPESATORMODE::CONSTANT_BIAS:
  { /* code */
    m_compensated_result = DEFAULT_CONST_COMPENSATE_MAG_DEG;
    break;
  }
  case CAMBERCOMPESATORMODE::FIRST_ORDER:
  {
    double lower_thres_vel = 20;
    double comp_at_lower_thres = DEFAULT_MAX_COMPENSATE_MAG_DEG;
    double higher_thres_vel = 40;
    double comp_at_higher_thres = DEFAULT_MAX_COMPENSATE_MAG_DEG / 2.0;

    if (m_veh_speed < lower_thres_vel)
    {
      m_compensated_result = comp_at_lower_thres;
    }
    else if (m_veh_speed > comp_at_higher_thres)
    {
      m_compensated_result = comp_at_higher_thres;
    }
    else
    {
      m_compensated_result = (comp_at_higher_thres - comp_at_lower_thres) /
                                 (higher_thres_vel - lower_thres_vel) *
                                 (m_veh_speed - lower_thres_vel) +
                             comp_at_lower_thres;
    }
    /* code */
    break;
  }
  case CAMBERCOMPESATORMODE::EXPONENTIAL:
  { //   TODO : here
    /* code */
    break;
  }
  default:
  {
    std::cout << "Invalid compesator mode. Set to Defult : CONSTANT_BIAS mode"
              << std::endl;
    m_compesator_mode = CAMBERCOMPESATORMODE::CONSTANT_BIAS;
    break;
  }
  }

  if (abs(m_compensated_result) > abs(m_max_compensate_mag))
  {
    // clip output
    m_compensated_result =
        m_sign_multiplier * m_max_compensate_mag * sgn(m_compensated_result);
  }
  else
  {
    m_compensated_result = m_sign_multiplier * m_compensated_result;
  }
  return m_compensated_result;
}