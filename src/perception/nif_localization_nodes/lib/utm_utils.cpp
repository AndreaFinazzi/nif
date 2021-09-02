#include "utm_utils.h"

using namespace utm_utils;

UtmProjector::UtmProjector(sensor_msgs::msg::NavSatFix& origin) : m_isInNorthernHemisphere(true), m_origin(origin)
{
  GeographicLib::UTMUPS::Forward(m_origin.latitude, m_origin.longitude, m_zone,
                                 m_isInNorthernHemisphere, m_utmOrigin.x, m_utmOrigin.y);

}

geometry_msgs::msg::Pose2D UtmProjector::forward(const sensor_msgs::msg::NavSatFix& gps){

  geometry_msgs::msg::Pose2D utmXY;

  if(!m_isInNorthernHemisphere)
    RCLCPP_WARN(rclcpp::get_logger("utm_utils"), "Is it Southern Hemisphere?");

  try {
    GeographicLib::UTMUPS::Forward(gps.latitude, gps.longitude, m_zone, m_isInNorthernHemisphere, utmXY.x, utmXY.y);

    // printf("utm Data: \n x: %.9f ,y: %.9f \n" , utmXY.x, utmXY.y);
    // printf("origin Data: \n x: %.9f ,y: %.9f \n" , m_utmOrigin.x, m_utmOrigin.y);

    utmXY.x = utmXY.x - m_utmOrigin.x;
    utmXY.y = utmXY.y - m_utmOrigin.y;

  } catch (GeographicLib::GeographicErr& e) {
    RCLCPP_WARN(rclcpp::get_logger("utm_utils"), "GeographicLib FORWARD ERROR");

  }

  return utmXY;
}

sensor_msgs::msg::NavSatFix UtmProjector::reverse(const geometry_msgs::msg::Pose2D& utm){

  sensor_msgs::msg::NavSatFix GpsRaw;
  geometry_msgs::msg::Pose2D utmXY; 

  if(!m_isInNorthernHemisphere)
    RCLCPP_WARN(rclcpp::get_logger("utm_utils"), "Is it Southern Hemisphere?");

  try { 
    utmXY.x = utm.x + m_utmOrigin.x;
    utmXY.y = utm.y + m_utmOrigin.y;
    GeographicLib::UTMUPS::Reverse(m_zone, m_isInNorthernHemisphere, utmXY.x, utmXY.y, GpsRaw.latitude, GpsRaw.longitude);
  } catch (GeographicLib::GeographicErr& e) {
    RCLCPP_WARN(rclcpp::get_logger("utm_utils"), "UTM Reverse Error");
  }

  return GpsRaw;
}