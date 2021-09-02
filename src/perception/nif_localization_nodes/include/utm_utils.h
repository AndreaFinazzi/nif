#include <GeographicLib/UTMUPS.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "rclcpp/rclcpp.hpp"


namespace utm_utils {

class UtmProjector 
{
 public:
  UtmProjector(sensor_msgs::msg::NavSatFix& origin);
  geometry_msgs::msg::Pose2D forward(const sensor_msgs::msg::NavSatFix& gps);
  sensor_msgs::msg::NavSatFix reverse(const geometry_msgs::msg::Pose2D& utm);

  sensor_msgs::msg::NavSatFix m_origin;
  geometry_msgs::msg::Pose2D m_utmOrigin;

 private:
  int m_zone;
  bool m_isInNorthernHemisphere;
};

} //utm_utils 