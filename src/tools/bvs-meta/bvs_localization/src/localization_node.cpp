/**
 * @brief node to convert between inspva messages and odometry
 * @note this probably belongs somewhere else... but I'm just trying
 *  to get code up rn
 **/

// Application
#include "bvs_utils/geodetic_conv.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "novatel_gps_msgs/msg/inspva.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


using bvs_utils::GeodeticConverter;

namespace bvs_localization {

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode() : Node("LocalizationNode") {
        this->declare_parameter("subscribe_novatel_oem7_msgs", true);
        this->declare_parameter("subscribe_novatel_gps_msgs", false);

        bool subscribe_novatel_oem7_msgs = this->get_parameter("subscribe_novatel_oem7_msgs").as_bool();
        bool subscribe_novatel_gps_msgs = this->get_parameter("subscribe_novatel_gps_msgs").as_bool();

        // Subscription to input odom info
        if(subscribe_novatel_gps_msgs && subscribe_novatel_oem7_msgs) {
            RCLCPP_WARN(get_logger(), "Subscribed to both oem7 and gps message types, that doesn't seem right.");
        }
        if(subscribe_novatel_oem7_msgs) {
            oem7_inspva_sub_ = this->create_subscription<novatel_oem7_msgs::msg::INSPVA>("novatel_oem7_msgs/inspva",
                                1, std::bind(&LocalizationNode::oem7InspvaCallback, this, std::placeholders::_1));
        } else if(subscribe_novatel_gps_msgs) {
            gps_inspva_sub_ = this->create_subscription<novatel_gps_msgs::msg::Inspva>("novatel_gps_msgs/inspva",
                                1, std::bind(&LocalizationNode::gpsInspvaCallback, this, std::placeholders::_1));
        } else {
            RCLCPP_ERROR(get_logger(), "No gps source subscribed :(");
        }

        // Output odometry publishers
        ltp_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("bvs_localization/ltp_odom", 1);
        ltp_origin_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("bvs_localization/ltp_origin", 1);
        tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 1);

        // Publish the local tangent plane origin at 2 Hz
        ltp_origin_pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&LocalizationNode::publishLTPOrigin, this));

        // Parameters
        this->declare_parameter("ltp_latitude", 39.809786);
        this->declare_parameter("ltp_longitude", -86.235148);
        this->declare_parameter("ltp_altitude", 0.);
        this->declare_parameter("ltp_frame", "ltp");
        this->declare_parameter("base_link_frame", "base_link");

        // Set the ltp reference point
        GeodeticConverter::GeoRef ref;
        ref.latitude = this->get_parameter("ltp_latitude").as_double();
        ref.longitude = this->get_parameter("ltp_longitude").as_double();
        ref.altitude = this->get_parameter("ltp_altitude").as_double();
        conv_.initializeReference(ref);

        ltp_frame_ = this->get_parameter("ltp_frame").as_string();
        base_link_frame_ = this->get_parameter("base_link_frame").as_string();
        ltp_origin_msg_.header.frame_id = ltp_frame_;
        ltp_origin_msg_.point.x = ref.latitude;
        ltp_origin_msg_.point.y = ref.longitude;
        ltp_origin_msg_.point.z = ref.altitude;
    };

    void publishLTPOrigin() {
        ltp_origin_pub_->publish(ltp_origin_msg_);
    }

    void oem7InspvaCallback(const novatel_oem7_msgs::msg::INSPVA::SharedPtr msg) {
        publish(*msg, msg->nov_header.gps_week_number, msg->nov_header.gps_week_milliseconds);
    }

    void gpsInspvaCallback(const novatel_gps_msgs::msg::Inspva::SharedPtr msg) {
        publish(*msg, msg->week, msg->seconds);
    }

    template<typename GPSMessage>
    void publish(const GPSMessage message, uint32_t week, double seconds) {
        // https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSPVA.htm
        // First generate the epoch timestamp (https://novatel.com/support/knowledge-and-learning/unit-conversions)
        const double epoch_to_gnss = 3657.*24.*60.*60.; // seconds between january 1, 1970 and january 6, 1980
        const double week_runover = 604800. * 1024. * 2.; // seconds in 2046 weeks
        double time_stamp = epoch_to_gnss + week_runover + 604800. * week + seconds;

        GeodeticConverter::GeoRef ref;
        ref.latitude = message.latitude;
        ref.longitude = message.longitude;
        // Currently ignore altitude for the most part and just track x/y
        ref.altitude = 0.;

        GeodeticConverter::CartesianPoint ltp_pt;
        conv_.geodetic2Ned(ref, ltp_pt);

        // Publish Odometry
        nav_msgs::msg::Odometry ltp_odom;
        ltp_odom.header.frame_id = ltp_frame_;
        ltp_odom.header.stamp = rclcpp::Time(time_stamp);
        ltp_odom.child_frame_id = base_link_frame_;
        ltp_odom.pose.pose.position.x = ltp_pt.x;
         // We convert from NED to FLU
        ltp_odom.pose.pose.position.y = -ltp_pt.y;
        ltp_odom.pose.pose.position.z = -ltp_pt.z;
        ltp_odom.pose.pose.orientation.x = 0.;
        ltp_odom.pose.pose.orientation.y = 0.;
        double yaw = conv_.deg2Rad(-message.azimuth);
        ltp_odom.pose.pose.orientation.z = std::sin(yaw / 2.);
        ltp_odom.pose.pose.orientation.w = std::cos(yaw / 2.);
        // Since we are targetting an FLU frame, y velocity = -msg->east_velocity
        ltp_odom.twist.twist.linear.x = message.north_velocity * std::cos(-yaw) + message.east_velocity * std::sin(-yaw);
        ltp_odom.twist.twist.linear.y = message.north_velocity * std::sin(-yaw) - message.east_velocity * std::cos(-yaw);
        ltp_odom.twist.twist.linear.z = message.up_velocity;
        ltp_odom_pub_->publish(ltp_odom);

        // Publish Transform
        tf2_msgs::msg::TFMessage tf_message;
        geometry_msgs::msg::TransformStamped tf_stamped;
        tf_stamped.header = ltp_odom.header;
        tf_stamped.child_frame_id = ltp_odom.child_frame_id;
        tf_stamped.transform.translation.x = ltp_odom.pose.pose.position.x;
        tf_stamped.transform.translation.y = ltp_odom.pose.pose.position.y;
        tf_stamped.transform.translation.z = ltp_odom.pose.pose.position.z;
        tf_stamped.transform.rotation = ltp_odom.pose.pose.orientation;
        tf_message.transforms.push_back(tf_stamped);
        tf_pub_->publish(tf_message);
    }

private:
    //! Misc ROS interface
    rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::SharedPtr oem7_inspva_sub_;
    rclcpp::Subscription<novatel_gps_msgs::msg::Inspva>::SharedPtr gps_inspva_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ltp_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ltp_origin_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

    //! Publish the ltp origin at some rate
    rclcpp::TimerBase::SharedPtr ltp_origin_pub_timer_;

    //! Convert from geodetic to odometry
    GeodeticConverter conv_;
    geometry_msgs::msg::PointStamped ltp_origin_msg_;

    //! Frame Names
    std::string ltp_frame_;
    std::string base_link_frame_;

}; /* class LocalizationNode */

} /* namespace bvs_localization */

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bvs_localization::LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
