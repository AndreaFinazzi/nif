/**
 * @brief node to publish the trajectory at 5 Hz after wrapping
 *  the path to start near the current odometry
 **/

// Application
#include "bvs_utils/geodetic_conv.h"
#include "bvs_utils/path_util.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using bvs_utils::GeodeticConverter;
using bvs_utils::PathUtil;

namespace bvs_utils {

class PathPublisherNode : public rclcpp::Node {
public:
    PathPublisherNode() : Node("PathPublisherNode") {
        // Publisher
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("bvs_controller/target_path", 1);

        // Subscriber
        ltp_origin_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("bvs_localization/ltp_origin",
            1, std::bind(&PathPublisherNode::ltpOriginCallback, this, std::placeholders::_1));
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("bvs_localization/ltp_odom",
            rclcpp::SensorDataQoS{}, std::bind(&PathPublisherNode::odometryCallback, this, std::placeholders::_1));

        // Publish a path at 5 Hz
        traj_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200), std::bind(&PathPublisherNode::publishTrajectory, this));

        // Parameters
        this->declare_parameter("odom_timeout", 0.5);
        this->declare_parameter("track_line_csv", "");

        // Get Parameters
        odom_timeout_ = this->get_parameter("odom_timeout").as_double();
        track_line_csv_ = this->get_parameter("track_line_csv").as_string();

        RCLCPP_INFO(get_logger(), "Loading path from %s once ltp is received.", track_line_csv_.c_str());
    };

    void publishTrajectory() {
        auto now = rclcpp::Clock().now();
        // Make sure the path / ltp is initialized and I have current odometry
        if(ltp_origin_received_ && (secs(now - odom_update_time_) < odom_timeout_ || odom_timeout_ < 0.0)) {
            auto local_path = path_.wrapPathTo(current_odometry_.pose.pose);
            local_path.header.frame_id = ltp_frame_;
            local_path.header.stamp = now;
            path_pub_->publish(local_path);
        }
    }

    void ltpOriginCallback(const geometry_msgs::msg::PointStamped::SharedPtr message) {
        if(ltp_origin_received_) {
            return;
        }

        // Set up converter
        GeodeticConverter::GeoRef ref;
        ref.latitude = message->point.x;
        ref.longitude = message->point.y;
        ref.altitude = message->point.z;
        conv_.initializeReference(ref);

        // Load the path :)
        path_.setConverter(conv_);
        path_.loadPath(track_line_csv_, message->header.frame_id);
        ltp_frame_ = message->header.frame_id;
        RCLCPP_INFO(get_logger(), "Loaded %d points from %s", path_.getPath().poses.size(), track_line_csv_.c_str());
        ltp_origin_received_ = true;
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message) {
        odom_update_time_ = rclcpp::Clock().now();
        current_odometry_ = *message;
    }

private:
    //! Misc ROS interface
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ltp_origin_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr traj_publish_timer_;

    //! Convert from geodetic to odometry
    bool ltp_origin_received_;
    std::string ltp_frame_;
    GeodeticConverter conv_;

    //! Store the current odometry
    nav_msgs::msg::Odometry current_odometry_;
    rclcpp::Time odom_update_time_;
    double odom_timeout_;

    //! Track Path
    std::string track_line_csv_;
    PathUtil path_;

    double secs(rclcpp::Duration t) {
        return static_cast<double>(t.seconds()) + static_cast<double>(t.nanoseconds())*1e-9;
    }

}; /* class PathPublisherNode */

} /* namespace bvs_localization */

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bvs_utils::PathPublisherNode>());
    rclcpp::shutdown();
    return 0;
}