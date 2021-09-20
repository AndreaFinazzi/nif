/**
 * @brief localization executive to input multiple state estimate sources
 *  and use the best one to generate odometry and update a tf
 **/

// Application
#include "bvs_utils/geodetic_conv.h"
#include "bvs_localization/estimate/estimate_merger.h"
#include "bvs_localization/sources/source_factory.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "novatel_gps_msgs/msg/inspva.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


using bvs_utils::GeodeticConverter;

namespace bvs_localization {

class LocalizationExecutive : public rclcpp::Node {
public:
    LocalizationExecutive() : Node("bvs_localization") {
        // Output odometry publishers
        std::string node_name = get_name();
        ltp_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("bvs_localization/merged/odom", 1);
        ltp_origin_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("bvs_localization/ltp_origin", 1);
        tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 1);
        
        // Publish the selected odometry at a high rate (100Hz)
        ltp_odom_pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&LocalizationExecutive::publishLTPOdom, this));
        // Publish the local tangent plane origin at 2 Hz
        ltp_origin_pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&LocalizationExecutive::publishLTPOrigin, this));

        // Parameters
       this->declare_parameter("ltp_latitude", 39.809786);
        this->declare_parameter("ltp_longitude", -86.235148);
        this->declare_parameter("ltp_altitude", 0.);
        this->declare_parameter("ltp_frame", "ltp");
        this->declare_parameter("vehicle_frame", "base_link");
        this->declare_parameter("config_source", "param");

        // Set the ltp reference point
        conv_ = std::make_shared<GeodeticConverter>();
        GeodeticConverter::GeoRef ref;
        ref.latitude = this->get_parameter("ltp_latitude").as_double();
        ref.longitude = this->get_parameter("ltp_longitude").as_double();
        ref.altitude = this->get_parameter("ltp_altitude").as_double();
        conv_->initializeReference(ref);

        ltp_frame_ = this->get_parameter("ltp_frame").as_string();
        vehicle_frame_ = this->get_parameter("vehicle_frame").as_string();
        ltp_origin_msg_.header.frame_id = ltp_frame_;
        ltp_origin_msg_.point.x = ref.latitude;
        ltp_origin_msg_.point.y = ref.longitude;
        ltp_origin_msg_.point.z = ref.altitude;


        config_source_ = this->get_parameter("config_source").as_string();
        sources_initialized_ = false;
    };

    void initializeSources() {
        sources::SourceFactory::Resources resources;
        resources.converter = conv_;
        resources.node = shared_from_this();
        sources::SourceFactory factory(resources);

        sources_ = factory.generate(config_source_);
        estimate_merger_.setPriorities(factory.getSourcePriority());
        merge_info_publisher_.initROSInterface(shared_from_this());
    }

    void publishLTPOdom() {
        if(!sources_initialized_) {
            initializeSources();
            sources_initialized_ = true;
        }
        estimate_merger_.cleanEstimate();
        // return;
        for(auto it = sources_.begin(); it != sources_.end(); ++it) {
            estimate::Estimate estimate;
            auto estimate_status = it->second->getEstimate(estimate);
            it->second->publish(estimate, estimate_status);
            estimate_merger_.addEstimate(it->first, estimate_status, estimate);
        }
        auto estimate_info = estimate_merger_.getEstimate();
        auto estimate = estimate_info.first;
        auto ltp_odom = estimate.toOdom();
        ltp_odom.header.frame_id = ltp_frame_;
        ltp_odom.child_frame_id = vehicle_frame_;
        ltp_odom_pub_->publish(ltp_odom);
        merge_info_publisher_.publish(estimate_info.second);
    
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

    void publishLTPOrigin() {
        ltp_origin_pub_->publish(ltp_origin_msg_);
    }

private:
    //! Misc ROS interface
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ltp_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ltp_origin_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
    
    //! Publish the ltp origin at some rate
    rclcpp::TimerBase::SharedPtr ltp_origin_pub_timer_;
    rclcpp::TimerBase::SharedPtr ltp_odom_pub_timer_;

    //! Source / Estimate Tracking
    bool sources_initialized_;
    std::string config_source_;
    std::map<std::string, estimate::EstimateSource::SharedPtr> sources_;
    estimate::EstimateMerger estimate_merger_;
    estimate::MergeInfoPublisher merge_info_publisher_;

    //! Convert from geodetic to odometry
    GeodeticConverter::SharedPtr conv_;
    geometry_msgs::msg::PointStamped ltp_origin_msg_;

    //! Frame Names
    std::string ltp_frame_;
    std::string vehicle_frame_;

}; /* class LocalizationNode */

} /* namespace bvs_localization */

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bvs_localization::LocalizationExecutive>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}