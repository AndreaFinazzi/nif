#ifndef BVS_LOCALIZATION_ESTIMATE_ESTIMATE_MERGER_H_
#define BVS_LOCALIZATION_ESTIMATE_ESTIMATE_MERGER_H_

// Application
#include "bvs_localization/estimate/estimate.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"

// STD
#include <vector>
#include <map>

namespace bvs_localization {
namespace estimate {

class EstimateMerger {
public:
    //! Tracks the source info for a single field
    struct FieldInfo {
        std::string source_name = "";
        unsigned int source_priority = 0;
        EstimateStatus status = EstimateStatus::UNKNOWN;
    };
    //! Tracks the source info for all fields
    struct MergeInfo {
        FieldInfo position;
        FieldInfo orientation;
        FieldInfo linear_x;
        FieldInfo linear_y;
        FieldInfo linear_z;
    };

    /**
     * @brief creates an estimate mergers
     **/
    EstimateMerger();

    /**
     * @brief Allows the setting of the priority listing (earlier = higher priority)
     **/
    void setPriorities(std::vector<std::string> source_priority);

    /**
     * @brief clears the current estimate
     **/
    void cleanEstimate();

    /**
     * @brief adds an estimate 
     **/
    void addEstimate(std::string name, EstimateStatus status, Estimate estimate);

    /** 
     * @brief gets the merged estimate as well as source info
     **/
    std::pair<Estimate, MergeInfo> getEstimate();

private:
    /**
     * @brief logic to add an estimate to the merged estimate
     * @param source the augmented source info containing all
     *  info required to make an update decision
     * @param estimate the odometry estimate to merge in
     **/
    void addEstimate(FieldInfo source_info, Estimate estimate);

    /**
     * @brief determines whether or not to update a particular field
     *  based on the info in FieldInfo
     **/
    bool acceptField(FieldInfo source_info, FieldInfo current);

    //! Quick mapping of source name to priority
    std::map<std::string, int> priority_mapping_;
    //! The merged estimate
    Estimate estimate_;
    //! Information on the estimate
    MergeInfo merge_info_;

}; /* class EstimateMerger */


class MergeInfoPublisher {
public:
    // The Required publishers for a single field
    struct FieldPublishers {
        // The source of the field
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_pub;
        // The status of the field
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_pub;

        /**
         * @brief generates publishers
         * @param node the node to generate publishers on
         * @param field_name name of the field to generate publishers for
         **/
        void generate(rclcpp::Node::SharedPtr node, std::string field_name);

        /**
         * @brief publishes info for a given field
         * @param field_info the information to publish
         **/
        void publish(const EstimateMerger::FieldInfo& field_info);
    };

    /**
     * @brief initializes a merge info publisher with no 
     **/
    MergeInfoPublisher();

    /**
     * @brief initializes publishers and subscribers
     * @param node the node to generate publishers on
     **/
    void initROSInterface(rclcpp::Node::SharedPtr node);

    /**
     * @brief publishes merge info across a few different topics
     * @param merge_info the info to publish
     **/
    void publish(const EstimateMerger::MergeInfo& merge_info);

private:
    /**
     * @brief tracks the min amd max fields
     * @param[in] new_field the field to compare to the current min / max fields
     * @param field_name the name of new_field
     * @param[in,out] min the current min to compare against and override if applicable
     * @param[in,out] max the current max to compare against and override if applicable
     **/
    void trackMaxMinField(
        const EstimateMerger::FieldInfo& new_field,
        std::string field_name,
        EstimateMerger::FieldInfo& min,
        EstimateMerger::FieldInfo& max
    );

    // Field Publishers for the individual fields
    FieldPublishers position_pubs_;
    FieldPublishers orientation_pubs_;
    FieldPublishers linear_x_pubs_;
    FieldPublishers linear_y_pubs_;
    FieldPublishers linear_z_pubs_;
    // Publisher for min / max status for a given MergeInfo
    FieldPublishers min_pubs_;
    FieldPublishers max_pubs_;

}; /* class MergeInfoPublisher */


} /* namespace estimate */
} /* namespace bvs_localization */

#endif /* BVS_LOCALIZATION_ESTIMATE_ESTIMATE_MERGER_H_ */