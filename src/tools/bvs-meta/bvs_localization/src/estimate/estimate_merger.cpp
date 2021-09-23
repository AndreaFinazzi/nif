#include "bvs_localization/estimate/estimate_merger.h"

namespace bvs_localization {
namespace estimate {

/****************************************************************************************/
/************************************ EstimateMerger ************************************/
/****************************************************************************************/
EstimateMerger::EstimateMerger() {
    cleanEstimate();
}

void
EstimateMerger::setPriorities(std::vector<std::string> source_priority) {
    priority_mapping_.clear();

    // Priority values should be ascending
    // (higher int = higher priority)
    int priority = source_priority.size() + 1;
    for(auto it = source_priority.begin(); it != source_priority.end(); ++it) {
        priority_mapping_[*it] = priority;
        --priority;
    }
}

void
EstimateMerger::cleanEstimate() {
    estimate_ = Estimate();
    merge_info_ = MergeInfo();
}


std::pair<Estimate, EstimateMerger::MergeInfo>
EstimateMerger::getEstimate() {
    std::pair<Estimate, MergeInfo> result;
    result.first = estimate_;
    result.second = merge_info_;
    return result;
}


void
EstimateMerger::addEstimate(std::string name, EstimateStatus status, Estimate estimate) {
    auto priority = priority_mapping_.find(name);
    // if source is not in priority mapping we can't add it
    if(priority == priority_mapping_.end()) {
        return;
    }
    FieldInfo source_info;
    source_info.source_name = name;
    source_info.source_priority = priority->second;
    source_info.status = status;
    addEstimate(source_info, estimate);
}

void
EstimateMerger::addEstimate(FieldInfo source_info, Estimate estimate) {
    if(estimate.valid_fields & Estimate::Fields::POSITION) {
        if(acceptField(source_info, merge_info_.position)) {
            estimate_.position = estimate.position;
            merge_info_.position = source_info;
        }
    }
    if(estimate.valid_fields & Estimate::Fields::ORIENTATION) {
        if(acceptField(source_info, merge_info_.orientation)) {
            estimate_.orientation = estimate.orientation;
            merge_info_.orientation = source_info;
        }
    }
    if(estimate.valid_fields & Estimate::Fields::VELOCITY_X) {
        if(acceptField(source_info, merge_info_.linear_x)) {
            estimate_.velocity.linear_x = estimate.velocity.linear_x;
            merge_info_.linear_x = source_info;
        }
    }
    if(estimate.valid_fields & Estimate::Fields::VELOCITY_Y) {
        if(acceptField(source_info, merge_info_.linear_y)) {
            estimate_.velocity.linear_y = estimate.velocity.linear_y;
            merge_info_.linear_y = source_info;
        }
    }
    if(estimate.valid_fields & Estimate::Fields::VELOCITY_Z) {
        if(acceptField(source_info, merge_info_.linear_z)) {
            estimate_.velocity.linear_z = estimate.velocity.linear_z;
            merge_info_.linear_z = source_info;
        }
    }
}

bool
EstimateMerger::acceptField(FieldInfo source_info, FieldInfo current) {
    //! If the data is good we take it!
    if(source_info.status < current.status) {
        return true;
    }
    //! If it is just as good (status == status), then we take the higher priority data
    if(source_info.status == current.status && source_info.source_priority > current.source_priority) {
        return true;
    }
    return false;
}


/****************************************************************************************/
/************************************ MergeInfoPublisher ********************************/
/****************************************************************************************/
void 
MergeInfoPublisher::FieldPublishers::generate(
    rclcpp::Node::SharedPtr node,
    std::string field_name
) {
    source_pub = node->create_publisher<std_msgs::msg::String>("bvs_localization/merged/" + field_name + "/source", 1);
    status_pub = node->create_publisher<std_msgs::msg::UInt8>("bvs_localization/merged/" + field_name + "/status", 1);
}

void
MergeInfoPublisher::FieldPublishers::publish(
    const EstimateMerger::FieldInfo& field_info
) {
    // Make sure the publishers are initialized
    if(source_pub != nullptr) {
        std_msgs::msg::String source_msg;
        std_msgs::msg::UInt8 status_msg;
        source_msg.data = field_info.source_name;
        status_msg.data = field_info.status;
        source_pub->publish(source_msg);
        status_pub->publish(status_msg);
    }
}

MergeInfoPublisher::MergeInfoPublisher()
{}

void
MergeInfoPublisher::initROSInterface(rclcpp::Node::SharedPtr node) {
    position_pubs_.generate(node, "position");
    orientation_pubs_.generate(node, "orientation");
    linear_x_pubs_.generate(node, "linear_x");
    linear_y_pubs_.generate(node, "linear_y");
    linear_z_pubs_.generate(node, "linear_z");
    min_pubs_.generate(node, "min");
    max_pubs_.generate(node, "max");
}

void 
MergeInfoPublisher::publish(
    const EstimateMerger::MergeInfo& merge_info
) {
    auto min = merge_info.position;
    auto max = merge_info.position;
    min.source_name = "position";
    max.source_name = "position";
    position_pubs_.publish(merge_info.position);
    trackMaxMinField(merge_info.orientation, "orientation", min, max);
    orientation_pubs_.publish(merge_info.orientation);
    trackMaxMinField(merge_info.linear_x, "linear_x", min, max);
    linear_x_pubs_.publish(merge_info.linear_x);
    trackMaxMinField(merge_info.linear_y, "linear_y", min, max);
    linear_y_pubs_.publish(merge_info.linear_y);
    trackMaxMinField(merge_info.linear_z, "linear_z", min, max);
    linear_z_pubs_.publish(merge_info.linear_z);
    min_pubs_.publish(min);
    max_pubs_.publish(max);
}

void
MergeInfoPublisher::trackMaxMinField(
    const EstimateMerger::FieldInfo& new_field,
    std::string field_name,
    EstimateMerger::FieldInfo& min,
    EstimateMerger::FieldInfo& max
) {
    if(new_field.status == min.status) {
        min.source_name += "," + field_name;
    } else if(new_field.status < min.status) {
        min = new_field;
        min.source_name = field_name;
    }
    if(new_field.status == max.status) {
        max.source_name += "," + field_name;
    } else if(new_field.status > max.status) {
        max = new_field; 
        max.source_name = field_name;
    }
}


} /* namespace estimate */
} /* namespace bvs_localization */