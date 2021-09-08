/*
 * points_concat_filter.cpp
 *
 *  Created on: Aug 10, 2021
 *      Author: Daegyu Lee
 */

#include "nif_points_preprocessor_nodes/points_concat_node.h"
#include "nif_frame_id/frame_id.h"

using namespace nif::perception;
using namespace nif::common::frame_id::localization;

PointsConcatFilterNode::PointsConcatFilterNode(const std::string &node_name_)
    : Node(node_name_) {
  this->declare_parameter<std::string>(
      "input_topics",
      std::string("[/luminar_front_points, /luminar_left_points, "
                  "/luminar_right_points]"));
  this->declare_parameter<std::string>("output_topic",
                                       std::string("/merged/lidar"));
  this->declare_parameter<std::string>("output_frame_id",
                                       std::string(BASE_LINK));
  // setup QOS to be best effort
  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
  qos.best_effort();
  auto rmw_qos_profile = qos.get_rmw_qos_profile();

  respond();
  make_transform_list();

  std::cout << "input_topics : " << input_topics_ << std::endl;
  std::cout << "output_topic : " << output_topic_ << std::endl;
  std::cout << "output_frame_id : " << output_frame_id_ << std::endl;

  // input_topics_ = "[/front_lidar, /left_lidar, /right_lidar]";
  // std::cout << input_topics_ << std::endl;

  YAML::Node topics = YAML::Load(input_topics_);

  input_topics_size_ = topics.size();
  if (input_topics_size_ < 2 || 4 < input_topics_size_) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "The size of input_topics must be between 2 and 8");
    rclcpp::shutdown();
  }

  for (size_t i = 0; i < 4; ++i) {
    if (i < input_topics_size_) {
      cloud_subscribers_[i] = new message_filters::Subscriber<PointCloudMsgT>(
          this, topics[i].as<std::string>(), rmw_qos_profile);
    } else {
      cloud_subscribers_[i] = new message_filters::Subscriber<PointCloudMsgT>(
          this, topics[0].as<std::string>(), rmw_qos_profile);
    }
  }
  cloud_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(
      SyncPolicyT(10), *cloud_subscribers_[0], *cloud_subscribers_[1],
      *cloud_subscribers_[2]);
  cloud_synchronizer_->registerCallback(bind(
      &PointsConcatFilterNode::pointcloud_callback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));
  cloud_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
}

PointsConcatFilterNode::~PointsConcatFilterNode() {}

void PointsConcatFilterNode::respond() {
  this->get_parameter("input_topics", input_topics_);
  this->get_parameter("output_topic", output_topic_);
  this->get_parameter("output_frame_id", output_frame_id_);
}

void PointsConcatFilterNode::pointcloud_callback(
    const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
    const PointCloudMsgT::ConstPtr &msg3) {
  assert(2 <= input_topics_size_ && input_topics_size_ <= 4);

  PointCloudMsgT::ConstPtr msgs[3] = {msg1, msg2, msg3};
  // PointCloudT::Ptr cloud_sources[8];
  // PointCloudT::Ptr cloud_concatenated(new PointCloudT);

  pcl::PointCloud<LuminarPointXYZIRT>::Ptr source_points[3];
  pcl::PointCloud<LuminarPointXYZIRT>::Ptr concatenated_points(
      new pcl::PointCloud<LuminarPointXYZIRT>);

  for (size_t i = 0; i < input_topics_size_; ++i) {
    // Note: If you use kinetic, you can directly receive messages as
    std::string warning_msg;
    // cloud_sources[i] = PointCloudT().makeShared();
    source_points[i] = pcl::PointCloud<LuminarPointXYZIRT>().makeShared();
    pcl::fromROSMsg(*msgs[i], *source_points[i]);

    transformPointCloudCustom(*source_points[i], *source_points[i],
                              transfrom_list[i]);
  }

  // merge points
  for (size_t i = 0; i < input_topics_size_; ++i) {
    // *cloud_concatenated += *cloud_sources[i];
    *concatenated_points += *source_points[i];
  }
  // std::cout << "merged: " << concatenated_points->points.size() << std::endl;

  // publsh points
  sensor_msgs::msg::PointCloud2 MergedCloudMsg;
  // concatenated_points->header = pcl_conversions::toPCL(msgs[0]->header);
  concatenated_points->header.frame_id = output_frame_id_;
  pcl::toROSMsg(*concatenated_points, MergedCloudMsg);

  MergedCloudMsg.header.frame_id = output_frame_id_;
  cloud_publisher_->publish(MergedCloudMsg);
}

void PointsConcatFilterNode::make_transform_list() {
  tf2::Transform transform_to_center_of_gravity;
  transform_to_center_of_gravity.setOrigin(
      tf2::Vector3(-1.3206, 0.030188, -0.23598));
  tf2::Quaternion quat_to_center_of_gravity;
  quat_to_center_of_gravity.setRPY(0.0, 0.0, 0.0);
  transform_to_center_of_gravity.setRotation(quat_to_center_of_gravity);

  // Lidar Rotate
  tf2::Transform transform_lidar_rotate;
  transform_lidar_rotate.setOrigin(tf2::Vector3(0, 0, 0));
  tf2::Quaternion quat_lidar_rotate;
  // quat_lidar_rotate.setRPY(0.0, 0.0, -M_PI / 2); //origin data was rotated
  // but our sensory data was okay
  quat_lidar_rotate.setRPY(0.0, 0.0, 0.0);
  transform_lidar_rotate.setRotation(quat_lidar_rotate);

  //  FRONT LIDAR
  tf2::Transform transform_front;
  transform_front.setOrigin(tf2::Vector3(2.242, 0, 0.448));
  tf2::Quaternion quat_front;
  quat_front.setRPY(0.0, 0.0, 0.0);
  transform_front.setRotation(quat_front);
  transfrom_list.push_back(transform_to_center_of_gravity * transform_front *
                           transform_lidar_rotate);

  //  LEFT LIDAR
  tf2::Transform transform_left;
  transform_left.setOrigin(tf2::Vector3(1.549, 0.267, 0.543));
  tf2::Quaternion quat_left;
  quat_left.setRPY(0.0, 0.0, 2.0943951024);
  transform_left.setRotation(quat_left);
  transfrom_list.push_back(transform_to_center_of_gravity * transform_left *
                           transform_lidar_rotate);

  //  RIGHT LIDAR
  tf2::Transform transform_right;
  transform_right.setOrigin(tf2::Vector3(1.549, -0.267, 0.543));
  tf2::Quaternion quat_right;
  quat_right.setRPY(0.0, 0.0, -2.0943951024);
  transform_right.setRotation(quat_right);
  transfrom_list.push_back(transform_to_center_of_gravity * transform_right *
                           transform_lidar_rotate);
}

void PointsConcatFilterNode::transformPointCloudCustom(
    const pcl::PointCloud<LuminarPointXYZIRT> &cloud_in,
    pcl::PointCloud<LuminarPointXYZIRT> &cloud_out,
    const tf2::Transform &transform) {
  // Bullet (used by tf) and Eigen both store quaternions in x,y,z,w order,
  // despite the ordering of arguments in Eigen's constructor. We could use an
  // Eigen Map to convert without copy, but this only works if Bullet uses
  // floats, that is if BT_USE_DOUBLE_PRECISION is not defined. Rather that
  // risking a mistake, we copy the quaternion, which is a small cost compared
  // to the conversion of the point cloud anyway. Idem for the origin.
  tf2::Quaternion q = transform.getRotation();
  Eigen::Quaternionf rotation(q.w(), q.x(), q.y(),
                              q.z()); // internally stored as (x,y,z,w)
  tf2::Vector3 v = transform.getOrigin();
  Eigen::Vector3f origin(v.x(), v.y(), v.z());
  //    Eigen::Translation3f translation(v);
  // Assemble an Eigen Transform
  // Eigen::Transform3f t;
  // t = translation * rotation;
  transformPointCloud(cloud_in, cloud_out, origin, rotation);
}

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);

//   auto node = std::make_shared<PointsConcatFilterNode>();

//   rclcpp::spin(node);
//   rclcpp::shutdown();

//   return 0;
// }
