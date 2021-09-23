#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <algorithm>

class DecodeNode : public rclcpp::Node {
public:
  DecodeNode() : Node("DecodeNode") {
    declare_parameter("topics", std::vector<std::string>{
      "/camera/front_left/image/compressed",
      "/camera/front_left_center/image/compressed",
      "/camera/front_right/image/compressed",
      "/camera/front_right_center/image/compressed",
      "/camera/rear_left/image/compressed",
      "/camera/rear_right/image/compressed",
    });
    const auto topics = get_parameter("topics").as_string_array();

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.best_effort();

    for (const auto& topic : topics) {
      RCLCPP_INFO(get_logger(), "Creating publishers / subscribers for topic: %s", topic.c_str());

      subscribers_.push_back(create_subscription<sensor_msgs::msg::CompressedImage>(topic, qos,
        [this, topic](const sensor_msgs::msg::CompressedImage::SharedPtr img) {
          const auto decoded = cv::imdecode(img->data, cv::IMREAD_UNCHANGED);
          cv::imshow(topic, decoded);
          cv::waitKey(1);
        }
      ));
    }
  }

private:
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> subscribers_;
};

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DecodeNode>());
  rclcpp::shutdown();

  return 0;
}
