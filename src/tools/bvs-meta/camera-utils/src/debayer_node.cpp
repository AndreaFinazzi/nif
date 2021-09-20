#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <algorithm>

class DebayerNode : public rclcpp::Node {
public:
  DebayerNode() : Node("DebayerNode") {
    declare_parameter("topics", std::vector<std::string>{
      "/camera/front_left/image",
      "/camera/front_left_center/image",
      "/camera/front_right/image",
      "/camera/front_right_center/image",
      "/camera/rear_left/image",
      "/camera/rear_right/image",
    });

    const auto topics = get_parameter("topics").as_string_array();

    declare_parameter("bayer_type", "BG");
    const auto bayer_type = get_parameter("bayer_type").as_string();
  
    // debayered_publishers_.reserve(topics.size());
    compressed_publishers_.reserve(topics.size());

    for (const auto& topic : topics) {
      RCLCPP_INFO(get_logger(), "Creating publishers and subscribers for topic: %s", topic.c_str());

      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
      qos.best_effort();

      // auto& debayered_pub = debayered_publishers_.emplace_back(create_publisher<sensor_msgs::msg::Image>(topic + "/debayered", qos));
      auto& compressed_pub = compressed_publishers_.emplace_back(create_publisher<sensor_msgs::msg::CompressedImage>(topic + "/compressed", qos));

      subscribers_.push_back(create_subscription<sensor_msgs::msg::Image>(topic, qos, 
        // [this, topic, bayer_type, &debayered_pub, &compressed_pub](const sensor_msgs::msg::Image::SharedPtr img) {
        [this, topic, bayer_type, &compressed_pub](const sensor_msgs::msg::Image::SharedPtr img) {

          // RCLCPP_INFO(get_logger(), "input w x h: (%u, %u), encoding: %s, bigendian: %d, step: %u", 
          //   img->width, img->height, img->encoding.c_str(), img->is_bigendian, img->step);

          // const auto debayer_start = std::chrono::high_resolution_clock::now();

          // Wrap the input image data in a cv mat, without copying stuff.
          const cv::Mat m(img->height, img->width, CV_8UC1, static_cast<uint8_t*>(img->data.data()), img->step);

          // Make the output message
          sensor_msgs::msg::Image output_image;
          output_image.header = img->header;
          output_image.height = img->height;
          output_image.width = img->width;
          output_image.encoding = "bgr8";
          output_image.is_bigendian = false; // TODO: Make this not hardcoded
          output_image.step = img->step * 3; // TODO: Not hardcode?
          output_image.data = std::vector<uint8_t>(output_image.height * output_image.width * 3);

          // RCLCPP_INFO(get_logger(), "output w x h: (%u, %u), encoding: %s, bigendian: %d, step: %u", 
          //   output_image.width, output_image.height, output_image.encoding.c_str(), output_image.is_bigendian, output_image.step);

          cv::Mat output_mat(output_image.height, output_image.width, CV_8UC3,
            static_cast<uint8_t*>(output_image.data.data()), output_image.step);

          cv::ColorConversionCodes code;
          if (bayer_type == "BG") {
            code = cv::COLOR_BayerBG2BGR;
          } else if (bayer_type == "GB") {
            code = cv::COLOR_BayerGB2BGR;
          } else if (bayer_type == "RG") {
            code = cv::COLOR_BayerRG2BGR;
          } else if (bayer_type == "GR") {
            code = cv::COLOR_BayerGR2BGR;
          } else {
            RCLCPP_ERROR(get_logger(), "Received an invalid conversion code: %s, assuming RG", bayer_type.c_str());
            // DEFAULT
            code = cv::COLOR_BayerRG2BGR;
          }

          cv::demosaicing(m, output_mat, code);
          // debayered_pub->publish(output_image);

          // const auto debayer_end = std::chrono::high_resolution_clock::now();
          // const auto compress_start = debayer_end;

          sensor_msgs::msg::CompressedImage compressed_image;
          compressed_image.header = img->header;
          compressed_image.format = "jpeg";
          cv::imencode(".jpg", output_mat, compressed_image.data,
            std::vector<int>{
              cv::IMWRITE_JPEG_QUALITY, 90
              // TODO: Add more parameters here
            }
          );

          compressed_pub->publish(compressed_image);

          // const auto compress_end = std::chrono::high_resolution_clock::now();

          // RCLCPP_DEBUG(get_logger(), "%s image [%s](%u x %u) debayer took %lu microseconds, compress took %lu microseconds afterwards",
          //   topic.c_str(), img->encoding.c_str(), img->width, img->height,
          //   std::chrono::duration_cast<std::chrono::microseconds>(debayer_end - debayer_start).count(),
          //   std::chrono::duration_cast<std::chrono::microseconds>(compress_end - compress_start).count()
          // );
        }
      ));
    }


  }

private:
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscribers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> debayered_publishers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> compressed_publishers_;
};

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DebayerNode>());
  rclcpp::shutdown();

  return 0;
}