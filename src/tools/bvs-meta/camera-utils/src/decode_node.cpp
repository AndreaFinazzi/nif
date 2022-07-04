#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <algorithm>
#include <map>

class DecodeNode : public rclcpp::Node
{
public:
  DecodeNode() : Node("DecodeNode")
  {
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

    std::map<std::string, std::string> topic_file_names;

    for (const auto &topic : topics)
    {
      std::string file_name = topic;                              // copy
      std::replace(file_name.begin(), file_name.end(), '/', '_'); // replace all '/' to '_'
      file_name.append(".avi");

      topic_file_names.insert({topic, file_name});
    }

    for (const auto &topic : topics)
    {
      RCLCPP_INFO(get_logger(), "Creating publishers / subscribers for topic: %s", topic.c_str());

      cv::namedWindow(topic, cv::WINDOW_KEEPRATIO);
      this->topic_vw_ptr.insert({topic,
                                 std::make_unique<cv::VideoWriter>(topic_file_names[topic], cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 43.75, cv::Size(772, 1032))});

      subscribers_.push_back(create_subscription<sensor_msgs::msg::CompressedImage>(topic, qos,
                                                                                    [this, topic](const sensor_msgs::msg::CompressedImage::SharedPtr img)
                                                                                    {
                                                                                      const auto decoded = cv::imdecode(img->data, cv::IMREAD_UNCHANGED);

                                                                                      std::ostringstream ss;
                                                                                      ss << "sec: " << img->header.stamp.sec << "   nanosec: " << img->header.stamp.nanosec;

                                                                                      // Insert stamp overlay
                                                                                      cv::putText(decoded,
                                                                                                  ss.str(),
                                                                                                  cv::Point(20, 20),              // Coordinates (Bottom-left corner of the text string in the image)
                                                                                                  cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                                                                                                  1.0,                            // Scale. 2.0 = 2x bigger
                                                                                                  cv::Scalar(33, 33, 255),        // BGR Color
                                                                                                  1,                              // Line Thickness (Optional)
                                                                                                  cv::LINE_AA);                   // Anti-alias (Optional, see version note)

                                                                                      this->topic_vw_ptr[topic]->write(decoded);
                                                                                      cv::imshow(topic, decoded);
                                                                                      cv::waitKey(1);
                                                                                    }));
    }
  }

  ~DecodeNode()
  {
    for (auto const &[topic, vw_ptr] : topic_vw_ptr)
    {
      vw_ptr->release();
    }
  }

private:
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> subscribers_;

  std::map<std::string, std::unique_ptr<cv::VideoWriter>> topic_vw_ptr;
};

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DecodeNode>());
  rclcpp::shutdown();

  return 0;
}
