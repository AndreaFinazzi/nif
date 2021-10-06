////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020 NovAtel Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>

#include <oem7_ros_publisher.hpp>


#include "nmea_msgs/msg/sentence.hpp"

#include <novatel_oem7_driver/oem7_message_util.hpp>

namespace novatel_oem7_driver
{
  class NMEAHandler: public Oem7MessageHandlerIf
  {
    std::unique_ptr<Oem7RosPublisher<nmea_msgs::msg::Sentence>> NMEA_pub_; ///< Publisher for NMEA sentences;


    void publishNMEASentence(Oem7RawMessageIf::ConstPtr msg)
    {
      std::shared_ptr<nmea_msgs::msg::Sentence> nmea_sentence(new nmea_msgs::msg::Sentence);
      nmea_sentence->sentence.assign(reinterpret_cast<const char*>(msg->getMessageData(0)), msg->getMessageDataLength());
      NMEA_pub_->publish(nmea_sentence);
    }

  public:
    void initialize(rclcpp::Node& node)
    {
      NMEA_pub_ = std::make_unique<Oem7RosPublisher<nmea_msgs::msg::Sentence>>("NMEA_Sentence", node);
    }

    const std::vector<int>& getMessageIds()
    {
      return OEM7_NMEA_MSGIDS;
    }

    void handleMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      publishNMEASentence(msg);
    }
  };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::NMEAHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
