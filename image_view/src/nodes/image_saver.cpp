/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>

#include <memory>
#include <string>
#include <vector>
#include <stdio.h>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rcutils/cmdline_parser.h>

bool save_all_image, save_image_service;
std::string encoding;
bool request_start_end;
rclcpp::Node::SharedPtr node;
std::string format_string;
char buf[1024];

bool service(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {
  save_image_service = true;
  return true;
}

class Callbacks {
public:
  Callbacks() : is_first_image_(true), has_camera_info_(false), count_(0) {
  }

  bool callbackStartSave(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                         const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    RCLCPP_INFO(node->get_logger(), "Received start saving request");
    start_time_ = rclcpp::Clock().now();
    end_time_ = rclcpp::Time(0);

    res->success = true;
    return true;
  }

  bool callbackEndSave(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                       const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    RCLCPP_INFO(node->get_logger(), "Received end saving request");
    end_time_ = rclcpp::Clock().now();

    res->success = true;
    return true;
  }

  void callbackWithoutCameraInfo(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg)
  {
    if (is_first_image_) {
      is_first_image_ = false;

      // Wait a tiny bit to see whether callbackWithCameraInfo is called
      rclcpp::sleep_for(std::chrono::nanoseconds(1));
    }

    if (has_camera_info_)
      return;

    // saving flag priority:
    //  1. request by service.
    //  2. request by topic about start and end.
    //  3. flag 'save_all_image'.
    if (!save_image_service && request_start_end) {
      if (start_time_ == rclcpp::Time(0))
        return;
      else if (start_time_ > image_msg->header.stamp)
        return;  // wait for message which comes after start_time
      else if ((end_time_ != rclcpp::Time(0)) && (end_time_ < image_msg->header.stamp))
        return;  // skip message which comes after end_time
    }

    // save the image
    std::string filename;
    if (!saveImage(image_msg, filename))
      return;

    count_++;
  }

  void callbackWithCameraInfo(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                              const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
  {
    has_camera_info_ = true;

    if (!save_image_service && request_start_end) {
      if (start_time_ == rclcpp::Time(0))
        return;
      else if (start_time_ > image_msg->header.stamp)
        return;  // wait for message which comes after start_time
      else if ((end_time_ != rclcpp::Time(0)) && (end_time_ < image_msg->header.stamp))
        return;  // skip message which comes after end_time
    }

    // save the image
    std::string filename;
    if (!saveImage(image_msg, filename))
      return;

    // save the CameraInfo
    if (info) {
      filename = filename.replace(filename.rfind("."), filename.length(), ".ini");
      camera_calibration_parsers::writeCalibration(filename, "camera", *info);
    }

    count_++;
  }
private:
  bool saveImage(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                 std::string & filename) 
  {
    cv::Mat image;
    try
    {
      image = cv_bridge::toCvShare(image_msg, encoding)->image;
    } 
    catch(cv_bridge::Exception)
    {
      RCLCPP_ERROR(node->get_logger(), "Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
      return false;
    }
    if (!image.empty()) {
        filename = format_string + std::to_string(count_) + std::string(".jpg"); 
      if ( save_all_image || save_image_service ) {
        cv::imwrite(filename, image);
        RCLCPP_INFO(node->get_logger(), "Saved image %s", filename.c_str());
        save_image_service = false;
      } else {
        return false;
      }
    } else {
      RCLCPP_WARN(node->get_logger(), "Couldn't save image, no data!");
      return false;
    }
    return true;
  }

private:
  bool is_first_image_;
  bool has_camera_info_;
  size_t count_;
  rclcpp::Time start_time_;
  rclcpp::Time end_time_;
};

int main(int argc, char** argv)
{
  std::string topic, encoding, format_string;
  bool request_start_end = false;
  save_all_image;
  rclcpp::init(argc, argv);

  node = rclcpp::Node::make_shared("image_saver");

  Callbacks callbacks;
  // Useful when CameraInfo is being published
  node->get_parameter_or_set("image", topic, std::string("image"));
  node->get_parameter_or_set("format_string", format_string, std::string("frame"));
  node->get_parameter_or_set("encoding", encoding, std::string("bgr8"));
  node->get_parameter_or_set("save_all_image", save_all_image, false);
  image_transport::CameraSubscriber sub_image_and_camera = 
    image_transport::create_camera_subscription(node.get(), topic,
        std::bind(&Callbacks::callbackWithCameraInfo, 
          callbacks, std::placeholders::_1, std::placeholders::_2),
        "raw");  // Useful when CameraInfo is not being published
  image_transport::Subscriber sub_image = 
    image_transport::create_subscription(
      node.get(), topic, std::bind(&Callbacks::callbackWithoutCameraInfo, 
        &callbacks, std::placeholders:: _1), "raw");

  auto save = node->create_service<std_srvs::srv::Empty>("save", &service);
  if (request_start_end && !save_all_image)
    RCLCPP_WARN(node->get_logger(), "'request_start_end' is true, so overwriting 'save_all_image' as true");
    RCLCPP_INFO(node->get_logger(), "Save image from topic: %s", topic.c_str());
  // FIXME(unkown): This does not make services appear
  // if (request_start_end) {
    auto srv_start = node->create_service<std_srvs::srv::Trigger>(
      "start", std::bind(&Callbacks::callbackStartSave, &callbacks, std::placeholders::_1, std::placeholders::_2));
    auto srv_end = node->create_service<std_srvs::srv::Trigger>(
      "end", std::bind(&Callbacks::callbackStartSave, &callbacks, std::placeholders::_1, std::placeholders::_2));
  // }

  rclcpp::spin(node);
  rclcpp::shutdown();
}
