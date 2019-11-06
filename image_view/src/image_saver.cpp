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
#include <camera_calibration_parsers/parse.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <boost/format.hpp>

#include <chrono>

namespace image_view
{

class ImageSaverNode
  : public rclcpp::Node
{
  boost::format g_format;
  bool save_all_image, save_image_service;
  std::string encoding;
  bool request_start_end;
  bool is_first_image_;
  bool has_camera_info_;
  size_t count_;
  rclcpp::Time start_time_;
  rclcpp::Time end_time_;
  image_transport::CameraSubscriber cam_sub_;
  image_transport::Subscriber image_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr end_srv_;

  bool saveImage(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg, std::string & filename)
  {
    cv::Mat image;
    try {
      image = cv_bridge::toCvShare(image_msg, encoding)->image;
    } catch(cv_bridge::Exception) {
      RCLCPP_ERROR(
        this->get_logger(), "Unable to convert %s image to %s",
        image_msg->encoding.c_str(), encoding.c_str());
      return false;
    }

    if (!image.empty()) {
      try {
        filename = (g_format).str();
      } catch (...) {
        g_format.clear();
      }

      try {
        filename = (g_format % count_).str();
      } catch (...) {
        g_format.clear();
      }

      try { 
        filename = (g_format % count_ % "jpg").str();
      } catch (...) {
        g_format.clear();
      }

      if (save_all_image || save_image_service ) {
        cv::imwrite(filename, image);
        RCLCPP_INFO(this->get_logger(), "Saved image %s", filename.c_str());

        save_image_service = false;
      } else {
        return false;
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Couldn't save image, no data!");
      return false;
    }

    return true;
  }

public:
  ImageSaverNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("image_saver_node", options),
      is_first_image_(true), has_camera_info_(false), count_(0)
  {
    auto topic = rclcpp::expand_topic_or_service_name("image", this->get_name(), this->get_namespace());

    // Useful when CameraInfo is being published
    cam_sub_ = image_transport::create_camera_subscription(
      this, topic, std::bind(
        &ImageSaverNode::callbackWithCameraInfo, this, std::placeholders::_1, std::placeholders::_2),
      "raw");

    // Useful when CameraInfo is not being published
    image_sub_ = image_transport::create_subscription(
      this, topic, std::bind(
        &ImageSaverNode::callbackWithoutCameraInfo, this, std::placeholders::_1),
      "raw");

    std::string format_string;
    format_string = this->declare_parameter("filename_format", std::string("left%04i.%s"));
    encoding = this->declare_parameter("encoding", std::string("bgr8"));
    save_all_image = this->declare_parameter("save_all_image", true);
    request_start_end = this->declare_parameter("request_start_end", false);
    g_format.parse(format_string);

    if (request_start_end && !save_all_image) {
      RCLCPP_WARN(
        this->get_logger(), "'request_start_end' is true, so overwriting 'save_all_image' as true");
    }

    save_srv_ = this->create_service<std_srvs::srv::Empty>(
      "save", std::bind(&ImageSaverNode::service, this, std::placeholders::_1, std::placeholders::_2));

    // FIXME(unkown): This does not make services appear
    // if (request_start_end) {
    start_srv_ = this->create_service<std_srvs::srv::Empty>(
        "start", std::bind(&ImageSaverNode::callbackStartSave, this, std::placeholders::_1, std::placeholders::_2));
    end_srv_ = this->create_service<std_srvs::srv::Empty>(
        "end", std::bind(&ImageSaverNode::callbackEndSave, this, std::placeholders::_1, std::placeholders::_2));
    // }
  }

  bool service(std_srvs::srv::Empty::Request &req, std_srvs::srv::Empty::Response &res)
  {
    (void)req;
    (void)res;
    save_image_service = true;
    return true;
  }

  bool callbackStartSave(
    std_srvs::srv::Trigger::Request &req,
    std_srvs::srv::Trigger::Response &res)
  {
    (void)req;
    RCLCPP_INFO(this->get_logger(), "Received start saving request");
    start_time_ = this->now();
    end_time_ = rclcpp::Time(0);

    res.success = true;
    return true;
  }

  bool callbackEndSave(
    std_srvs::srv::Trigger::Request &req,
    std_srvs::srv::Trigger::Response &res)
  {
    (void)req;
    RCLCPP_INFO(this->get_logger(), "Received end saving request");
    end_time_ = this->now();

    res.success = true;
    return true;
  }

  void callbackWithoutCameraInfo(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg)
  {
    if (is_first_image_) {
      is_first_image_ = false;

      // Wait a tiny bit to see whether callbackWithCameraInfo is called
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }

    if (has_camera_info_) {
      return;
    }

    // saving flag priority:
    //  1. request by service.
    //  2. request by topic about start and end.
    //  3. flag 'save_all_image'.
    if (!save_image_service && request_start_end) {
      if (start_time_ == rclcpp::Time(0)) {
        return;
      } else if (start_time_ > image_msg->header.stamp) {
        return;  // wait for message which comes after start_time
      } else if ((end_time_ != rclcpp::Time(0)) && (end_time_ < image_msg->header.stamp)) {
        return;  // skip message which comes after end_time
      }
    }

    // save the image
    std::string filename;
    if (!saveImage(image_msg, filename)) {
      return;
    }

    count_++;
  }

  void callbackWithCameraInfo(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
  {
    has_camera_info_ = true;

    if (!save_image_service && request_start_end) {
      if (start_time_ == rclcpp::Time(0)) {
        return;
      } else if (start_time_ > image_msg->header.stamp) {
        return;  // wait for message which comes after start_time
      } else if ((end_time_ != rclcpp::Time(0)) && (end_time_ < image_msg->header.stamp)) {
        return;  // skip message which comes after end_time
      }
    }

    // save the image
    std::string filename;
    if (!saveImage(image_msg, filename)) {
      return;
    }

    // save the CameraInfo
    if (info) {
      filename = filename.replace(filename.rfind("."), filename.length(), ".ini");
      camera_calibration_parsers::writeCalibration(filename, "camera", *info);
    }

    count_++;
  }
};

}  // namespace image_view

int main(int argc, char** argv)
{
  using image_view::ImageSaverNode;

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto is_node = std::make_shared<ImageSaverNode>(options);

  rclcpp::spin(is_node);
}
