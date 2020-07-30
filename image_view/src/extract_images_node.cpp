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

// Copyright 2019, Joshua Whitley
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <boost/format.hpp>

#include <mutex>
#include <string>

#include "image_view/extract_images_node.hpp"

namespace image_view
{

ExtractImagesNode::ExtractImagesNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("extract_images_node", options),
  filename_format_(""), count_(0), _time(this->now())
{
  auto topic = rclcpp::expand_topic_or_service_name(
    "image", this->get_name(), this->get_namespace());

  std::string transport = this->declare_parameter("transport", std::string("raw"));

  sub_ = image_transport::create_subscription(
    this, topic, std::bind(
      &ExtractImagesNode::image_cb, this, std::placeholders::_1),
    transport);

  auto topics = this->get_topic_names_and_types();

  if (topics.find(topic) != topics.end()) {
    RCLCPP_WARN(
      this->get_logger(), "extract_images: image has not been remapped! "
      "Typical command-line usage:\n\t$ ./extract_images image:=<image topic> [transport]");
  }

  std::string format_string =
    this->declare_parameter("filename_format", std::string("frame%04i.jpg"));
  filename_format_.parse(format_string);

  double sec_per_frame_ = this->declare_parameter("sec_per_frame", 0.1);

  RCLCPP_INFO(this->get_logger(), "Initialized sec per frame to %f", sec_per_frame_);
}

void ExtractImagesNode::image_cb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> guard(image_mutex_);

  // Hang on to message pointer for sake of mouse_cb
  last_msg_ = msg;

  // May want to view raw bayer data
  // NB: This is hacky, but should be OK since we have only one image CB.
  if (msg->encoding.find("bayer") != std::string::npos) {
    std::const_pointer_cast<sensor_msgs::msg::Image>(msg)->encoding = "mono8";
  }

  cv::Mat image;
  try {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;
  } catch (const cv_bridge::Exception &) {
    RCLCPP_ERROR(this->get_logger(), "Unable to convert %s image to bgr8", msg->encoding.c_str());
  }

  rclcpp::Duration delay = this->now() - _time;

  if (delay.seconds() >= sec_per_frame_) {
    _time = this->now();

    if (!image.empty()) {
      std::string filename = (filename_format_ % count_).str();

      cv::imwrite(filename, image);

      RCLCPP_INFO(this->get_logger(), "Saved image %s", filename.c_str());
      count_++;
    } else {
      RCLCPP_WARN(this->get_logger(), "Couldn't save image, no data!");
    }
  }
}

}  // namespace image_view

RCLCPP_COMPONENTS_REGISTER_NODE(image_view::ExtractImagesNode)
