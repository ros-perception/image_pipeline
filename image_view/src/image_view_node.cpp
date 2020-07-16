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

#include "image_view/image_view_node.hpp"

#include <boost/format.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace image_view
{

void ThreadSafeImage::set(cv_bridge::CvImageConstPtr image)
{
  std::lock_guard<std::mutex> lock(mutex_);
  image_ = image;
  condition_.notify_one();
}

cv_bridge::CvImageConstPtr ThreadSafeImage::get()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return image_;
}

cv_bridge::CvImageConstPtr ThreadSafeImage::pop()
{
  cv_bridge::CvImageConstPtr image;

  {
    std::unique_lock<std::mutex> lock(mutex_);

    condition_.wait_for(
      lock, std::chrono::milliseconds(100),
      [this] {
        return !image_;
      });

    image = std::move(image_);
  }

  return image;
}

ImageViewNode::ImageViewNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_view_node", options)
{
  auto transport = this->declare_parameter("image_transport", "raw");
  RCLCPP_INFO(this->get_logger(), "Using transport \"%s\"", transport.c_str());

  std::string topic = rclcpp::expand_topic_or_service_name(
    "image", this->get_name(), this->get_namespace());

  image_transport::TransportHints hints(this, transport);
  pub_ = this->create_publisher<sensor_msgs::msg::Image>("output", 1);
  sub_ = image_transport::create_subscription(
    this, topic, std::bind(
      &ImageViewNode::imageCb, this, std::placeholders::_1), hints.getTransport());

  auto topics = this->get_topic_names_and_types();

  if (topics.find(topic) != topics.end()) {
    RCLCPP_WARN(
      this->get_logger(), "Topic 'image' has not been remapped! "
      "Typical command-line usage:\n"
      "\t$ rosrun image_view image_view image:=<image topic> [transport]");
  }

  // Default window name is the resolved topic name
  window_name_ = this->declare_parameter("window_name", topic);
  g_gui = this->declare_parameter("gui", true);  // gui/no_gui mode
  autosize_ = this->declare_parameter("autosize", false);
  window_height_ = this->declare_parameter("height", -1);
  window_width_ = this->declare_parameter("width", -1);
  std::string format_string =
    this->declare_parameter("filename_format", std::string("frame%04i.jpg"));
  filename_format_.parse(format_string);

  colormap_ = this->declare_parameter("colormap", -1);
  min_image_value_ = this->declare_parameter("min_image_value", 0);
  max_image_value_ = this->declare_parameter("max_image_value", 0);

  if (g_gui) {
    window_thread_ = std::thread(&ImageViewNode::windowThread, this);
  }

  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ImageViewNode::paramCallback, this, std::placeholders::_1));
}

ImageViewNode::~ImageViewNode()
{
  if (window_thread_.joinable()) {
    window_thread_.join();
  }
}

void ImageViewNode::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // We want to scale floating point images so that they display nicely
  bool do_dynamic_scaling = (msg->encoding.find("F") != std::string::npos);

  // Convert to OpenCV native BGR color
  try {
    cv_bridge::CvtColorForDisplayOptions options;
    options.do_dynamic_scaling = do_dynamic_scaling;

    {
      std::lock_guard<std::mutex> lock(param_mutex_);
      options.colormap = colormap_;

      // Set min/max value for scaling to visualize depth/float image.
      if (min_image_value_ == max_image_value_) {
        // Not specified by rosparam, then set default value.
        // Because of current sensor limitation, we use 10m as default of max range of depth
        // with consistency to the configuration in rqt_image_view.
        options.min_image_value = 0;

        if (msg->encoding == "32FC1") {
          options.max_image_value = 10;  // 10 [m]
        } else if (msg->encoding == "16UC1") {
          options.max_image_value = 10 * 1000;  // 10 * 1000 [mm]
        }
      } else {
        options.min_image_value = min_image_value_;
        options.max_image_value = max_image_value_;
      }
    }

    queued_image_.set(
      cv_bridge::cvtColorForDisplay(
        cv_bridge::toCvShare(msg), "bgr8", options));
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR_EXPRESSION(
      this->get_logger(), (static_cast<int>(this->now().seconds()) % 30 == 0),
      "Unable to convert '%s' image for display: '%s'",
      msg->encoding.c_str(), e.what());
  }

  if (pub_->get_subscription_count() > 0) {
    pub_->publish(*msg);
  }
}

void ImageViewNode::mouseCb(int event, int /* x */, int /* y */, int /* flags */, void * param)
{
  ImageViewNode * this_ = reinterpret_cast<ImageViewNode *>(param);

  if (event == cv::EVENT_LBUTTONDOWN) {
    RCLCPP_WARN_ONCE(
      this_->get_logger(),
      "Left-clicking no longer saves images. Right-click instead.");
    return;
  }

  if (event != cv::EVENT_RBUTTONDOWN) {
    return;
  }

  cv_bridge::CvImageConstPtr image(this_->shown_image_.get());

  if (!image) {
    RCLCPP_WARN(this_->get_logger(), "Couldn't save image, no data!");
    return;
  }

  std::string filename = (this_->filename_format_ % this_->count_).str();

  if (cv::imwrite(filename, image->image)) {
    RCLCPP_INFO(this_->get_logger(), "Saved image %s", filename.c_str());
    this_->count_++;
  } else {
    /// @todo Show full path, ask if user has permission to write there
    RCLCPP_ERROR(this_->get_logger(), "Failed to save image.");
  }
}

void ImageViewNode::windowThread()
{
  int flags = autosize_ ?
    (cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED) : 0;
  cv::namedWindow(window_name_, flags);
  cv::setMouseCallback(window_name_, &ImageViewNode::mouseCb, this);

  if (!autosize_ && window_width_ > -1 && window_height_ > -1) {
    cv::resizeWindow(window_name_, window_width_, window_height_);
  }

  while (rclcpp::ok()) {
    cv_bridge::CvImageConstPtr image(queued_image_.pop());

    if (cv::getWindowProperty(window_name_, 1) < 0) {
      break;
    }

    if (image) {
      cv::imshow(window_name_, image->image);
      shown_image_.set(image);
      cv::waitKey(1);
    }
  }

  cv::destroyWindow(window_name_);

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

rcl_interfaces::msg::SetParametersResult
ImageViewNode::paramCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "colormap") {
      std::lock_guard<std::mutex> lock(param_mutex_);
      colormap_ = parameter.as_int();
      break;
    } else if (parameter.get_name() == "min_image_value") {
      std::lock_guard<std::mutex> lock(param_mutex_);
      min_image_value_ = parameter.as_int();
      break;
    } else if (parameter.get_name() == "max_image_value") {
      std::lock_guard<std::mutex> lock(param_mutex_);
      max_image_value_ = parameter.as_int();
      break;
    }
  }

  return result;
}

}  // namespace image_view

RCLCPP_COMPONENTS_REGISTER_NODE(image_view::ImageViewNode)
