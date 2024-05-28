// Copyright (c) 2014, JSK Lab.
// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cmath>
#include <chrono>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "cv_bridge/cv_bridge.h"

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_publisher/image_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace image_publisher
{

using namespace std::chrono_literals;

ImagePublisher::ImagePublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("ImagePublisher", options)
{
  pub_ = image_transport::create_camera_publisher(this, "image_raw");

  field_of_view_ = this->declare_parameter("field_of_view", static_cast<double>(0));
  flip_horizontal_ = this->declare_parameter("flip_horizontal", false);
  flip_vertical_ = this->declare_parameter("flip_vertical", false);
  frame_id_ = this->declare_parameter("frame_id", std::string("camera"));
  publish_rate_ = this->declare_parameter("publish_rate", static_cast<double>(10));
  camera_info_url_ = this->declare_parameter("camera_info_url", std::string(""));
  retry_ = this->declare_parameter("retry", false);
  timeout_ = this->declare_parameter("timeout", 2000);

  auto param_change_callback =
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      RCLCPP_INFO(get_logger(), "param_change_callback");

      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters) {
        if (parameter.get_name() == "filename") {
          filename_ = parameter.as_string();
          RCLCPP_INFO(get_logger(), "Reset filename as '%s'", filename_.c_str());
          ImagePublisher::onInit();
          return result;
        } else if (parameter.get_name() == "field_of_view") {
          field_of_view_ = parameter.as_double();
          RCLCPP_INFO(get_logger(), "Reset field_of_view as '%f'", field_of_view_);
          ImagePublisher::onInit();
          return result;
        } else if (parameter.get_name() == "flip_horizontal") {
          flip_horizontal_ = parameter.as_bool();
          RCLCPP_INFO(get_logger(), "Reset flip_horizontal as '%d'", flip_horizontal_);
          ImagePublisher::onInit();
          return result;
        } else if (parameter.get_name() == "flip_vertical") {
          flip_vertical_ = parameter.as_bool();
          RCLCPP_INFO(get_logger(), "Reset flip_vertical as '%d'", flip_vertical_);
          ImagePublisher::onInit();
          return result;
        } else if (parameter.get_name() == "frame_id") {
          frame_id_ = parameter.as_string();
          RCLCPP_INFO(get_logger(), "Reset frame_id as '%s'", frame_id_.c_str());
        } else if (parameter.get_name() == "publish_rate") {
          publish_rate_ = parameter.as_double();
          RCLCPP_INFO(get_logger(), "Reset publish_rate as '%lf'", publish_rate_);
        } else if (parameter.get_name() == "camera_info_url") {
          camera_info_url_ = parameter.as_string();
          RCLCPP_INFO(get_logger(), "Reset camera_info_rul as '%s'", camera_info_url_.c_str());
        }
      }
      ImagePublisher::reconfigureCallback();
      return result;
    };
  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(param_change_callback);
}

void ImagePublisher::reconfigureCallback()
{
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / publish_rate_)),
    std::bind(&ImagePublisher::doWork, this));

  camera_info_manager::CameraInfoManager c(this);
  if (!camera_info_url_.empty()) {
    RCLCPP_INFO(get_logger(), "camera_info_url exist");
    try {
      c.validateURL(camera_info_url_);
      c.loadCameraInfo(camera_info_url_);
      camera_info_ = c.getCameraInfo();
    } catch (cv::Exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "camera calibration failed to load: %s %s %s %i",
        e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
  } else {
    RCLCPP_INFO(get_logger(), "no camera_info_url exist");
  }
}

void ImagePublisher::doWork()
{
  // If the image is empty retry loading the image from the filename
  if (image_.empty() && retry_) {
    ImagePublisher::onInit();
  }
  // Transform the image.
  try {
    if (cap_.isOpened()) {
      if (!cap_.read(image_)) {
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
      }
      image_flipped_ = false;
    }
    if (flip_image_ && !image_flipped_) {
      cv::flip(image_, image_, flip_value_);
      image_flipped_ = true;
    }

    sensor_msgs::msg::Image::SharedPtr out_img =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
    out_img->header.frame_id = frame_id_;
    out_img->header.stamp = this->now();
    camera_info_.header.frame_id = out_img->header.frame_id;
    camera_info_.header.stamp = out_img->header.stamp;

    pub_.publish(*out_img, camera_info_);
  } catch (cv::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Image processing error: %s %s %s %i",
      e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
  }
}

void ImagePublisher::onInit()
{
  RCLCPP_INFO(this->get_logger(), "File name for publishing image is : %s", filename_.c_str());
  try {
    image_ = cv::imread(filename_, cv::IMREAD_COLOR);
    if (image_.empty()) {  // if filename not exist, open video device
      try {  // if filename is number
        int num = std::stoi(filename_);  // num is 1234798797
        cap_.open(num);
      } catch (const std::invalid_argument &) {  // if file name is string
        cap_.open(filename_);
      }
      CV_Assert(cap_.isOpened());
      cap_.read(image_);
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
    }
    CV_Assert(!image_.empty());
  } catch (cv::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to load image (%s): %s %s %s %i",
      filename_.c_str(), e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    if (retry_) {
      RCLCPP_INFO(get_logger(), "Retrying in %i millisecs", timeout_);
      std::this_thread::sleep_for(std::chrono::milliseconds(timeout_));
      ImagePublisher::onInit();
    }
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Flip horizontal image is : %s", ((flip_horizontal_) ? "true" : "false"));
  RCLCPP_INFO(
    this->get_logger(),
    "Flip flip_vertical image is : %s", ((flip_vertical_) ? "true" : "false"));

  // From http://docs.opencv.org/modules/core/doc/operations_on_arrays.html
  // #void flip(InputArray src, OutputArray dst, int flipCode)
  // FLIP_HORIZONTAL == 1, FLIP_VERTICAL == 0 or FLIP_BOTH == -1
  flip_image_ = true;
  if (flip_horizontal_ && flip_vertical_) {
    flip_value_ = 0;  // flip both, horizontal and vertical
  } else if (flip_horizontal_) {
    flip_value_ = 1;
  } else if (flip_vertical_) {
    flip_value_ = -1;
  } else {
    flip_image_ = false;
  }
  image_flipped_ = false;  // Image newly read, needs to be flipped

  camera_info_.width = image_.cols;
  camera_info_.height = image_.rows;
  camera_info_.distortion_model = "plumb_bob";
  camera_info_.d = {0, 0, 0, 0, 0};

  double f_approx = 1.0;  // FOV equal to 0 disables the approximation
  if (std::abs(field_of_view_) > std::numeric_limits<double>::epsilon()) {
    // Based on https://learnopencv.com/approximate-focal-length-for-webcams-and-cell-phone-cameras/
    f_approx = (camera_info_.width / 2) / std::tan((field_of_view_ * M_PI / 180) / 2);
  }
  camera_info_.k = {f_approx, 0, static_cast<float>(camera_info_.width / 2), 0, f_approx,
    static_cast<float>(camera_info_.height / 2), 0, 0, 1};
  camera_info_.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  camera_info_.p = {f_approx, 0, static_cast<float>(camera_info_.width / 2), 0, 0, f_approx,
    static_cast<float>(camera_info_.height / 2), 0, 0, 0, 1, 0};

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / publish_rate_)),
    std::bind(&ImagePublisher::doWork, this));
}

}  // namespace image_publisher

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_publisher::ImagePublisher)
