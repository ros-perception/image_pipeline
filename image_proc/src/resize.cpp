// Copyright 2017, 2019 Kentaro Wada, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
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

#include <functional>
#include <memory>
#include <string>

#include "cv_bridge/cv_bridge.hpp"
#include "tracetools_image_pipeline/tracetools.h"

#include <image_proc/resize.hpp>
#include <image_proc/utils.hpp>

#include <image_transport/image_transport.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace image_proc
{

ResizeNode::ResizeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ResizeNode", options)
{
  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("image_transport", "raw");

  // For compressed topics to remap appropriately, we need to pass a
  // fully expanded and remapped topic name to image_transport
  auto node_base = this->get_node_base_interface();
  image_topic_ = node_base->resolve_topic_or_service_name("image/image_raw", false);
  std::string pub_topic = node_base->resolve_topic_or_service_name("resize/image_raw", false);

  // Declare parameters before we setup any publishers or subscribers
  interpolation_ = this->declare_parameter("interpolation", 1);
  use_scale_ = this->declare_parameter("use_scale", true);
  scale_height_ = this->declare_parameter("scale_height", 1.0);
  scale_width_ = this->declare_parameter("scale_width", 1.0);
  height_ = this->declare_parameter("height", -1);
  width_ = this->declare_parameter("width", -1);

  // Setup lazy subscriber using publisher connection callback
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.matched_callback =
    [this](rclcpp::MatchedInfo &)
    {
      if (pub_image_.getNumSubscribers() == 0) {
        sub_image_.shutdown();
      } else if (!sub_image_) {
        // Create subscriber with QoS matched to subscribed topic publisher
        auto qos_profile = getTopicQosProfile(this, image_topic_);
        image_transport::TransportHints hints(this);
        sub_image_ = image_transport::create_camera_subscription(
          this, image_topic_,
          std::bind(
            &ResizeNode::imageCb, this,
            std::placeholders::_1,
            std::placeholders::_2), hints.getTransport(), qos_profile);
      }
    };

  // Create publisher - allow overriding QoS settings (history, depth, reliability)
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pub_image_ =
    image_transport::create_camera_publisher(this, pub_topic, rmw_qos_profile_default, pub_options);
}

void ResizeNode::imageCb(
  sensor_msgs::msg::Image::ConstSharedPtr image_msg,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
{
  TRACEPOINT(
    image_proc_resize_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)));

  cv_bridge::CvImageConstPtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvShare(image_msg);
  } catch (cv_bridge::Exception & e) {
    TRACEPOINT(
      image_proc_resize_fini,
      static_cast<const void *>(this),
      static_cast<const void *>(&(*image_msg)),
      static_cast<const void *>(&(*info_msg)));
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  if (use_scale_) {
    cv::resize(
      cv_ptr->image, scaled_cv_.image, cv::Size(0, 0), scale_width_,
      scale_height_, interpolation_);
  } else {
    int height = height_ == -1 ? image_msg->height : height_;
    int width = width_ == -1 ? image_msg->width : width_;
    cv::resize(cv_ptr->image, scaled_cv_.image, cv::Size(width, height), 0, 0, interpolation_);
  }

  auto dst_info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(*info_msg);

  double scale_y;
  double scale_x;

  if (use_scale_) {
    scale_y = scale_height_;
    scale_x = scale_width_;
    dst_info_msg->height = static_cast<int>(info_msg->height * scale_height_);
    dst_info_msg->width = static_cast<int>(info_msg->width * scale_width_);
  } else {
    scale_y = static_cast<double>(height_) / info_msg->height;
    scale_x = static_cast<double>(width_) / info_msg->width;
    dst_info_msg->height = height_;
    dst_info_msg->width = width_;
  }

  dst_info_msg->k[0] = dst_info_msg->k[0] * scale_x;  // fx
  dst_info_msg->k[2] = dst_info_msg->k[2] * scale_x;  // cx
  dst_info_msg->k[4] = dst_info_msg->k[4] * scale_y;  // fy
  dst_info_msg->k[5] = dst_info_msg->k[5] * scale_y;  // cy

  dst_info_msg->p[0] = dst_info_msg->p[0] * scale_x;  // fx
  dst_info_msg->p[2] = dst_info_msg->p[2] * scale_x;  // cx
  dst_info_msg->p[3] = dst_info_msg->p[3] * scale_x;  // T
  dst_info_msg->p[5] = dst_info_msg->p[5] * scale_y;  // fy
  dst_info_msg->p[6] = dst_info_msg->p[6] * scale_y;  // cy

  dst_info_msg->roi.x_offset = static_cast<int>(dst_info_msg->roi.x_offset * scale_x);
  dst_info_msg->roi.y_offset = static_cast<int>(dst_info_msg->roi.y_offset * scale_y);
  dst_info_msg->roi.width = static_cast<int>(dst_info_msg->roi.width * scale_x);
  dst_info_msg->roi.height = static_cast<int>(dst_info_msg->roi.height * scale_y);

  auto dst_image_msg = std::make_unique<sensor_msgs::msg::Image>();

  scaled_cv_.header = image_msg->header;
  scaled_cv_.encoding = image_msg->encoding;
  scaled_cv_.toImageMsg(*dst_image_msg);

  pub_image_.publish(std::move(dst_image_msg), std::move(dst_info_msg));

  TRACEPOINT(
    image_proc_resize_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)));
}

}  // namespace image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_proc::ResizeNode)
