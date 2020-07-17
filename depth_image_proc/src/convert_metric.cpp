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
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <depth_image_proc/visibility.h>
#include <cmath>
#include <memory>
#include <limits>

namespace depth_image_proc
{

namespace enc = sensor_msgs::image_encodings;

class ConvertMetricNode : public rclcpp::Node
{
public:
  DEPTH_IMAGE_PROC_PUBLIC ConvertMetricNode(const rclcpp::NodeOptions & options);

private:
  // Subscriptions
  image_transport::Subscriber sub_raw_;

  // Publications
  std::mutex connect_mutex_;
  image_transport::Publisher pub_depth_;

  void connectCb();

  void depthCb(const sensor_msgs::msg::Image::ConstSharedPtr & raw_msg);

  rclcpp::Logger logger_ = rclcpp::get_logger("ConvertMetricNode");
};

ConvertMetricNode::ConvertMetricNode(const rclcpp::NodeOptions & options)
: Node("ConvertMetricNode", options)
{
  // Monitor whether anyone is subscribed to the output
  // TODO(ros2) Implement when SubscriberStatusCallback is available
  // image_transport::SubscriberStatusCallback connect_cb =
  //     std::bind(&ConvertMetricNode::connectCb, this);
  connectCb();

  // Make sure we don't enter connectCb() between advertising and assigning to pub_depth_
  std::lock_guard<std::mutex> lock(connect_mutex_);
  // TODO(ros2) Implement when SubscriberStatusCallback is available
  // pub_depth_ = it_->advertise("image", 1, connect_cb, connect_cb);
  pub_depth_ = image_transport::create_publisher(this, "image");
}

// Handles (un)subscribing when clients (un)subscribe
void ConvertMetricNode::connectCb()
{
  std::lock_guard<std::mutex> lock(connect_mutex_);
  // TODO(ros2) Implement getNumSubscribers when rcl/rmw support it
  // if (pub_depth_.getNumSubscribers() == 0)
  if (0) {
    sub_raw_.shutdown();
  } else if (!sub_raw_) {
    image_transport::TransportHints hints(this, "raw");
    sub_raw_ = image_transport::create_subscription(
      this, "image_raw",
      std::bind(&ConvertMetricNode::depthCb, this, std::placeholders::_1),
      hints.getTransport());
  }
}

void ConvertMetricNode::depthCb(const sensor_msgs::msg::Image::ConstSharedPtr & raw_msg)
{
  auto depth_msg = std::make_shared<sensor_msgs::msg::Image>();
  depth_msg->header = raw_msg->header;
  depth_msg->height = raw_msg->height;
  depth_msg->width = raw_msg->width;

  // Set data, encoding and step after converting the metric.
  if (raw_msg->encoding == enc::TYPE_16UC1) {
    depth_msg->encoding = enc::TYPE_32FC1;
    depth_msg->step = raw_msg->width * (enc::bitDepth(depth_msg->encoding) / 8);
    depth_msg->data.resize(depth_msg->height * depth_msg->step);
    // Fill in the depth image data, converting mm to m
    float bad_point = std::numeric_limits<float>::quiet_NaN();
    const uint16_t * raw_data = reinterpret_cast<const uint16_t *>(&raw_msg->data[0]);
    float * depth_data = reinterpret_cast<float *>(&depth_msg->data[0]);
    for (unsigned index = 0; index < depth_msg->height * depth_msg->width; ++index) {
      uint16_t raw = raw_data[index];
      depth_data[index] = (raw == 0) ? bad_point : static_cast<float>(raw * 0.001f);
    }
  } else if (raw_msg->encoding == enc::TYPE_32FC1) {
    depth_msg->encoding = enc::TYPE_16UC1;
    depth_msg->step = raw_msg->width * (enc::bitDepth(depth_msg->encoding) / 8);
    depth_msg->data.resize(depth_msg->height * depth_msg->step);
    // Fill in the depth image data, converting m to mm
    uint16_t bad_point = 0;
    const float * raw_data = reinterpret_cast<const float *>(&raw_msg->data[0]);
    uint16_t * depth_data = reinterpret_cast<uint16_t *>(&depth_msg->data[0]);
    for (unsigned index = 0; index < depth_msg->height * depth_msg->width; ++index) {
      float raw = raw_data[index];
      depth_data[index] = std::isnan(raw) ? bad_point : static_cast<uint16_t>(raw * 1000);
    }
  } else {
    RCLCPP_ERROR(logger_, "Unsupported image conversion from %s.", raw_msg->encoding.c_str());
    return;
  }

  pub_depth_.publish(depth_msg);
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::ConvertMetricNode)
