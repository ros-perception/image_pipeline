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

#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "depth_image_proc/visibility.h"
#include "message_filters/subscriber.hpp"
#include "message_filters/time_synchronizer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <depth_image_proc/depth_traits.hpp>

namespace depth_image_proc
{

class DisparityNode : public rclcpp::Node
{
public:
  DEPTH_IMAGE_PROC_PUBLIC DisparityNode(const rclcpp::NodeOptions & options);

private:
  image_transport::SubscriberFilter sub_depth_image_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_info_;
  using Sync = message_filters::TimeSynchronizer<sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo>;
  std::shared_ptr<Sync> sync_;

  std::mutex connect_mutex_;
  using DisparityImage = stereo_msgs::msg::DisparityImage;
  rclcpp::Publisher<DisparityImage>::SharedPtr pub_disparity_;
  double min_range_;
  double max_range_;
  double delta_d_;

  void depthCb(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

  template<typename T>
  void convert(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    stereo_msgs::msg::DisparityImage::SharedPtr & disp_msg);
};

DisparityNode::DisparityNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("DisparityNode", options)
{
  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("image_transport", "raw");

  // Read parameters
  int queue_size = this->declare_parameter<int>("queue_size", 5);
  min_range_ = this->declare_parameter<double>("min_range", 0.0);
  max_range_ = this->declare_parameter<double>(
    "max_range",
    std::numeric_limits<double>::infinity());
  delta_d_ = this->declare_parameter<double>("delta_d", 0.125);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  sync_ = std::make_shared<Sync>(sub_depth_image_, sub_info_, queue_size);
  sync_->registerCallback(
    std::bind(
      &DisparityNode::depthCb, this, std::placeholders::_1, std::placeholders::_2));

  // Create publisher with connect callback
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.matched_callback =
    [this](rclcpp::MatchedInfo &)
    {
      std::lock_guard<std::mutex> lock(connect_mutex_);
      if (pub_disparity_->get_subscription_count() == 0) {
        sub_depth_image_.unsubscribe();
        sub_info_.unsubscribe();
      } else if (!sub_depth_image_.getSubscriber()) {
        // For compressed topics to remap appropriately, we need to pass a
        // fully expanded and remapped topic name to image_transport
        auto node_base = this->get_node_base_interface();
        std::string topic = node_base->resolve_topic_or_service_name("left/image_rect", false);
        image_transport::TransportHints hints(this);
        sub_depth_image_.subscribe(this, topic, hints.getTransport());
        sub_info_.subscribe(this, "right/camera_info", rclcpp::QoS(10));
      }
    };
  pub_disparity_ = create_publisher<stereo_msgs::msg::DisparityImage>(
    "left/disparity", rclcpp::SensorDataQoS(), pub_options);
}

void DisparityNode::depthCb(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  auto disp_msg = std::make_shared<DisparityImage>();
  disp_msg->header = depth_msg->header;
  disp_msg->image.header = disp_msg->header;
  disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  disp_msg->image.height = depth_msg->height;
  disp_msg->image.width = depth_msg->width;
  disp_msg->image.step = disp_msg->image.width * sizeof(float);
  disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step, 0.0f);
  double fx = info_msg->p[0];
  disp_msg->t = -info_msg->p[3] / fx;
  disp_msg->f = fx;
  // Remaining fields depend on device characteristics, so rely on user input
  disp_msg->min_disparity = disp_msg->f * disp_msg->t / max_range_;
  disp_msg->max_disparity = disp_msg->f * disp_msg->t / min_range_;
  disp_msg->delta_d = delta_d_;

  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    convert<uint16_t>(depth_msg, disp_msg);
  } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    convert<float>(depth_msg, disp_msg);
  } else {
    RCLCPP_ERROR(
      get_logger(), "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  pub_disparity_->publish(*disp_msg);
}

template<typename T>
void DisparityNode::convert(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  stereo_msgs::msg::DisparityImage::SharedPtr & disp_msg)
{
  // For each depth Z, disparity d = fT / Z
  float unit_scaling = DepthTraits<T>::toMeters(T(1) );
  float constant = disp_msg->f * disp_msg->t / unit_scaling;

  const T * depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  float * disp_data = reinterpret_cast<float *>(&disp_msg->image.data[0]);
  for (int v = 0; v < static_cast<int>(depth_msg->height); ++v) {
    for (int u = 0; u < static_cast<int>(depth_msg->width); ++u) {
      T depth = depth_row[u];
      if (DepthTraits<T>::valid(depth)) {
        *disp_data = constant / depth;
      }
      ++disp_data;
    }

    depth_row += row_step;
  }
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::DisparityNode)
