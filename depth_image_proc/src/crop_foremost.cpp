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
#include <mutex>
#include <string>

#include "cv_bridge/cv_bridge.hpp"
#include "depth_image_proc/visibility.h"

#include <rclcpp/rclcpp.hpp>
#include <image_proc/utils.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace depth_image_proc
{

namespace enc = sensor_msgs::image_encodings;

class CropForemostNode : public rclcpp::Node
{
public:
  DEPTH_IMAGE_PROC_PUBLIC CropForemostNode(const rclcpp::NodeOptions & options);

private:
  // Subscriptions
  image_transport::Subscriber sub_raw_;

  // Publications
  std::mutex connect_mutex_;
  image_transport::Publisher pub_depth_;

  void depthCb(const sensor_msgs::msg::Image::ConstSharedPtr & raw_msg);

  double distance_;

  rclcpp::Logger logger_ = rclcpp::get_logger("CropForemostNode");
};

CropForemostNode::CropForemostNode(const rclcpp::NodeOptions & options)
: Node("CropForemostNode", options)
{
  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("image_transport", "raw");

  distance_ = this->declare_parameter("distance", 0.0);


  // Setup lazy subscriber using publisher connection callback
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.matched_callback =
    [this](rclcpp::MatchedInfo &)
    {
      std::lock_guard<std::mutex> lock(connect_mutex_);
      if (pub_depth_.getNumSubscribers() == 0) {
        sub_raw_.shutdown();
      } else if (!sub_raw_) {
        // For compressed topics to remap appropriately, we need to pass a
        // fully expanded and remapped topic name to image_transport
        auto node_base = this->get_node_base_interface();
        std::string topic = node_base->resolve_topic_or_service_name("image_raw", false);
        // Get transport hints
        image_transport::TransportHints hints(this);
        // Create publisher with same QoS as subscribed topic publisher
        auto qos_profile = image_proc::getTopicQosProfile(this, topic);
        sub_raw_ = image_transport::create_subscription(
          this, topic,
          std::bind(&CropForemostNode::depthCb, this, std::placeholders::_1),
          hints.getTransport(),
          qos_profile);
      }
    };
  // For compressed topics to remap appropriately, we need to pass a
  // fully expanded and remapped topic name to image_transport
  auto node_base = this->get_node_base_interface();
  std::string topic = node_base->resolve_topic_or_service_name("image", false);

  // Create publisher - allow overriding QoS settings (history, depth, reliability)
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pub_depth_ = image_transport::create_publisher(this, topic, rmw_qos_profile_default,
    pub_options);
}

void CropForemostNode::depthCb(const sensor_msgs::msg::Image::ConstSharedPtr & raw_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(raw_msg);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(logger_, "cv_bridge exception: %s", e.what());
    return;
  }

  // Check the number of channels
  if (sensor_msgs::image_encodings::numChannels(raw_msg->encoding) != 1) {
    RCLCPP_ERROR(
      logger_, "Only grayscale image is acceptable, got [%s]",
      raw_msg->encoding.c_str());
    return;
  }

  // search the min value without invalid value "0"
  double minVal;
  cv::minMaxIdx(cv_ptr->image, &minVal, 0, 0, 0, cv_ptr->image != 0);

  int imtype = cv_bridge::getCvType(raw_msg->encoding);
  switch (imtype) {
    case CV_8UC1:
    case CV_8SC1:
    case CV_32F:
      cv::threshold(cv_ptr->image, cv_ptr->image, minVal + distance_, 0, CV_THRESH_TOZERO_INV);
      break;
    case CV_16UC1:
    case CV_16SC1:
    case CV_32SC1:
    case CV_64F:
      // 8 bit or 32 bit floating array is required to use cv::threshold
      cv_ptr->image.convertTo(cv_ptr->image, CV_32F);
      cv::threshold(cv_ptr->image, cv_ptr->image, minVal + distance_, 1, CV_THRESH_TOZERO_INV);

      cv_ptr->image.convertTo(cv_ptr->image, imtype);
      break;
  }

  auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
  cv_ptr->toImageMsg(*image_msg);
  pub_depth_.publish(std::move(image_msg));
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::CropForemostNode)
