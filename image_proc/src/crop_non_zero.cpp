// Copyright 2008, 2019 Willow Garage, Inc., Steve Macenski, Joshua Whitley
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

#include <algorithm>
#include <functional>
#include <iterator>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"

#include <image_proc/crop_non_zero.hpp>
#include <image_proc/utils.hpp>

#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace image_proc
{

CropNonZeroNode::CropNonZeroNode(const rclcpp::NodeOptions & options)
: Node("CropNonZeroNode", options)
{
  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("image_transport", "raw");

  // For compressed topics to remap appropriately, we need to pass a
  // fully expanded and remapped topic name to image_transport
  auto node_base = this->get_node_base_interface();
  image_topic_ = node_base->resolve_topic_or_service_name("image_raw", false);
  std::string pub_topic = node_base->resolve_topic_or_service_name("image", false);

  // Setup lazy subscriber using publisher connection callback
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.matched_callback =
    [this](rclcpp::MatchedInfo &)
    {
      if (pub_.getNumSubscribers() == 0) {
        sub_raw_.shutdown();
      } else if (!sub_raw_) {
        // Create subscriber with QoS matched to subscribed topic publisher
        auto qos_profile = getTopicQosProfile(this, image_topic_);
        image_transport::TransportHints hints(this);
        sub_raw_ = image_transport::create_subscription(
          this, image_topic_, std::bind(
            &CropNonZeroNode::imageCb, this,
            std::placeholders::_1), hints.getTransport(), qos_profile);
      }
    };

  // Create publisher - allow overriding QoS settings (history, depth, reliability)
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pub_ = image_transport::create_publisher(this, pub_topic, rmw_qos_profile_default, pub_options);
}

void CropNonZeroNode::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & raw_msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(raw_msg);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Check the number of channels
  if (sensor_msgs::image_encodings::numChannels(raw_msg->encoding) != 1) {
    RCLCPP_ERROR(
      this->get_logger(), "Only grayscale image is acceptable, got [%s]",
      raw_msg->encoding.c_str());
    return;
  }
  std::vector<std::vector<cv::Point>> cnt;
  cv::Mat1b m(raw_msg->width, raw_msg->height);

  if (raw_msg->encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
    m = cv_ptr->image;
  } else {
    double minVal, maxVal;
    cv::minMaxIdx(cv_ptr->image, &minVal, &maxVal, 0, 0, cv_ptr->image != 0.);
    double ra = maxVal - minVal;

    cv_ptr->image.convertTo(m, CV_8U, 255. / ra, -minVal * 255. / ra);
  }

  cv::findContours(m, cnt, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  // search the largest area
  std::vector<std::vector<cv::Point>>::iterator it =
    std::max_element(
    cnt.begin(), cnt.end(), [](std::vector<cv::Point> a,
    std::vector<cv::Point> b) {
      return a.size() < b.size();
    });

  cv::Rect r = cv::boundingRect(cnt[std::distance(cnt.begin(), it)]);

  cv_bridge::CvImage out_msg;
  out_msg.header = raw_msg->header;
  out_msg.encoding = raw_msg->encoding;
  out_msg.image = cv_ptr->image(r);

  auto out_image = std::make_unique<sensor_msgs::msg::Image>();
  out_msg.toImageMsg(*out_image);

  pub_.publish(std::move(out_image));
}

}  // namespace image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_proc::CropNonZeroNode)
