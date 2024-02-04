// Copyright 2024 Open Navigation LLC
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

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <image_proc/track_marker.hpp>
#include <image_proc/utils.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/quaternion.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

namespace image_proc
{

TrackMarkerNode::TrackMarkerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("TrackMarkerNode", options)
{
  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("image_transport", "raw");

  // For compressed topics to remap appropriately, we need to pass a
  // fully expanded and remapped topic name to image_transport
  auto node_base = this->get_node_base_interface();
  image_topic_ = node_base->resolve_topic_or_service_name("image", false);

  // Declare parameters before we setup any publishers or subscribers
  marker_id_ = this->declare_parameter("marker_id", 0);
  marker_size_ = this->declare_parameter("marker_size", 0.05);
  // Default dictionary is cv::aruco::DICT_6X6_250
  int dict_id = this->declare_parameter("dictionary", 10);

  detector_params_ = cv::aruco::DetectorParameters::create();
  dictionary_ = cv::aruco::getPredefinedDictionary(dict_id);

  // Setup lazy subscriber using publisher connection callback
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.matched_callback =
    [this](rclcpp::MatchedInfo &)
    {
      if (pub_->get_subscription_count() == 0) {
        sub_camera_.shutdown();
      } else if (!sub_camera_) {
        // Create subscriber with QoS matched to subscribed topic publisher
        auto qos_profile = getTopicQosProfile(this, image_topic_);
        image_transport::TransportHints hints(this);
        sub_camera_ = image_transport::create_camera_subscription(
          this, image_topic_, std::bind(
            &TrackMarkerNode::imageCb,
            this, std::placeholders::_1, std::placeholders::_2),
          hints.getTransport(), qos_profile);
      }
    };

  // Create publisher - allow overriding QoS settings (history, depth, reliability)
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "tracked_pose", 10, pub_options);
}

void TrackMarkerNode::imageCb(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(image_msg);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::aruco::detectMarkers(cv_ptr->image, dictionary_, marker_corners, marker_ids);

  for (size_t i = 0; i < marker_ids.size(); ++i) {
    if (marker_ids[i] == marker_id_) {
      // This is our desired marker
      std::vector<std::vector<cv::Point2f>> corners;
      corners.push_back(marker_corners[i]);

      // Copy the matrices since they are const and OpenCV functions are not
      auto k = info_msg->k;
      auto d = info_msg->d;

      // Get the camera info
      cv::Mat intrinsics(3, 3, CV_64FC1, reinterpret_cast<void *>(k.data()));
      cv::Mat dist_coeffs(info_msg->d.size(), 1, CV_64FC1, reinterpret_cast<void *>(d.data()));

      // Estimate pose
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(
        corners, marker_size_, intrinsics, dist_coeffs, rvecs, tvecs);

      // Publish pose of marker
      geometry_msgs::msg::PoseStamped pose;
      pose.header = image_msg->header;
      // Fill in pose
      pose.pose.position.x = tvecs[0][0];
      pose.pose.position.y = tvecs[0][1];
      pose.pose.position.z = tvecs[0][2];
      // Convert angle-axis to quaternion
      cv::Quatd q = cv::Quatd::createFromRvec(rvecs[0]);
      pose.pose.orientation.x = q.x;
      pose.pose.orientation.y = q.y;
      pose.pose.orientation.z = q.z;
      pose.pose.orientation.w = q.w;
      pub_->publish(pose);
    }
  }
}

}  // namespace image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_proc::TrackMarkerNode)
