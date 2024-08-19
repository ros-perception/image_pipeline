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

/********************************************************************
* image_rotate_node.cpp
* this is a forked version of image_rotate.
* this image_rotate_node supports:
*  1) node
*  2) tf and tf2
*********************************************************************/

#include "image_rotate/image_rotate_node.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace image_rotate
{

ImageRotateNode::ImageRotateNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ImageRotateNode", options)
{
  auto reconfigureCallback =
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters) {
        if (parameter.get_name() == "target_x") {
          config_.target_x = parameter.as_double();
          RCLCPP_INFO(get_logger(), "Reset target_x as '%lf'", config_.target_x);
        } else if (parameter.get_name() == "target_y") {
          config_.target_y = parameter.as_double();
          RCLCPP_INFO(get_logger(), "Reset target_y as '%lf'", config_.target_y);
        } else if (parameter.get_name() == "target_z") {
          config_.target_z = parameter.as_double();
          RCLCPP_INFO(get_logger(), "Reset target_z as '%lf'", config_.target_z);
        } else if (parameter.get_name() == "source_x") {
          config_.source_x = parameter.as_double();
          RCLCPP_INFO(get_logger(), "Reset source_x as '%lf'", config_.source_x);
        } else if (parameter.get_name() == "source_y") {
          config_.source_y = parameter.as_double();
          RCLCPP_INFO(get_logger(), "Reset source_y as '%lf'", config_.source_y);
        } else if (parameter.get_name() == "source_z") {
          config_.source_z = parameter.as_double();
          RCLCPP_INFO(get_logger(), "Reset source_z as '%lf'", config_.source_z);
        }
      }

      target_vector_.vector.x = config_.target_x;
      target_vector_.vector.y = config_.target_y;
      target_vector_.vector.z = config_.target_z;

      source_vector_.vector.x = config_.source_x;
      source_vector_.vector.y = config_.source_y;
      source_vector_.vector.z = config_.source_z;

      return result;
    };
  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(reconfigureCallback);

  // Set parameters AFTER add_on_set_parameters_callback
  config_.target_frame_id = this->declare_parameter("target_frame_id", std::string(""));
  config_.target_x = this->declare_parameter("target_x", 0.0);
  config_.target_y = this->declare_parameter("target_y", 0.0);
  config_.target_z = this->declare_parameter("target_z", 1.0);

  config_.source_frame_id = this->declare_parameter("source_frame_id", std::string(""));
  config_.source_x = this->declare_parameter("source_x", 0.0);
  config_.source_y = this->declare_parameter("source_y", -1.0);
  config_.source_z = this->declare_parameter("source_z", 0.0);

  config_.output_frame_id = this->declare_parameter("output_frame_id", std::string(""));
  config_.input_frame_id = this->declare_parameter("input_frame_id", std::string(""));
  config_.use_camera_info = this->declare_parameter("use_camera_info", true);
  config_.max_angular_rate = this->declare_parameter("max_angular_rate", 10.0);
  config_.output_image_size = this->declare_parameter("output_image_size", 2.0);

  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("image_transport", "raw");

  onInit();
}

const std::string ImageRotateNode::frameWithDefault(
  const std::string & frame,
  const std::string & image_frame)
{
  if (frame.empty()) {
    return image_frame;
  }
  return frame;
}

void ImageRotateNode::imageCallbackWithInfo(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
{
  do_work(msg, cam_info->header.frame_id);
}

void ImageRotateNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  do_work(msg, msg->header.frame_id);
}

void ImageRotateNode::do_work(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg,
  const std::string input_frame_from_msg)
{
  try {
    std::string input_frame_id = frameWithDefault(config_.input_frame_id, input_frame_from_msg);
    std::string target_frame_id = frameWithDefault(config_.target_frame_id, input_frame_from_msg);
    std::string source_frame_id = frameWithDefault(config_.source_frame_id, input_frame_from_msg);

    // Transform the target vector into the image frame.
    target_vector_.header.stamp = msg->header.stamp;
    target_vector_.header.frame_id = target_frame_id;
    geometry_msgs::msg::Vector3Stamped target_vector_transformed;
    tf2::TimePoint tf2_time = tf2_ros::fromMsg(msg->header.stamp);

    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      target_frame_id, input_frame_id, tf2_time, tf2_time - prev_stamp_);
    tf2::doTransform(target_vector_, target_vector_transformed, transform);

    // Transform the source vector into the image frame.
    source_vector_.header.stamp = msg->header.stamp;
    source_vector_.header.frame_id = source_frame_id;
    geometry_msgs::msg::Vector3Stamped source_vector_transformed;
    transform = tf_buffer_->lookupTransform(
      source_frame_id, input_frame_id, tf2_time, tf2_time - prev_stamp_);
    tf2::doTransform(source_vector_, source_vector_transformed, transform);

    // Calculate the angle of the rotation.
    double angle = angle_;
    if ((target_vector_transformed.vector.x != 0 || target_vector_transformed.vector.y != 0) &&
      (source_vector_transformed.vector.x != 0 || source_vector_transformed.vector.y != 0))
    {
      angle = atan2(target_vector_transformed.vector.y, target_vector_transformed.vector.x);
      angle -= atan2(source_vector_transformed.vector.y, source_vector_transformed.vector.x);
    }

    // Rate limit the rotation.
    if (config_.max_angular_rate == 0) {
      angle_ = angle;
    } else {
      double delta = fmod(angle - angle_, 2.0 * M_PI);
      if (delta > M_PI) {
        delta -= 2.0 * M_PI;
      } else if (delta < -M_PI) {
        delta += 2.0 * M_PI;
      }

      double max_delta = config_.max_angular_rate *
        (tf2_ros::timeToSec(msg->header.stamp) - tf2::timeToSec(prev_stamp_));
      if (delta > max_delta) {
        delta = max_delta;
      } else if (delta < -max_delta) {
        delta = -max_delta;
      }

      angle_ += delta;
    }
    angle_ = fmod(angle_, 2.0 * M_PI);
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", e.what());
  }

  // Publish the transform.
  geometry_msgs::msg::TransformStamped transform;
  transform.transform.translation.x = 0;
  transform.transform.translation.y = 0;
  transform.transform.translation.z = 0;
  transform.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), angle_));
  transform.header.frame_id = msg->header.frame_id;
  transform.child_frame_id = frameWithDefault(
    config_.output_frame_id, msg->header.frame_id + "_rotated");
  transform.header.stamp = msg->header.stamp;

  if (tf_pub_) {
    tf_pub_->sendTransform(transform);
  }

  // Transform the image.
  try {
    // Convert the image into something opencv can handle.
    cv::Mat in_image = cv_bridge::toCvShare(msg, msg->encoding)->image;

    // Compute the output image size.
    int max_dim = in_image.cols > in_image.rows ? in_image.cols : in_image.rows;
    int min_dim = in_image.cols < in_image.rows ? in_image.cols : in_image.rows;
    int noblack_dim = min_dim / sqrt(2);
    int diag_dim = sqrt(in_image.cols * in_image.cols + in_image.rows * in_image.rows);
    int out_size;
    // diag_dim repeated to simplify limit case.
    int candidates[] = {noblack_dim, min_dim, max_dim, diag_dim, diag_dim};
    int step = config_.output_image_size;
    out_size = candidates[step] + (candidates[step + 1] - candidates[step]) *
      (config_.output_image_size - step);

    // Compute the rotation matrix.
    cv::Mat rot_matrix = cv::getRotationMatrix2D(
      cv::Point2f(in_image.cols / 2.0, in_image.rows / 2.0), 180 * angle_ / M_PI, 1);
    cv::Mat translation = rot_matrix.col(2);
    rot_matrix.at<double>(0, 2) += (out_size - in_image.cols) / 2.0;
    rot_matrix.at<double>(1, 2) += (out_size - in_image.rows) / 2.0;

    // Do the rotation
    cv::Mat out_image;
    cv::warpAffine(in_image, out_image, rot_matrix, cv::Size(out_size, out_size));

    // Publish the image.
    auto out_img = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(msg->header, msg->encoding, out_image).toImageMsg(*out_img);
    out_img->header.frame_id = transform.child_frame_id;
    img_pub_.publish(std::move(out_img));
  } catch (const cv::Exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Image processing error: %s %s %s %i",
      e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
  }

  prev_stamp_ = tf2_ros::fromMsg(msg->header.stamp);
}

void ImageRotateNode::onInit()
{
  subscriber_count_ = 0;
  angle_ = 0;
  prev_stamp_ = tf2::get_now();
  rclcpp::Clock::SharedPtr clock = this->get_clock();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
  tf_sub_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  // Create publisher with connect callback
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.matched_callback =
    [this](rclcpp::MatchedInfo &)
    {
      if (img_pub_.getNumSubscribers() == 0) {
        RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
        img_sub_.shutdown();
        cam_sub_.shutdown();
      } else {
        // For compressed topics to remap appropriately, we need to pass a
        // fully expanded and remapped topic name to image_transport
        auto node_base = this->get_node_base_interface();
        std::string topic_name = node_base->resolve_topic_or_service_name("image", false);
        RCLCPP_INFO(get_logger(), "Subscribing to %s topic.", topic_name.c_str());

        // Check that remapping appears to be correct
        auto topics = this->get_topic_names_and_types();
        if (topics.find(topic_name) == topics.end()) {
          RCLCPP_WARN(
            get_logger(),
            "Topic %s is not connected! Typical command-line usage:\n"
            "\t$ ros2 run image_rotate image_rotate --ros-args -r image:=<image topic>",
            topic_name.c_str());
        }

        // This will check image_transport parameter to get proper transport
        image_transport::TransportHints transport_hint(this, "raw");

        if (config_.use_camera_info && config_.input_frame_id.empty()) {
          auto custom_qos = rmw_qos_profile_system_default;
          custom_qos.depth = 3;
          cam_sub_ = image_transport::create_camera_subscription(
            this,
            topic_name,
            std::bind(
              &ImageRotateNode::imageCallbackWithInfo, this,
              std::placeholders::_1, std::placeholders::_2),
            transport_hint.getTransport(),
            custom_qos);
        } else {
          auto custom_qos = rmw_qos_profile_system_default;
          custom_qos.depth = 3;
          img_sub_ = image_transport::create_subscription(
            this,
            topic_name,
            std::bind(&ImageRotateNode::imageCallback, this, std::placeholders::_1),
            transport_hint.getTransport(),
            custom_qos);
        }
      }
    };

  // For compressed topics to remap appropriately, we need to pass a
  // fully expanded and remapped topic name to image_transport
  auto node_base = this->get_node_base_interface();
  std::string topic = node_base->resolve_topic_or_service_name("rotated/image", false);

  // Allow overriding QoS settings (history, depth, reliability)
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  img_pub_ = image_transport::create_publisher(this, topic, rmw_qos_profile_default, pub_options);
}
}  // namespace image_rotate

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(image_rotate::ImageRotateNode)
