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

#include <vector>
#include <string>
#include <memory>

namespace image_rotate
{

ImageRotateNode::ImageRotateNode()
: Node("ImageRotateNode")
{
  config_.target_frame_id = this->declare_parameter("target_frame_id", std::string(""));
  config_.target_x = this->declare_parameter("target_x", static_cast<double>(0));
  config_.target_y = this->declare_parameter("target_y", static_cast<double>(0));
  config_.target_z = this->declare_parameter("target_z", static_cast<double>(1));

  config_.source_frame_id = this->declare_parameter("source_frame_id", std::string(""));
  config_.source_x = this->declare_parameter("source_x", static_cast<double>(0));
  config_.source_y = this->declare_parameter("source_y", static_cast<double>(-1));
  config_.source_z = this->declare_parameter("source_z", static_cast<double>(0));

  config_.output_frame_id = this->declare_parameter("output_frame_id", std::string(""));
  config_.input_frame_id = this->declare_parameter("input_frame_id", std::string(""));
  config_.use_camera_info = this->declare_parameter("use_camera_info", true);
  config_.max_angular_rate = this->declare_parameter(
    "max_angular_rate",
    static_cast<double>(10));
  config_.output_image_size = this->declare_parameter(
    "output_image_size",
    static_cast<double>(2));

  auto reconfigureCallback =
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      RCLCPP_INFO(get_logger(), "reconfigureCallback");

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
      if (subscriber_count_) {  // @todo: Could do this without an interruption at some point.
        unsubscribe();
        subscribe();
      }
      return result;
    };
  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(reconfigureCallback);
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

    // RCUTILS_LOG_INFO("target: %f %f %f", target_vector_.x, target_vector_.y, target_vector_.z);
    // RCUTILS_LOG_INFO("target_transformed: %f %f %f", target_vector_transformed.x, "
    //  "target_vector_transformed.y, target_vector_transformed.z");
    // RCUTILS_LOG_INFO("source: %f %f %f", source_vector_.x, source_vector_.y, source_vector_.z);
    // RCUTILS_LOG_INFO("source_transformed: %f %f %f", source_vector_transformed.x, "
    //  "source_vector_transformed.y, source_vector_transformed.z");

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
  } catch (tf2::TransformException & e) {
    RCUTILS_LOG_ERROR("Transform error: %s", e.what());
  }

  // RCUTILS_LOG_INFO("angle: %f", 180 * angle_ / M_PI);

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
    // RCUTILS_LOG_INFO("out_size: %d", out_size);

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
    sensor_msgs::msg::Image::SharedPtr out_img =
      cv_bridge::CvImage(msg->header, msg->encoding, out_image).toImageMsg();
    out_img->header.frame_id = transform.child_frame_id;
    img_pub_.publish(out_img);
  } catch (cv::Exception & e) {
    RCUTILS_LOG_ERROR(
      "Image processing error: %s %s %s %i",
      e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
  }

  prev_stamp_ = tf2_ros::fromMsg(msg->header.stamp);
}

void ImageRotateNode::subscribe()
{
  RCUTILS_LOG_DEBUG("Subscribing to image topic.");
  if (config_.use_camera_info && config_.input_frame_id.empty()) {
    auto custom_qos = rmw_qos_profile_system_default;
    custom_qos.depth = 3;

    cam_sub_ = image_transport::create_camera_subscription(
      this,
      "image",
      std::bind(
        &ImageRotateNode::imageCallbackWithInfo, this,
        std::placeholders::_1, std::placeholders::_2),
      "raw",
      custom_qos);
  } else {
    auto custom_qos = rmw_qos_profile_system_default;
    custom_qos.depth = 3;
    img_sub_ = image_transport::create_subscription(
      this,
      "image",
      std::bind(&ImageRotateNode::imageCallback, this, std::placeholders::_1),
      "raw",
      custom_qos);
  }
}

void ImageRotateNode::unsubscribe()
{
  RCUTILS_LOG_DEBUG("Unsubscribing from image topic.");
  img_sub_.shutdown();
  cam_sub_.shutdown();
}

void ImageRotateNode::connectCb()
{
  if (subscriber_count_++ == 0) {
    subscribe();
  }
}

void ImageRotateNode::disconnectCb()
{
  subscriber_count_--;
  if (subscriber_count_ == 0) {
    unsubscribe();
  }
}

void ImageRotateNode::onInit()
{
  subscriber_count_ = 0;
  angle_ = 0;
  prev_stamp_ = tf2::get_now();
  rclcpp::Clock::SharedPtr clock = this->get_clock();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
  tf_sub_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // TODO(yechun1): Implement when SubscriberStatusCallback is available
  // image_transport::SubscriberStatusCallback connect_cb =
  //   boost::bind(&ImageRotateNode::connectCb, this, _1);
  // image_transport::SubscriberStatusCallback connect_cb =
  //   std::bind(&CropForemostNode::connectCb, this);
  // image_transport::SubscriberStatusCallback disconnect_cb =
  //   boost::bind(&ImageRotateNode::disconnectCb, this, _1);
  // img_pub_ = image_transport::ImageTransport(ros::NodeHandle(nh_, "rotated")).advertise(
  //  "image", 1, connect_cb, disconnect_cb);
  connectCb();
  img_pub_ = image_transport::create_publisher(this, "rotated/image");
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}
}  // namespace image_rotate

#include "class_loader/register_macro.hpp"

// Register the component with class_loader.
CLASS_LOADER_REGISTER_CLASS(image_rotate::ImageRotateNode, rclcpp::Node)
