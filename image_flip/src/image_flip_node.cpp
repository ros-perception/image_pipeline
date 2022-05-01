// Copyright (c) 2022, CHRISLab, Christopher Newport University
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
* image_flip_node.cpp
* this is a forked version of image_flip.
* this image_flip_node supports:
*  1) node
*  2) tf and tf2
*********************************************************************/

#include "image_flip/image_flip_node.hpp"

#include <vector>
#include <string>
#include <memory>
#include <opencv2/core/core.hpp>

namespace image_flip
{

ImageFlipNode::ImageFlipNode(rclcpp::NodeOptions options)
: Node("ImageFlipNode", options)
{

  config_.output_frame_id = this->declare_parameter("output_frame_id", std::string(""));
  config_.rotation_steps = this->declare_parameter("rotation_steps", 2);
  config_.use_camera_info = this->declare_parameter("use_camera_info", true);
  config_.in_image_topic_name = this->declare_parameter("in_image_topic_name", std::string("image"));
  config_.out_image_topic_name = this->declare_parameter("out_image_topic_name", std::string("rotated_image"));

  auto reconfigureCallback =
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      RCLCPP_INFO(get_logger(), "reconfigureCallback");

      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters) {
        if (parameter.get_name() == "output_frame_id") {
          config_.output_frame_id = parameter.as_string();
          RCLCPP_INFO(get_logger(), "Reset output_frame_id '%s'", config_.output_frame_id.c_str());
        } else if (parameter.get_name() == "rotation_steps") {
          config_.rotation_steps = parameter.as_int();
          angle_ = config_.rotation_steps * M_PI / 2.0;
          RCLCPP_INFO(get_logger(), "Reset rotation_steps as '%d'", config_.rotation_steps);
          transform_.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), angle_));

        }
      }

      if (subscriber_count_) {  // @todo: Could do this without an interruption at some point.
        unsubscribe();
        subscribe();
      }
      return result;
    };
  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(reconfigureCallback);
  onInit();
  angle_ = config_.rotation_steps * M_PI / 2.0;
  transform_.transform.translation.x = 0;
  transform_.transform.translation.y = 0;
  transform_.transform.translation.z = 0;
  transform_.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), angle_));


}

const std::string ImageFlipNode::frameWithDefault(
  const std::string & frame,
  const std::string & image_frame)
{
  if (frame.empty()) {
    return image_frame;
  }
  return frame;
}

void ImageFlipNode::imageCallbackWithInfo(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
{
  std::string frame_id = cam_info->header.frame_id;
  if (frame_id.length() == 0) {
    frame_id = msg->header.frame_id;
  }
  do_work(msg, cam_info, frame_id);
}

void ImageFlipNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  do_work(msg, nullptr, msg->header.frame_id);
}

void ImageFlipNode::do_work(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info,
  const std::string input_frame_from_msg)
{

  // Transform the image.
  try {
    // Convert the image into something opencv can handle.
    cv::Mat in_image = cv_bridge::toCvShare(msg, msg->encoding)->image;
    cv::Mat out_image;

    // based on https://stackoverflow.com/questions/15043152/rotate-opencv-matrix-by-90-180-270-degrees
    if (config_.rotation_steps == 1) {
      transpose(in_image, out_image);
      flip(out_image, out_image,0); //transpose+flip(0)=CCW
    } else if (config_.rotation_steps ==2){
      flip(in_image, out_image,-1);    //flip(-1)=180
    } else if (config_.rotation_steps == 3){
      transpose(in_image, out_image);
      flip(out_image, out_image,1); //transpose+flip(1)=CW
    } else { //if not 0,1,2,3:
      RCLCPP_WARN(get_logger(), "Unknown rotation_steps %d", config_.rotation_steps);
      out_image = in_image;
    }

    // Update the transform
    transform_.header.frame_id = input_frame_from_msg;
    transform_.child_frame_id = frameWithDefault(config_.output_frame_id, input_frame_from_msg + "_rotated");
    transform_.header.stamp = msg->header.stamp;

    // Publish the image.
    sensor_msgs::msg::Image::SharedPtr out_img =
      cv_bridge::CvImage(msg->header, msg->encoding, out_image).toImageMsg();
    out_img->header.frame_id = transform_.child_frame_id;

    if (cam_pub_) {
      sensor_msgs::msg::CameraInfo::SharedPtr out_info(new sensor_msgs::msg::CameraInfo(*cam_info));
      out_info->header.frame_id = out_img->header.frame_id;
      out_info->height = out_img->height;
      out_info->width = out_img->width;
      cam_pub_.publish(out_img, out_info);
    }
    else {
      img_pub_.publish(out_img);
    }

    // Publish the transform.

    if (tf_pub_) {
      tf_pub_->sendTransform(transform_);
    }


  } catch (cv::Exception & e) {
    RCUTILS_LOG_ERROR(
      "Image processing error: %s %s %s %i",
      e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
  }

  prev_stamp_ = tf2_ros::fromMsg(msg->header.stamp);
}

void ImageFlipNode::subscribe()
{
  // This is a foxy hack while waiting on rclcpp resolve_topic_name
  std::string image_topic = config_.in_image_topic_name;
  RCUTILS_LOG_INFO("Subscribing to image topic %s.", image_topic.c_str());

  if (config_.use_camera_info) {
    auto custom_qos = rmw_qos_profile_sensor_data; // To match Gazebo 11 pub
    cam_sub_ = image_transport::create_camera_subscription(
      this,
      image_topic, //"image",
      std::bind(
        &ImageFlipNode::imageCallbackWithInfo, this,
        std::placeholders::_1, std::placeholders::_2),
      "raw",
      custom_qos);
  } else {
    auto custom_qos = rmw_qos_profile_sensor_data; // To match Gazebo 11 pub
    img_sub_ = image_transport::create_subscription(
      this,
      image_topic, //"image",
      std::bind(&ImageFlipNode::imageCallback, this, std::placeholders::_1),
      "raw",
      custom_qos);
  }
}

void ImageFlipNode::unsubscribe()
{
  RCUTILS_LOG_DEBUG("Unsubscribing from image topic.");
  img_sub_.shutdown();
  cam_sub_.shutdown();
}

void ImageFlipNode::connectCb()
{
  if (subscriber_count_++ == 0) {
    subscribe();
  }
}

void ImageFlipNode::disconnectCb()
{
  subscriber_count_--;
  if (subscriber_count_ == 0) {
    unsubscribe();
  }
}

void ImageFlipNode::onInit()
{
  subscriber_count_ = 0;
  angle_ = 0;
  prev_stamp_ = tf2::get_now();
  rclcpp::Clock::SharedPtr clock = this->get_clock();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
  tf_sub_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // --- Note: From image_rotate (foxy branch)
  // TODO(yechun1): Implement when SubscriberStatusCallback is available
  // image_transport::SubscriberStatusCallback connect_cb =
  //   boost::bind(&ImageFlipNode::connectCb, this, _1);
  // image_transport::SubscriberStatusCallback connect_cb =
  //   std::bind(&CropForemostNode::connectCb, this);
  // image_transport::SubscriberStatusCallback disconnect_cb =
  //   boost::bind(&ImageFlipNode::disconnectCb, this, _1);
  // img_pub_ = image_transport::ImageTransport(ros::NodeHandle(nh_, "rotated")).advertise(
  //  "image", 1, connect_cb, disconnect_cb);
  //----------------------------------------------------

  connectCb();
  // This is a foxy hack while waiting on rclcpp resolve_topic_name
  std::string out_image_topic = config_.out_image_topic_name;
  RCUTILS_LOG_DEBUG("Advertising to image topic %s.", out_image_topic.c_str());
  auto custom_qos = rmw_qos_profile_sensor_data; // To match Gazebo 11 pub

  if (config_.use_camera_info) {
    cam_pub_ = image_transport::create_camera_publisher(this, out_image_topic, custom_qos);
  } else {
    img_pub_ = image_transport::create_publisher(this, out_image_topic, custom_qos);
  }

  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}
}  // namespace image_flip

// Register the component with class_loader.
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(image_flip::ImageFlipNode)
