/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <thread>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include "visibility.h"
#include "../include/rectify.hpp"
namespace image_proc {

RectifyNode::RectifyNode()
: Node("RectifyNode")
{
    auto parameter_change_cb =
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters) {
        if (parameter.get_name() == "camera_namespace") {
          camera_namespace_ = parameter.as_string();
          RCLCPP_INFO(get_logger(), "camera_namespace: %s ", camera_namespace_.c_str());
          break;
        }
      }
      for (auto parameter : parameters){
        if (parameter.get_name() == "image_mono") {
          image_topic = camera_namespace_ + parameter.as_string();
          image_rect = camera_namespace_+"/image_rect";
          RCLCPP_INFO(get_logger(), "image_topic: %s, image_rect: %s", image_topic.c_str(), image_rect.c_str());
          connectCb();
          std::lock_guard<std::mutex> lock(connect_mutex_);
          pub_rect_ = image_transport::create_publisher(this, image_rect);
          break;
        }
        if (parameter.get_name() == "image_color") {
          image_topic = camera_namespace_ + parameter.as_string();
          image_rect = camera_namespace_ + "/image_rect_color";
          RCLCPP_INFO(get_logger(), "image_topic: %s, image_rect: %s", image_topic.c_str(), image_rect.c_str());
          connectCb();
          std::lock_guard<std::mutex> lock(connect_mutex_);
          pub_rect_ = image_transport::create_publisher(this, image_rect);
          break;
        }
      }
      return result;
    };
  this->get_parameter_or("queue_size", queue_size_, 5);
  this->get_parameter_or("interpolation", interpolation, 0);
  this->register_param_change_callback(parameter_change_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void RectifyNode::connectCb( )
{
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (0)
    sub_camera_.shutdown();
  else if (!sub_camera_)
  {
    sub_camera_ = image_transport::create_camera_subscription(this, image_topic,
                      std::bind(&RectifyNode::imageCb, 
                                this, std::placeholders::_1, std::placeholders::_2),"raw");
  }
}

void RectifyNode::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  // Verify camera is actually calibrated
  if (info_msg->k[0] == 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Rectified topic '%s' requested but camera publishing '%s' "
                           "is uncalibrated", pub_rect_.getTopic().c_str(),
                           sub_camera_.getInfoTopic().c_str());
    return;
  }

  // If zero distortion, just pass the message along
  bool zero_distortion = true;
  for (size_t i = 0; i < info_msg->d.size(); ++i)
  {
    if (info_msg->d[i] != 0.0)
    {
      zero_distortion = false;
      break;
    }
  }
  // This will be true if D is empty/zero sized
  if (zero_distortion)
  {
    pub_rect_.publish(image_msg);
    return;
  }

  // Update the camera model
  model_.fromCameraInfo(info_msg);
  
  // Create cv::Mat views onto both buffers
  const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
  cv::Mat rect;

  // Rectify and publish
  model_.rectifyImage(image, rect, interpolation);

  // Allocate new rectified image message
  sensor_msgs::msg::Image::SharedPtr rect_msg = cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect).toImageMsg();
  pub_rect_.publish(rect_msg);
}
} // namespace image_proc
