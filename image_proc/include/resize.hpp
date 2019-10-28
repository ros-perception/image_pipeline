/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, 2019, Willow Garage, Inc., Andreas Klintberg.
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
#ifndef IMAGE_PROC_RESIZE_HPP
#define IMAGE_PROC_RESIZE_HPP

#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>

#include <ament_index_cpp/get_resource.hpp>
#include "rclcpp/rclcpp.hpp"

namespace image_proc {

class ResizeNode : public rclcpp::Node
{
public:
  ResizeNode(const rclcpp::NodeOptions &);
protected:
  // ROS communication
  image_transport::Publisher pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
  image_transport::Subscriber sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;

  std::string image_topic_;
  std::string camera_info_topic_;

  // Configuration
  std::string camera_namespace_;
  int interpolation_;
  bool use_scale_;
  double scale_height_;
  double scale_width_;
  int height_;
  int width_;

  std::mutex connect_mutex_;

  void connectCb();

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);
  void infoCb(const sensor_msgs::msg::CameraInfo::SharedPtr info_msg);

};
} // namespace image_proc
#endif