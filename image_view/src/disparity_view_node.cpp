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

// Copyright 2019, Joshua Whitley
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "image_view/disparity_view_node.hpp"

#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include <algorithm>
#include <string>

namespace image_view
{

DisparityViewNode::DisparityViewNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("disparity_view_node", options)
{
  std::string topic =
    rclcpp::expand_topic_or_service_name("image", this->get_name(), this->get_namespace());

  if (topic == "image") {
    RCLCPP_WARN(
      this->get_logger(), "Topic 'image' has not been remapped! Typical command-line usage:\n"
      "\t$ rosrun image_view disparity_view image:=<disparity image topic>");
  }

  initialized = false;

  // Default window name is the resolved topic name
  window_name_ = this->declare_parameter("window_name", topic);
  // bool autosize = this->declare_parameter("autosize", false);

  // cv::namedWindow(window_name_, autosize ? cv::WND_PROP_AUTOSIZE : 0);

  sub_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(
    topic, rclcpp::QoS(10), std::bind(&DisparityViewNode::imageCb, this, std::placeholders::_1));
}

DisparityViewNode::~DisparityViewNode()
{
  cv::destroyWindow(window_name_);
}

void DisparityViewNode::imageCb(const stereo_msgs::msg::DisparityImage::SharedPtr msg)
{
  // Check for common errors in input
  if (msg->min_disparity == 0.0 && msg->max_disparity == 0.0) {
    RCLCPP_ERROR_EXPRESSION(
      this->get_logger(), (static_cast<int>(this->now().seconds()) % 30 == 0),
      "Disparity image fields min_disparity and max_disparity are not set");
    return;
  }

  if (msg->image.encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
    RCLCPP_ERROR_EXPRESSION(
      this->get_logger(), (static_cast<int>(this->now().seconds()) % 30 == 0),
      "Disparity image must be 32-bit floating point (encoding '32FC1'), but has encoding '%s'",
      msg->image.encoding.c_str());
    return;
  }

  if (!initialized) {
    cv::namedWindow(window_name_, false ? cv::WND_PROP_AUTOSIZE : 0);
    initialized = true;
  }

  // Colormap and display the disparity image
  float min_disparity = msg->min_disparity;
  float max_disparity = msg->max_disparity;
  float multiplier = 255.0f / (max_disparity - min_disparity);

  const cv::Mat_<float> dmat(
    msg->image.height, msg->image.width,
    reinterpret_cast<float *>(&msg->image.data[0]), msg->image.step);
  disparity_color_.create(msg->image.height, msg->image.width);

  for (int row = 0; row < disparity_color_.rows; ++row) {
    const float * d = dmat[row];
    cv::Vec3b * disparity_color = disparity_color_[row];
    cv::Vec3b * disparity_color_end = disparity_color + disparity_color_.cols;

    for (; disparity_color < disparity_color_end; ++disparity_color, ++d) {
      int index = (*d - min_disparity) * multiplier + 0.5;
      index = std::min(255, std::max(0, index));
      // Fill as BGR
      (*disparity_color)[2] = colormap[3 * index + 0];
      (*disparity_color)[1] = colormap[3 * index + 1];
      (*disparity_color)[0] = colormap[3 * index + 2];
    }
  }

  cv::imshow(window_name_, disparity_color_);
  cv::waitKey(10);
}

unsigned char DisparityViewNode::colormap[768] =
{150, 150, 150,
  107, 0, 12,
  106, 0, 18,
  105, 0, 24,
  103, 0, 30,
  102, 0, 36,
  101, 0, 42,
  99, 0, 48,
  98, 0, 54,
  97, 0, 60,
  96, 0, 66,
  94, 0, 72,
  93, 0, 78,
  92, 0, 84,
  91, 0, 90,
  89, 0, 96,
  88, 0, 102,
  87, 0, 108,
  85, 0, 114,
  84, 0, 120,
  83, 0, 126,
  82, 0, 131,
  80, 0, 137,
  79, 0, 143,
  78, 0, 149,
  77, 0, 155,
  75, 0, 161,
  74, 0, 167,
  73, 0, 173,
  71, 0, 179,
  70, 0, 185,
  69, 0, 191,
  68, 0, 197,
  66, 0, 203,
  65, 0, 209,
  64, 0, 215,
  62, 0, 221,
  61, 0, 227,
  60, 0, 233,
  59, 0, 239,
  57, 0, 245,
  56, 0, 251,
  55, 0, 255,
  54, 0, 255,
  52, 0, 255,
  51, 0, 255,
  50, 0, 255,
  48, 0, 255,
  47, 0, 255,
  46, 0, 255,
  45, 0, 255,
  43, 0, 255,
  42, 0, 255,
  41, 0, 255,
  40, 0, 255,
  38, 0, 255,
  37, 0, 255,
  36, 0, 255,
  34, 0, 255,
  33, 0, 255,
  32, 0, 255,
  31, 0, 255,
  29, 0, 255,
  28, 0, 255,
  27, 0, 255,
  26, 0, 255,
  24, 0, 255,
  23, 0, 255,
  22, 0, 255,
  20, 0, 255,
  19, 0, 255,
  18, 0, 255,
  17, 0, 255,
  15, 0, 255,
  14, 0, 255,
  13, 0, 255,
  11, 0, 255,
  10, 0, 255,
  9, 0, 255,
  8, 0, 255,
  6, 0, 255,
  5, 0, 255,
  4, 0, 255,
  3, 0, 255,
  1, 0, 255,
  0, 4, 255,
  0, 10, 255,
  0, 16, 255,
  0, 22, 255,
  0, 28, 255,
  0, 34, 255,
  0, 40, 255,
  0, 46, 255,
  0, 52, 255,
  0, 58, 255,
  0, 64, 255,
  0, 70, 255,
  0, 76, 255,
  0, 82, 255,
  0, 88, 255,
  0, 94, 255,
  0, 100, 255,
  0, 106, 255,
  0, 112, 255,
  0, 118, 255,
  0, 124, 255,
  0, 129, 255,
  0, 135, 255,
  0, 141, 255,
  0, 147, 255,
  0, 153, 255,
  0, 159, 255,
  0, 165, 255,
  0, 171, 255,
  0, 177, 255,
  0, 183, 255,
  0, 189, 255,
  0, 195, 255,
  0, 201, 255,
  0, 207, 255,
  0, 213, 255,
  0, 219, 255,
  0, 225, 255,
  0, 231, 255,
  0, 237, 255,
  0, 243, 255,
  0, 249, 255,
  0, 255, 255,
  0, 255, 249,
  0, 255, 243,
  0, 255, 237,
  0, 255, 231,
  0, 255, 225,
  0, 255, 219,
  0, 255, 213,
  0, 255, 207,
  0, 255, 201,
  0, 255, 195,
  0, 255, 189,
  0, 255, 183,
  0, 255, 177,
  0, 255, 171,
  0, 255, 165,
  0, 255, 159,
  0, 255, 153,
  0, 255, 147,
  0, 255, 141,
  0, 255, 135,
  0, 255, 129,
  0, 255, 124,
  0, 255, 118,
  0, 255, 112,
  0, 255, 106,
  0, 255, 100,
  0, 255, 94,
  0, 255, 88,
  0, 255, 82,
  0, 255, 76,
  0, 255, 70,
  0, 255, 64,
  0, 255, 58,
  0, 255, 52,
  0, 255, 46,
  0, 255, 40,
  0, 255, 34,
  0, 255, 28,
  0, 255, 22,
  0, 255, 16,
  0, 255, 10,
  0, 255, 4,
  2, 255, 0,
  8, 255, 0,
  14, 255, 0,
  20, 255, 0,
  26, 255, 0,
  32, 255, 0,
  38, 255, 0,
  44, 255, 0,
  50, 255, 0,
  56, 255, 0,
  62, 255, 0,
  68, 255, 0,
  74, 255, 0,
  80, 255, 0,
  86, 255, 0,
  92, 255, 0,
  98, 255, 0,
  104, 255, 0,
  110, 255, 0,
  116, 255, 0,
  122, 255, 0,
  128, 255, 0,
  133, 255, 0,
  139, 255, 0,
  145, 255, 0,
  151, 255, 0,
  157, 255, 0,
  163, 255, 0,
  169, 255, 0,
  175, 255, 0,
  181, 255, 0,
  187, 255, 0,
  193, 255, 0,
  199, 255, 0,
  205, 255, 0,
  211, 255, 0,
  217, 255, 0,
  223, 255, 0,
  229, 255, 0,
  235, 255, 0,
  241, 255, 0,
  247, 255, 0,
  253, 255, 0,
  255, 251, 0,
  255, 245, 0,
  255, 239, 0,
  255, 233, 0,
  255, 227, 0,
  255, 221, 0,
  255, 215, 0,
  255, 209, 0,
  255, 203, 0,
  255, 197, 0,
  255, 191, 0,
  255, 185, 0,
  255, 179, 0,
  255, 173, 0,
  255, 167, 0,
  255, 161, 0,
  255, 155, 0,
  255, 149, 0,
  255, 143, 0,
  255, 137, 0,
  255, 131, 0,
  255, 126, 0,
  255, 120, 0,
  255, 114, 0,
  255, 108, 0,
  255, 102, 0,
  255, 96, 0,
  255, 90, 0,
  255, 84, 0,
  255, 78, 0,
  255, 72, 0,
  255, 66, 0,
  255, 60, 0,
  255, 54, 0,
  255, 48, 0,
  255, 42, 0,
  255, 36, 0,
  255, 30, 0,
  255, 24, 0,
  255, 18, 0,
  255, 12, 0,
  255, 6, 0,
  255, 0, 0,
};

}  // namespace image_view

RCLCPP_COMPONENTS_REGISTER_NODE(image_view::DisparityViewNode)
