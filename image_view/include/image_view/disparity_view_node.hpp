// Copyright 2019 Joshua Whitley
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

#ifndef IMAGE_VIEW__DISPARITY_VIEW_NODE_HPP_
#define IMAGE_VIEW__DISPARITY_VIEW_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <memory>
#include <string>

namespace image_view
{

class DisparityViewNode
  : public rclcpp::Node
{
public:
  explicit DisparityViewNode(const rclcpp::NodeOptions & options);
  explicit DisparityViewNode(const DisparityViewNode &) = default;
  explicit DisparityViewNode(DisparityViewNode &&) = default;
  DisparityViewNode & operator=(const DisparityViewNode &) = default;
  DisparityViewNode & operator=(DisparityViewNode &&) = default;
  ~DisparityViewNode();

private:
  // colormap for disparities, RGB order
  static unsigned char colormap[];

  std::string window_name_;
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr sub_;
  cv::Mat_<cv::Vec3b> disparity_color_;
  bool initialized;

  void imageCb(const stereo_msgs::msg::DisparityImage::SharedPtr msg);
};

}  // namespace image_view

#endif  // IMAGE_VIEW__DISPARITY_VIEW_NODE_HPP_
