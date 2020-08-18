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

#ifndef IMAGE_VIEW__IMAGE_VIEW_NODE_HPP_
#define IMAGE_VIEW__IMAGE_VIEW_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <boost/format.hpp>

#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>

namespace image_view
{

class ThreadSafeImage
{
  std::mutex mutex_;
  std::condition_variable condition_;
  cv_bridge::CvImageConstPtr image_;

public:
  void set(cv_bridge::CvImageConstPtr image);
  cv_bridge::CvImageConstPtr get();
  cv_bridge::CvImageConstPtr pop();
};

class ImageViewNode
  : public rclcpp::Node
{
public:
  explicit ImageViewNode(const rclcpp::NodeOptions & options);
  explicit ImageViewNode(const ImageViewNode &) = default;
  explicit ImageViewNode(ImageViewNode &&) = default;
  ImageViewNode & operator=(const ImageViewNode &) = default;
  ImageViewNode & operator=(ImageViewNode &&) = default;
  ~ImageViewNode();

private:
  ThreadSafeImage queued_image_, shown_image_;
  bool autosize_;
  int window_height_, window_width_;
  bool g_gui;
  boost::format filename_format_;
  image_transport::Subscriber sub_;
  int count_;
  double min_image_value_, max_image_value_;
  int colormap_;
  rclcpp::TimerBase::SharedPtr gui_timer_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_;
  std::string window_name_;
  std::thread window_thread_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  static void mouseCb(int event, int x, int y, int flags, void * param);
  void windowThread();
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> &);
  std::mutex param_mutex_;
};

}  // namespace image_view

#endif  // IMAGE_VIEW__IMAGE_VIEW_NODE_HPP_
