// Copyright 2012, 2013, 2019 Open Source Robotics Foundation, Joshua Whitley
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

#ifndef IMAGE_VIEW__VIDEO_RECORDER_NODE_HPP_
#define IMAGE_VIEW__VIDEO_RECORDER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <memory>
#include <string>

namespace image_view
{

class VideoRecorderNode
  : public rclcpp::Node
{
public:
  explicit VideoRecorderNode(const rclcpp::NodeOptions & options);
  explicit VideoRecorderNode(const VideoRecorderNode &) = default;
  explicit VideoRecorderNode(VideoRecorderNode &&) = default;
  VideoRecorderNode & operator=(const VideoRecorderNode &) = default;
  VideoRecorderNode & operator=(VideoRecorderNode &&) = default;
  ~VideoRecorderNode();

private:
  cv::VideoWriter outputVideo;

  int g_count;
  rclcpp::Time g_last_wrote_time;
  std::string encoding;
  std::string codec;
  int fps;
  double min_depth_range;
  double max_depth_range;
  bool use_dynamic_range;
  int colormap;
  image_transport::Subscriber sub_image;
  bool recording_started;
  std::string filename;

  void callback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
};

}  // namespace image_view

#endif  // IMAGE_VIEW__VIDEO_RECORDER_NODE_HPP_
