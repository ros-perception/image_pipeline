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

#include "image_view/video_recorder_node.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <camera_calibration_parsers/parse.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <iostream>
#include <memory>
#include <string>

namespace image_view
{

VideoRecorderNode::VideoRecorderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("video_recorder_node", options),
  g_count(0),
  g_last_wrote_time(rclcpp::Time((int64_t) 0, RCL_ROS_TIME)),
  recording_started(false)
{
  bool stamped_filename;

  filename = this->declare_parameter("filename", std::string("output.avi"));
  stamped_filename = this->declare_parameter("stamped_filename", false);
  fps = this->declare_parameter("fps", 15);
  codec = this->declare_parameter("codec", std::string("MJPG"));
  encoding = this->declare_parameter("encoding", std::string("bgr8"));
  // cv_bridge::CvtColorForDisplayOptions
  min_depth_range = this->declare_parameter("min_depth_range", 0.0);
  max_depth_range = this->declare_parameter("max_depth_range", 0.0);
  use_dynamic_range = this->declare_parameter("use_dynamic_depth_range", false);
  colormap = this->declare_parameter("colormap", -1);

  if (stamped_filename) {
    std::size_t found = filename.find_last_of("/\\");
    std::string path = filename.substr(0, found + 1);
    std::string basename = filename.substr(found + 1);
    std::stringstream ss;
    ss << this->now().nanoseconds() << basename;
    filename = path + ss.str();
    RCLCPP_INFO(this->get_logger(), "Video recording to %s", filename.c_str());
  }

  if (codec.size() != 4) {
    RCLCPP_ERROR(this->get_logger(), "The video codec must be a FOURCC identifier (4 chars)");
    rclcpp::shutdown();
  }

  auto topic = rclcpp::expand_topic_or_service_name(
    "image", this->get_name(), this->get_namespace());
  sub_image = image_transport::create_subscription(
    this, topic, std::bind(&VideoRecorderNode::callback, this, std::placeholders::_1), "raw");

  RCLCPP_INFO(this->get_logger(), "Waiting for topic %s...", topic.c_str());
}

VideoRecorderNode::~VideoRecorderNode()
{
  if (recording_started) {
    std::cout << "\nVideo saved as: " << filename << std::endl;
  }
}

void VideoRecorderNode::callback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  if (!outputVideo.isOpened()) {
    cv::Size size(image_msg->width, image_msg->height);

    outputVideo.open(
      filename,
      cv::VideoWriter::fourcc(
        codec.c_str()[0],
        codec.c_str()[1],
        codec.c_str()[2],
        codec.c_str()[3]),
      fps,
      size,
      true);

    if (!outputVideo.isOpened()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not create the output video! Check filename and/or support for codec.");
      rclcpp::shutdown();
    }

    recording_started = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Starting to record %s video at %ix%i@%i fps. Press Ctrl+C to stop recording.",
      codec.c_str(), size.height, size.width, fps);
  }

  if (
    (rclcpp::Time(image_msg->header.stamp, RCL_ROS_TIME) - g_last_wrote_time) <
    rclcpp::Duration::from_seconds(1.0 / fps))
  {
    // Skip to get video with correct fps
    return;
  }

  try {
    cv_bridge::CvtColorForDisplayOptions options;
    options.do_dynamic_scaling = use_dynamic_range;
    options.min_image_value = min_depth_range;
    options.max_image_value = max_depth_range;
    options.colormap = colormap;

    const cv::Mat image =
      cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg), encoding, options)->image;

    if (!image.empty()) {
      outputVideo << image;
      RCLCPP_INFO(this->get_logger(), "Recording frame %i\x1b[1F", g_count);
      g_count++;
      g_last_wrote_time = rclcpp::Time(image_msg->header.stamp, RCL_ROS_TIME);
    } else {
      RCLCPP_WARN(this->get_logger(), "Frame skipped, no data!");
    }
  } catch (const cv_bridge::Exception &) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Unable to convert %s image to %s",
      image_msg->encoding.c_str(), encoding.c_str());
    return;
  }
}

}  // namespace image_view

RCLCPP_COMPONENTS_REGISTER_NODE(image_view::VideoRecorderNode)
