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

#include "image_view/stereo_view_node.hpp"

#include <boost/format.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace image_view
{

namespace enc = sensor_msgs::image_encodings;

using sensor_msgs::msg::Image;
using stereo_msgs::msg::DisparityImage;
using message_filters::sync_policies::ExactTime;
using message_filters::sync_policies::ApproximateTime;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

constexpr unsigned char StereoViewNode::colormap[768];

StereoViewNode::StereoViewNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("stereo_view_node", options),
  filename_format_(""), save_count_(0),
  left_received_(0), right_received_(0), disp_received_(0), all_received_(0)
{
  // Read local parameters
  bool autosize = this->declare_parameter("autosize", true);

  std::string format_string;
  format_string = this->declare_parameter("filename_format", std::string("%s%04i.jpg"));
  filename_format_.parse(format_string);

  std::string transport = this->declare_parameter("transport", std::string("raw"));

  // Do GUI window setup
  int flags = autosize ? (cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED) : 0;
  cv::namedWindow("left", flags);
  cv::namedWindow("right", flags);
  cv::namedWindow("disparity", flags);
  cv::setMouseCallback("left", &StereoViewNode::mouseCb, this);
  cv::setMouseCallback("right", &StereoViewNode::mouseCb, this);
  cv::setMouseCallback("disparity", &StereoViewNode::mouseCb, this);

  // Resolve topic names
  std::string stereo_ns = rclcpp::expand_topic_or_service_name(
    "stereo", this->get_name(), this->get_namespace());
  std::string left_topic = rclcpp::expand_topic_or_service_name(
    stereo_ns + "/left/" + rclcpp::expand_topic_or_service_name(
      "image", this->get_name(), this->get_namespace()),
    this->get_name(), this->get_namespace());
  std::string right_topic = rclcpp::expand_topic_or_service_name(
    stereo_ns + "/right/" + rclcpp::expand_topic_or_service_name(
      "image", this->get_name(), this->get_namespace()),
    this->get_name(), this->get_namespace());
  std::string disparity_topic = rclcpp::expand_topic_or_service_name(
    stereo_ns + "/disparity", this->get_name(), this->get_namespace());
  RCLCPP_INFO(
    this->get_logger(),
    "Subscribing to:\n\t* %s\n\t* %s\n\t* %s",
    left_topic.c_str(), right_topic.c_str(),
    disparity_topic.c_str());

  // Subscribe to three input topics.
  left_sub_.subscribe(this, left_topic, transport);
  right_sub_.subscribe(this, right_topic, transport);
  disparity_sub_.subscribe(this, disparity_topic);

  // Complain every 30s if the topics appear unsynchronized
  left_sub_.registerCallback(std::bind(increment, &left_received_));
  right_sub_.registerCallback(std::bind(increment, &right_received_));
  disparity_sub_.registerCallback(std::bind(increment, &disp_received_));
  check_synced_timer_ = this->create_wall_timer(
    std::chrono::seconds(15),
    std::bind(&StereoViewNode::checkInputsSynchronized, this));

  // Synchronize input topics. Optionally do approximate synchronization.
  queue_size_ = this->declare_parameter("queue_size", 5);
  bool approx = this->declare_parameter("approximate_sync", false);

  if (approx) {
    approximate_sync_.reset(
      new ApproximateSync(
        ApproximatePolicy(queue_size_), left_sub_, right_sub_, disparity_sub_));
    approximate_sync_->registerCallback(
      std::bind(&StereoViewNode::imageCb, this, _1, _2, _3));
  } else {
    exact_sync_.reset(
      new ExactSync(
        ExactPolicy(queue_size_),
        left_sub_, right_sub_, disparity_sub_));
    exact_sync_->registerCallback(
      std::bind(&StereoViewNode::imageCb, this, _1, _2, _3));
  }

  std::string stereo_topic =
    rclcpp::expand_topic_or_service_name("stereo", this->get_name(), this->get_namespace());
  std::string image_topic =
    rclcpp::expand_topic_or_service_name("image", this->get_name(), this->get_namespace());
  auto topics = this->get_topic_names_and_types();

  if (topics.find(stereo_topic) != topics.end()) {
    RCLCPP_WARN(
      this->get_logger(), "'stereo' has not been remapped! Example command-line usage:\n"
      "\t$ rosrun image_view stereo_view stereo:=narrow_stereo image:=image_color");
  }

  if (topics.find(image_topic) != topics.end()) {
    RCLCPP_WARN(
      this->get_logger(), "There is a delay between when the camera drivers publish "
      "the raw images and when stereo_image_proc publishes the computed point cloud. "
      "stereo_view may fail to synchronize these topics without a large queue_size.");
  }
}

StereoViewNode::~StereoViewNode()
{
  cv::destroyAllWindows();
}

void StereoViewNode::imageCb(
  const Image::ConstSharedPtr & left, const Image::ConstSharedPtr & right,
  const DisparityImage::ConstSharedPtr & disparity_msg)
{
  ++all_received_;  // For error checking

  image_mutex_.lock();

  // May want to view raw bayer data
  if (left->encoding.find("bayer") != std::string::npos) {
    std::const_pointer_cast<Image>(left)->encoding = "mono8";
  }

  if (right->encoding.find("bayer") != std::string::npos) {
    std::const_pointer_cast<Image>(right)->encoding = "mono8";
  }

  // Hang on to image data for sake of mouseCb
  last_left_msg_ = left;
  last_right_msg_ = right;

  try {
    last_left_image_ = cv_bridge::toCvShare(left, "bgr8")->image;
    last_right_image_ = cv_bridge::toCvShare(right, "bgr8")->image;
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Unable to convert one of '%s' or '%s' to 'bgr8'",
      left->encoding.c_str(), right->encoding.c_str());
  }

  // Colormap and display the disparity image
  float min_disparity = disparity_msg->min_disparity;
  float max_disparity = disparity_msg->max_disparity;
  float multiplier = 255.0f / (max_disparity - min_disparity);

  assert(disparity_msg->image.encoding == enc::TYPE_32FC1);
  const cv::Mat_<float> dmat(
    disparity_msg->image.height,
    disparity_msg->image.width,
    const_cast<float *>(reinterpret_cast<const float *>(&disparity_msg->image.data[0])),
    disparity_msg->image.step);
  disparity_color_.create(disparity_msg->image.height, disparity_msg->image.width);

  for (int row = 0; row < disparity_color_.rows; ++row) {
    const float * d = dmat[row];

    for (int col = 0; col < disparity_color_.cols; ++col) {
      int index = (d[col] - min_disparity) * multiplier + 0.5;
      index = std::min(255, std::max(0, index));
      // Fill as BGR
      disparity_color_(row, col)[2] = colormap[3 * index + 0];
      disparity_color_(row, col)[1] = colormap[3 * index + 1];
      disparity_color_(row, col)[0] = colormap[3 * index + 2];
    }
  }

  // Must release the mutex before calling cv::imshow, or can deadlock against
  // OpenCV's window mutex.
  image_mutex_.unlock();

  if (!last_left_image_.empty()) {
    cv::imshow("left", last_left_image_);
    cv::waitKey(1);
  }

  if (!last_right_image_.empty()) {
    cv::imshow("right", last_right_image_);
    cv::waitKey(1);
  }

  cv::imshow("disparity", disparity_color_);
  cv::waitKey(1);
}

void StereoViewNode::saveImage(const char * prefix, const cv::Mat & image)
{
  if (!image.empty()) {
    std::string filename = (filename_format_ % prefix % save_count_).str();
    cv::imwrite(filename, image);
    RCLCPP_INFO(this->get_logger(), "Saved image %s", filename.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "Couldn't save %s image, no data!", prefix);
  }
}

void StereoViewNode::mouseCb(int event, int x, int y, int flags, void * param)
{
  (void)x;
  (void)y;
  (void)flags;

  StereoViewNode * this_ = reinterpret_cast<StereoViewNode *>(param);

  if (event == cv::EVENT_LBUTTONDOWN) {
    RCLCPP_WARN_ONCE(
      this_->get_logger(),
      "Left-clicking no longer saves images. Right-click instead.");
    return;
  }

  if (event != cv::EVENT_RBUTTONDOWN) {
    return;
  }

  std::lock_guard<std::mutex> guard(this_->image_mutex_);

  this_->saveImage("left", this_->last_left_image_);
  this_->saveImage("right", this_->last_right_image_);
  this_->saveImage("disp", this_->disparity_color_);
  this_->save_count_++;
}

void StereoViewNode::checkInputsSynchronized()
{
  int threshold = 3 * all_received_;
  if (left_received_ >= threshold || right_received_ >= threshold || disp_received_ >= threshold) {
    RCLCPP_WARN(
      this->get_logger(),
      "[stereo_view] Low number of synchronized left/right/disparity triplets received.\n"
      "Left images received:      %d (topic '%s')\n"
      "Right images received:     %d (topic '%s')\n"
      "Disparity images received: %d (topic '%s')\n"
      "Synchronized triplets: %d\n"
      "Possible issues:\n"
      "\t* stereo_image_proc is not running.\n"
      "\t  Does `ros2 node info %s` show any connections?\n"
      "\t* The cameras are not synchronized.\n"
      "\t  Try restarting stereo_view with parameter _approximate_sync:=True\n"
      "\t* The network is too slow. One or more images are dropped from each triplet.\n"
      "\t  Try restarting stereo_view, increasing parameter 'queue_size' (currently %d)",
      left_received_, left_sub_.getTopic().c_str(),
      right_received_, right_sub_.getTopic().c_str(),
      disp_received_, disparity_sub_.getTopic().c_str(),
      all_received_, this->get_name(), queue_size_);
  }
}

}  // namespace image_view

RCLCPP_COMPONENTS_REGISTER_NODE(image_view::StereoViewNode)
