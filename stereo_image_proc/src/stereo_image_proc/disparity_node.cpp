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
//  * Neither the name of the copyright holder nor the names of its
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
#include <stereo_image_proc/stereo_processor.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include <opencv2/calib3d/calib3d.hpp>

#include <algorithm>
#include <memory>

namespace stereo_image_proc
{

class DisparityNode : public rclcpp::Node
{
public:
  explicit DisparityNode(const rclcpp::NodeOptions & options);

private:
  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_l_info_, sub_r_info_;
  using ExactPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using ExactSync = message_filters::Synchronizer<ExactPolicy>;
  using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
  std::shared_ptr<ExactSync> exact_sync_;
  std::shared_ptr<ApproximateSync> approximate_sync_;
  // Publications
  // TODO(jacobperron): Uncomment when we can be notified of subscriber status
  // std::mutex connect_mutex_;
  std::shared_ptr<rclcpp::Publisher<stereo_msgs::msg::DisparityImage>> pub_disparity_;

  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;
  // contains scratch buffers for block matching
  stereo_image_proc::StereoProcessor block_matcher_;

  void connectCb();

  void imageCb(
    const sensor_msgs::msg::Image::ConstSharedPtr & l_image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & l_info_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & r_image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & r_info_msg);

  // void configCb(Config &config, uint32_t level);
};

DisparityNode::DisparityNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("disparity_node", options)
{
  using namespace std::placeholders;

  // Declare/read parameters
  int queue_size = this->declare_parameter("queue_size", 5);
  bool approx = this->declare_parameter("approximate_sync", false);

  // Synchronize callbacks
  if (approx) {
    approximate_sync_.reset(new ApproximateSync(
        ApproximatePolicy(queue_size),
        sub_l_image_, sub_l_info_,
        sub_r_image_, sub_r_info_));
    approximate_sync_->registerCallback(
      std::bind(&DisparityNode::imageCb, this, _1, _2, _3, _4));
  } else {
    exact_sync_.reset(new ExactSync(
        ExactPolicy(queue_size),
        sub_l_image_, sub_l_info_,
        sub_r_image_, sub_r_info_));
    exact_sync_->registerCallback(
      std::bind(&DisparityNode::imageCb, this, _1, _2, _3, _4));
  }

  // TODO(jacobperron): Setup equivalent of dynamic reconfigure with ROS 2 parameters
  //                    See configCb

  // TODO(jacobperron): Monitoring subscriber status is currently not possible in ROS 2
  // std::lock_guard<std::mutex> lock(connect_mutex_);

  // Monitor whether anyone is subscribed to the output
  // ros::SubscriberStatusCallback connect_cb = std::bind(&DisparityNode::connectCb, this);

  pub_disparity_ = create_publisher<stereo_msgs::msg::DisparityImage>("disparity", 1);

  // TODO(jacobperron): Remove when we can be notified of subscriber status
  connectCb();
}

// Handles (un)subscribing when clients (un)subscribe
void DisparityNode::connectCb()
{
  // TODO(jacobperron): Uncomment when we can be notified of subscriber status
  // std::lock_guard<std::mutex> lock(connect_mutex_);
  // if (pub_disparity_.getNumSubscribers() == 0)
  // {
  //   sub_l_image_.unsubscribe();
  //   sub_l_info_ .unsubscribe();
  //   sub_r_image_.unsubscribe();
  //   sub_r_info_ .unsubscribe();
  // }
  // else if (!sub_l_image_.getSubscriber())
  if (!sub_l_image_.getSubscriber()) {
    image_transport::TransportHints hints(this, "raw");
    sub_l_image_.subscribe(this, "left/image_rect", hints.getTransport());
    sub_l_info_.subscribe(this, "left/camera_info");
    sub_r_image_.subscribe(this, "right/image_rect", hints.getTransport());
    sub_r_info_.subscribe(this, "right/camera_info");
  }
}

void DisparityNode::imageCb(
  const sensor_msgs::msg::Image::ConstSharedPtr & l_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & l_info_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & r_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & r_info_msg)
{
  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);

  // Allocate new disparity image message
  auto disp_msg = std::make_shared<stereo_msgs::msg::DisparityImage>();
  disp_msg->header = l_info_msg->header;
  disp_msg->image.header = l_info_msg->header;

  // Compute window of (potentially) valid disparities
  int border = block_matcher_.getCorrelationWindowSize() / 2;
  int left = block_matcher_.getDisparityRange() + block_matcher_.getMinDisparity() + border - 1;
  int wtf;
  if (block_matcher_.getMinDisparity() >= 0) {
    wtf = border + block_matcher_.getMinDisparity();
  } else {
    wtf = std::max(border, -block_matcher_.getMinDisparity());
  }
  // TODO(jacobperron): the message width has not been set yet! What should we do here?
  int right = disp_msg->image.width - 1 - wtf;
  int top = border;
  int bottom = disp_msg->image.height - 1 - border;
  disp_msg->valid_window.x_offset = left;
  disp_msg->valid_window.y_offset = top;
  disp_msg->valid_window.width = right - left;
  disp_msg->valid_window.height = bottom - top;

  // Create cv::Mat views onto all buffers
  const cv::Mat_<uint8_t> l_image =
    cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)->image;
  const cv::Mat_<uint8_t> r_image =
    cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)->image;

  // Perform block matching to find the disparities
  block_matcher_.processDisparity(l_image, r_image, model_, *disp_msg);

  pub_disparity_->publish(*disp_msg);
}

// void DisparityNode::configCb(Config &config, uint32_t level)
// {
//   // Tweak all settings to be valid
//   config.prefilter_size |= 0x1; // must be odd
//   config.correlation_window_size |= 0x1; // must be odd
//   config.disparity_range = (config.disparity_range / 16) * 16; // must be multiple of 16
//
//   // check stereo method
//   // Note: With single-threaded NodeHandle, configCb and imageCb can't be called
//   // concurrently, so this is thread-safe.
//   block_matcher_.setPreFilterCap(config.prefilter_cap);
//   block_matcher_.setCorrelationWindowSize(config.correlation_window_size);
//   block_matcher_.setMinDisparity(config.min_disparity);
//   block_matcher_.setDisparityRange(config.disparity_range);
//   block_matcher_.setUniquenessRatio(config.uniqueness_ratio);
//   block_matcher_.setSpeckleSize(config.speckle_size);
//   block_matcher_.setSpeckleRange(config.speckle_range);
//   if (config.stereo_algorithm == stereo_image_proc::Disparity_StereoBM) { // StereoBM
//     block_matcher_.setStereoType(StereoProcessor::BM);
//     block_matcher_.setPreFilterSize(config.prefilter_size);
//     block_matcher_.setTextureThreshold(config.texture_threshold);
//   }
//   else if (config.stereo_algorithm == stereo_image_proc::Disparity_StereoSGBM) { // StereoSGBM
//     block_matcher_.setStereoType(StereoProcessor::SGBM);
//     block_matcher_.setSgbmMode(config.fullDP);
//     block_matcher_.setP1(config.P1);
//     block_matcher_.setP2(config.P2);
//     block_matcher_.setDisp12MaxDiff(config.disp12MaxDiff);
//   }
// }

}  // namespace stereo_image_proc

// Register component
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_image_proc::DisparityNode)
