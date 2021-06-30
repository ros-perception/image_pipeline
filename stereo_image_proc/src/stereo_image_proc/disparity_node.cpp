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
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
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
#include <map>
#include <memory>
#include <string>
#include <sstream>
#include <utility>
#include <vector>

namespace stereo_image_proc
{

class DisparityNode : public rclcpp::Node
{
public:
  explicit DisparityNode(const rclcpp::NodeOptions & options);

private:
  enum StereoAlgorithm
  {
    BLOCK_MATCHING = 0,
    SEMI_GLOBAL_BLOCK_MATCHING
  };

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
  std::shared_ptr<rclcpp::Publisher<stereo_msgs::msg::DisparityImage>> pub_disparity_;

  // Handle to parameters callback
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

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

  rcl_interfaces::msg::SetParametersResult parameterSetCb(
    const std::vector<rclcpp::Parameter> & parameters);
};

// Some helper functions for adding a parameter to a collection
static void add_param_to_map(
  std::map<std::string, std::pair<int, rcl_interfaces::msg::ParameterDescriptor>> & parameters,
  const std::string & name,
  const std::string & description,
  const int default_value,
  const int from_value,
  const int to_value,
  const int step)
{
  rcl_interfaces::msg::IntegerRange integer_range;
  integer_range.from_value = from_value;
  integer_range.to_value = to_value;
  integer_range.step = step;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.integer_range = {integer_range};
  parameters[name] = std::make_pair(default_value, descriptor);
}

static void add_param_to_map(
  std::map<std::string, std::pair<double, rcl_interfaces::msg::ParameterDescriptor>> & parameters,
  const std::string & name,
  const std::string & description,
  const double default_value,
  const double from_value,
  const double to_value,
  const double step)
{
  rcl_interfaces::msg::FloatingPointRange floating_point_range;
  floating_point_range.from_value = from_value;
  floating_point_range.to_value = to_value;
  floating_point_range.step = step;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.floating_point_range = {floating_point_range};
  parameters[name] = std::make_pair(default_value, descriptor);
}

DisparityNode::DisparityNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("disparity_node", options)
{
  using namespace std::placeholders;

  // Declare/read parameters
  int queue_size = this->declare_parameter("queue_size", 5);
  bool approx = this->declare_parameter("approximate_sync", false);
  this->declare_parameter("use_system_default_qos", false);

  // Synchronize callbacks
  if (approx) {
    approximate_sync_.reset(
      new ApproximateSync(
        ApproximatePolicy(queue_size),
        sub_l_image_, sub_l_info_,
        sub_r_image_, sub_r_info_));
    approximate_sync_->registerCallback(
      std::bind(&DisparityNode::imageCb, this, _1, _2, _3, _4));
  } else {
    exact_sync_.reset(
      new ExactSync(
        ExactPolicy(queue_size),
        sub_l_image_, sub_l_info_,
        sub_r_image_, sub_r_info_));
    exact_sync_->registerCallback(
      std::bind(&DisparityNode::imageCb, this, _1, _2, _3, _4));
  }

  // Register a callback for when parameters are set
  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DisparityNode::parameterSetCb, this, _1));

  // Describe int parameters
  std::map<std::string, std::pair<int, rcl_interfaces::msg::ParameterDescriptor>> int_params;
  add_param_to_map(
    int_params,
    "stereo_algorithm",
    "Stereo algorithm: Block Matching (0) or Semi-Global Block Matching (1)",
    0, 0, 1, 1);  // default, from, to, step
  add_param_to_map(
    int_params,
    "prefilter_size",
    "Normalization window size in pixels (must be odd)",
    9, 5, 255, 2);
  add_param_to_map(
    int_params,
    "prefilter_cap",
    "Bound on normalized pixel values",
    31, 1, 63, 1);
  add_param_to_map(
    int_params,
    "correlation_window_size",
    "SAD correlation window width in pixels (must be odd)",
    15, 5, 255, 2);
  add_param_to_map(
    int_params,
    "min_disparity",
    "Disparity to begin search at in pixels",
    0, -2048, 2048, 1);
  add_param_to_map(
    int_params,
    "disparity_range",
    "Number of disparities to search in pixels (must be a multiple of 16)",
    64, 32, 4096, 16);
  add_param_to_map(
    int_params,
    "texture_threshold",
    "Filter out if SAD window response does not exceed texture threshold",
    10, 0, 10000, 1);
  add_param_to_map(
    int_params,
    "speckle_size",
    "Reject regions smaller than this size in pixels",
    100, 0, 1000, 1);
  add_param_to_map(
    int_params,
    "speckle_range",
    "Maximum allowed difference between detected disparities",
    4, 0, 31, 1);
  add_param_to_map(
    int_params,
    "disp12_max_diff",
    "Maximum allowed difference in the left-right disparity check in pixels"
    " (Semi-Global Block Matching only)",
    0, 0, 128, 1);

  // Describe double parameters
  std::map<std::string, std::pair<double, rcl_interfaces::msg::ParameterDescriptor>> double_params;
  add_param_to_map(
    double_params,
    "uniqueness_ratio",
    "Filter out if best match does not sufficiently exceed the next-best match",
    15.0, 0.0, 100.0, 0.0);
  add_param_to_map(
    double_params,
    "P1",
    "The first parameter ccontrolling the disparity smoothness (Semi-Global Block Matching only)",
    200.0, 0.0, 4000.0, 0.0);
  add_param_to_map(
    double_params,
    "P2",
    "The second parameter ccontrolling the disparity smoothness (Semi-Global Block Matching only)",
    400.0, 0.0, 4000.0, 0.0);

  // Describe bool parameters
  std::map<std::string, std::pair<bool, rcl_interfaces::msg::ParameterDescriptor>> bool_params;
  rcl_interfaces::msg::ParameterDescriptor full_dp_descriptor;
  full_dp_descriptor.description =
    "Run the full variant of the algorithm (Semi-Global Block Matching only)";
  bool_params["full_dp"] = std::make_pair(false, full_dp_descriptor);

  // Declaring parameters triggers the previously registered callback
  this->declare_parameters("", int_params);
  this->declare_parameters("", double_params);
  this->declare_parameters("", bool_params);

  pub_disparity_ = create_publisher<stereo_msgs::msg::DisparityImage>("disparity", 1);

  // TODO(jacobperron): Replace this with a graph event.
  //                    Only subscribe if there's a subscription listening to our publisher.
  connectCb();
}

// Handles (un)subscribing when clients (un)subscribe
void DisparityNode::connectCb()
{
  // TODO(jacobperron): Add unsubscribe logic when we use graph events
  image_transport::TransportHints hints(this, "raw");
  const bool use_system_default_qos = this->get_parameter("use_system_default_qos").as_bool();
  rclcpp::QoS image_sub_qos = rclcpp::SensorDataQoS();
  if (use_system_default_qos) {
    image_sub_qos = rclcpp::SystemDefaultsQoS();
  }
  const auto image_sub_rmw_qos = image_sub_qos.get_rmw_qos_profile();
  sub_l_image_.subscribe(this, "left/image_rect", hints.getTransport(), image_sub_rmw_qos);
  sub_l_info_.subscribe(this, "left/camera_info", image_sub_rmw_qos);
  sub_r_image_.subscribe(this, "right/image_rect", hints.getTransport(), image_sub_rmw_qos);
  sub_r_info_.subscribe(this, "right/camera_info", image_sub_rmw_qos);
}

void DisparityNode::imageCb(
  const sensor_msgs::msg::Image::ConstSharedPtr & l_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & l_info_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & r_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & r_info_msg)
{
  // If there are no subscriptions for the disparity image, do nothing
  if (pub_disparity_->get_subscription_count() == 0u) {
    return;
  }

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

rcl_interfaces::msg::SetParametersResult DisparityNode::parameterSetCb(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters) {
    const std::string param_name = param.get_name();
    if ("stereo_algorithm" == param_name) {
      const int stereo_algorithm_value = param.as_int();
      if (BLOCK_MATCHING == stereo_algorithm_value) {
        block_matcher_.setStereoType(StereoProcessor::BM);
      } else if (SEMI_GLOBAL_BLOCK_MATCHING == stereo_algorithm_value) {
        block_matcher_.setStereoType(StereoProcessor::SGBM);
      } else {
        result.successful = false;
        std::ostringstream oss;
        oss << "Unknown stereo algorithm type '" << stereo_algorithm_value << "'";
        result.reason = oss.str();
      }
    } else if ("prefilter_size" == param_name) {
      block_matcher_.setPreFilterSize(param.as_int());
    } else if ("prefilter_cap" == param_name) {
      block_matcher_.setPreFilterCap(param.as_int());
    } else if ("correlation_window_size" == param_name) {
      block_matcher_.setCorrelationWindowSize(param.as_int());
    } else if ("min_disparity" == param_name) {
      block_matcher_.setMinDisparity(param.as_int());
    } else if ("disparity_range" == param_name) {
      block_matcher_.setDisparityRange(param.as_int());
    } else if ("uniqueness_ratio" == param_name) {
      block_matcher_.setUniquenessRatio(param.as_double());
    } else if ("texture_threshold" == param_name) {
      block_matcher_.setTextureThreshold(param.as_int());
    } else if ("speckle_size" == param_name) {
      block_matcher_.setSpeckleSize(param.as_int());
    } else if ("speckle_range" == param_name) {
      block_matcher_.setSpeckleRange(param.as_int());
    } else if ("full_dp" == param_name) {
      block_matcher_.setSgbmMode(param.as_bool());
    } else if ("P1" == param_name) {
      block_matcher_.setP1(param.as_double());
    } else if ("P2" == param_name) {
      block_matcher_.setP2(param.as_double());
    } else if ("disp12_max_diff" == param_name) {
      block_matcher_.setDisp12MaxDiff(param.as_int());
    }
  }
  return result;
}

}  // namespace stereo_image_proc

// Register component
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_image_proc::DisparityNode)
