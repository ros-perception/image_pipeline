// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "cv_bridge/cv_bridge.hpp"

#include <depth_image_proc/conversions.hpp>
#include <depth_image_proc/point_cloud_xyzrgb.hpp>
#include <image_transport/camera_common.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace depth_image_proc
{

PointCloudXyzrgbNode::PointCloudXyzrgbNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("PointCloudXyzrgbNode", options)
{
  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("image_transport", "raw");
  this->declare_parameter<std::string>("depth_image_transport", "raw");

  // value used for invalid points for pcd conversion
  invalid_depth_ = this->declare_parameter<double>("invalid_depth", 0.0);

  // Upper bound on depth image pixel count; guards against oversized
  // width/height that would overflow internal size computations.
  // 100 MP is well above any realistic sensor.
  max_pixels_ = this->declare_parameter<int64_t>("max_pixels", 100LL * 1000LL * 1000LL);

  // Read parameters
  int queue_size = this->declare_parameter<int>("queue_size", 5);
  bool use_exact_sync = this->declare_parameter<bool>("exact_sync", false);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  if (use_exact_sync) {
    exact_sync_ = std::make_shared<ExactSynchronizer>(
      ExactSyncPolicy(queue_size),
      sub_depth_,
      sub_rgb_,
      sub_info_);
    exact_sync_->registerCallback(
      std::bind(
        &PointCloudXyzrgbNode::imageCb,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));
  } else {
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_);
    sync_->registerCallback(
      std::bind(
        &PointCloudXyzrgbNode::imageCb,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));
  }

  // Create publisher with connect callback
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.matched_callback =
    [this](rclcpp::MatchedInfo & s)
    {
      std::lock_guard<std::mutex> lock(connect_mutex_);
      if (s.current_count == 0) {
        sub_depth_.unsubscribe();
        sub_rgb_.unsubscribe();
        sub_info_.unsubscribe();
      } else if (!sub_depth_.getSubscriber()) {
        // For compressed topics to remap appropriately, we need to pass a
        // fully expanded and remapped topic name to image_transport
        auto node_base = this->get_node_base_interface();
        std::string depth_topic =
          node_base->resolve_topic_or_service_name("depth_registered/image_rect", false);
        std::string rgb_topic =
          node_base->resolve_topic_or_service_name("rgb/image_rect_color", false);
        // Allow also remapping camera_info to something different than default
        std::string rgb_info_topic =
          node_base->resolve_topic_or_service_name(
          image_transport::getCameraInfoTopic(rgb_topic), false);

        // parameter for depth_image_transport hint
        image_transport::TransportHints depth_hints(*this,
          "raw", "depth_image_transport");

        rclcpp::SubscriptionOptions sub_opts;
        // Update the subscription options to allow reconfigurable qos settings.
        sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions {
          {
            // Here all policies that are desired to be reconfigurable are listed.
            rclcpp::QosPolicyKind::Depth,
            rclcpp::QosPolicyKind::Durability,
            rclcpp::QosPolicyKind::History,
            rclcpp::QosPolicyKind::Reliability,
          }};

        // depth image can use different transport.(e.g. compressedDepth)
        sub_depth_.subscribe(
          *this, depth_topic,
          depth_hints.getTransport(), rclcpp::SystemDefaultsQoS(), sub_opts);

        // rgb uses normal ros transport hints.
        image_transport::TransportHints hints{*this};
        sub_rgb_.subscribe(
          *this,
          rgb_topic,
          hints.getTransport(),
          rclcpp::SystemDefaultsQoS(), sub_opts);
        sub_info_.subscribe(this, rgb_info_topic, rclcpp::QoS(10));
      }
    };
  // Allow overriding QoS settings (history, depth, reliability)
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pub_point_cloud_ = create_publisher<PointCloud2>("points", rclcpp::SystemDefaultsQoS(),
      pub_options);
}

PointCloudXyzrgbNode::~PointCloudXyzrgbNode()
{
  // Disconnect inputs so the synchronizer stops receiving new messages.
  // This must happen before the implicit member teardown, otherwise a
  // late imageCb() on another executor thread could touch members
  // (e.g. pub_point_cloud_) that have already been destroyed.
  {
    std::lock_guard<std::mutex> lock(connect_mutex_);
    sub_depth_.unsubscribe();
    sub_rgb_.unsubscribe();
    sub_info_.unsubscribe();
  }
  // Drop the synchronizers so any pending dispatch can no longer call
  // back into this object.
  sync_.reset();
  exact_sync_.reset();
  // Wait for any imageCb() that started before the unsubscribe above
  // to finish before we let members be destroyed.
  std::lock_guard<std::mutex> lock(callback_mutex_);
}

void PointCloudXyzrgbNode::imageCb(
  const Image::ConstSharedPtr & depth_msg,
  const Image::ConstSharedPtr & rgb_msg_in,
  const CameraInfo::ConstSharedPtr & info_msg)
{
  // Hold callback_mutex_ for the lifetime of this callback so the
  // destructor can wait for an in-flight callback to complete before
  // member teardown begins.
  std::lock_guard<std::mutex> lock(callback_mutex_);
  if (!pub_point_cloud_) {
    return;
  }
  // Check for bad inputs
  if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      10000,  // 10 seconds
      "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
      depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
  }

  // Reject unreasonable or inconsistent depth dimensions before doing any
  // work that depends on the buffer size. Without this guard, oversized
  // width/height (or a payload smaller than declared) lets convertDepth()
  // iterate past the end of depth_msg->data and read out of bounds.
  const uint64_t depth_pixels =
    static_cast<uint64_t>(depth_msg->height) * static_cast<uint64_t>(depth_msg->width);
  if (depth_pixels == 0 || depth_pixels > static_cast<uint64_t>(max_pixels_)) {
    RCLCPP_ERROR(
      get_logger(),
      "Depth image dimensions are unreasonable (width=%u, height=%u); skipping.",
      depth_msg->width, depth_msg->height);
    return;
  }

  size_t depth_bytes_per_pixel = 0;
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    depth_bytes_per_pixel = sizeof(uint16_t);
  } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    depth_bytes_per_pixel = sizeof(float);
  } else {
    RCLCPP_ERROR(
      get_logger(), "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  const size_t expected_depth_bytes =
    static_cast<size_t>(depth_pixels) * depth_bytes_per_pixel;
  if (depth_msg->data.size() < expected_depth_bytes ||
    depth_msg->step < static_cast<uint32_t>(depth_msg->width * depth_bytes_per_pixel))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Depth image buffer is inconsistent with declared dimensions "
      "(width=%u, height=%u, step=%u, data size=%zu, expected >=%zu); skipping.",
      depth_msg->width, depth_msg->height, depth_msg->step,
      depth_msg->data.size(), expected_depth_bytes);
    return;
  }

  // Update camera model
  model_.fromCameraInfo(info_msg);

  // Check if the input image has to be resized
  Image::ConstSharedPtr rgb_msg = rgb_msg_in;
  if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height) {
    if (depth_msg->width == 0 || rgb_msg->width == 0 ||
      depth_msg->height == 0 || rgb_msg->height == 0)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid image dimensions: depth (%ux%u), rgb (%ux%u)",
        depth_msg->width, depth_msg->height, rgb_msg->width, rgb_msg->height);
      return;
    }

    // Validate that a width-based scale also yields a valid in-bounds row crop
    // on the RGB image. Otherwise rowRange(...) below would throw cv::Exception
    // and abort the process. Use integer math equivalent of
    // (depth_height / ratio) where ratio = depth_width / rgb_width.
    const uint64_t crop_rows =
      (static_cast<uint64_t>(depth_msg->height) *
      static_cast<uint64_t>(rgb_msg->width)) /
      static_cast<uint64_t>(depth_msg->width);
    if (crop_rows == 0 || crop_rows > static_cast<uint64_t>(rgb_msg->height)) {
      RCLCPP_ERROR(
        get_logger(),
        "Depth (%ux%u) and RGB (%ux%u) have incompatible aspect ratios; "
        "cannot derive a valid resize crop. Skipping frame.",
        depth_msg->width, depth_msg->height, rgb_msg->width, rgb_msg->height);
      return;
    }

    CameraInfo info_msg_tmp = *info_msg;
    info_msg_tmp.width = depth_msg->width;
    info_msg_tmp.height = depth_msg->height;
    float ratio = static_cast<float>(depth_msg->width) / static_cast<float>(rgb_msg->width);
    info_msg_tmp.k[0] *= ratio;
    info_msg_tmp.k[2] *= ratio;
    info_msg_tmp.k[4] *= ratio;
    info_msg_tmp.k[5] *= ratio;
    info_msg_tmp.p[0] *= ratio;
    info_msg_tmp.p[2] *= ratio;
    info_msg_tmp.p[5] *= ratio;
    info_msg_tmp.p[6] *= ratio;
    model_.fromCameraInfo(info_msg_tmp);

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv_bridge::CvImage cv_rsz;
    cv_rsz.header = cv_ptr->header;
    cv_rsz.encoding = cv_ptr->encoding;
    try {
      cv::resize(
        cv_ptr->image.rowRange(0, static_cast<int>(crop_rows)), cv_rsz.image,
        cv::Size(depth_msg->width, depth_msg->height));
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR(get_logger(), "OpenCV exception while resizing RGB: %s", e.what());
      return;
    }
    if ((rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) ||
      (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) ||
      (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8))
    {
      rgb_msg = cv_rsz.toImageMsg();
    } else {
      rgb_msg =
        cv_bridge::toCvCopy(cv_rsz.toImageMsg(), sensor_msgs::image_encodings::RGB8)->toImageMsg();
    }

    RCLCPP_ERROR(
      get_logger(), "Depth resolution (%ux%u) does not match RGB resolution (%ux%u)",
      depth_msg->width, depth_msg->height, rgb_msg->width, rgb_msg->height);
    return;
  } else {
    rgb_msg = rgb_msg_in;
  }

  // Supported color encodings: RGB8, BGR8, MONO8
  int red_offset, green_offset, blue_offset, color_step;
  if (rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) {
    red_offset = 0;
    green_offset = 1;
    blue_offset = 2;
    color_step = 3;
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::RGBA8) {
    red_offset = 0;
    green_offset = 1;
    blue_offset = 2;
    color_step = 4;
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) {
    red_offset = 2;
    green_offset = 1;
    blue_offset = 0;
    color_step = 3;
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGRA8) {
    red_offset = 2;
    green_offset = 1;
    blue_offset = 0;
    color_step = 4;
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8) {
    red_offset = 0;
    green_offset = 0;
    blue_offset = 0;
    color_step = 1;
  } else {
    try {
      rgb_msg = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8)->toImageMsg();
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(
        get_logger(), "Unsupported encoding [%s]: %s", rgb_msg->encoding.c_str(), e.what());
      return;
    }
    red_offset = 0;
    green_offset = 1;
    blue_offset = 2;
    color_step = 3;
  }

  auto cloud_msg = std::make_unique<PointCloud2>();
  cloud_msg->header = depth_msg->header;  // Use depth image time stamp
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  // Cross-check that the modifier actually allocated enough storage for
  // height*width points. If not (e.g. arithmetic overflow), bail out
  // before convertDepth/convertRgb write past the cloud buffer.
  const size_t expected_cloud_bytes =
    static_cast<size_t>(depth_pixels) * cloud_msg->point_step;
  if (cloud_msg->data.size() < expected_cloud_bytes) {
    RCLCPP_ERROR(
      get_logger(),
      "PointCloud2 storage (%zu bytes) is smaller than required (%zu); skipping.",
      cloud_msg->data.size(), expected_cloud_bytes);
    return;
  }

  // Convert Depth Image to Pointcloud
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    convertDepth<uint16_t>(depth_msg, *cloud_msg, model_, invalid_depth_);
  } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    convertDepth<float>(depth_msg, *cloud_msg, model_, invalid_depth_);
  }

  // Convert RGB
  if (rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) {
    convertRgb(rgb_msg, *cloud_msg, red_offset, green_offset, blue_offset, color_step);
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) {
    convertRgb(rgb_msg, *cloud_msg, red_offset, green_offset, blue_offset, color_step);
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGRA8) {
    convertRgb(rgb_msg, *cloud_msg, red_offset, green_offset, blue_offset, color_step);
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::RGBA8) {
    convertRgb(rgb_msg, *cloud_msg, red_offset, green_offset, blue_offset, color_step);
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8) {
    convertRgb(rgb_msg, *cloud_msg, red_offset, green_offset, blue_offset, color_step);
  } else {
    RCLCPP_ERROR(
      get_logger(), "RGB image has unsupported encoding [%s]", rgb_msg->encoding.c_str());
    return;
  }

  pub_point_cloud_->publish(std::move(cloud_msg));
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::PointCloudXyzrgbNode)
