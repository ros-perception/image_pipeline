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
//  * Neither the name of the Willow Garage nor the names of its
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
#include "depth_image_proc/point_cloud_xyzi_radial.hpp"
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.hpp>
#include <depth_image_proc/conversions.hpp>
#include <depth_image_proc/visibility.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <memory>
#include <vector>
#include <string>
#include <limits>

namespace depth_image_proc
{


PointCloudXyziRadialNode::PointCloudXyziRadialNode(const rclcpp::NodeOptions & options)
: Node("PointCloudXyziRadialNode", options)
{
  // Read parameters
  queue_size_ = this->declare_parameter<int>("queue_size", 5);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  sync_ = std::make_shared<Synchronizer>(
    SyncPolicy(queue_size_),
    sub_depth_,
    sub_intensity_,
    sub_info_);
  sync_->registerCallback(
    std::bind(
      &PointCloudXyziRadialNode::imageCb,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3));

  // Monitor whether anyone is subscribed to the output
  // TODO(ros2) Implement when SubscriberStatusCallback is available
  // ros::SubscriberStatusCallback connect_cb =
  //   boost::bind(&PointCloudXyziRadialNode::connectCb, this);
  connectCb();

  // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
  std::lock_guard<std::mutex> lock(connect_mutex_);
  // TODO(ros2) Implement when SubscriberStatusCallback is available
  // pub_point_cloud_ = nh.advertise<PointCloud>("points", 20, connect_cb, connect_cb);
  pub_point_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "points", rclcpp::SensorDataQoS());
}

// Handles (un)subscribing when clients (un)subscribe
void PointCloudXyziRadialNode::connectCb()
{
  std::lock_guard<std::mutex> lock(connect_mutex_);

  // TODO(ros2) Implement getNumSubscribers when rcl/rmw support it
  // if (pub_point_cloud_.getNumSubscribers() == 0)
  if (0) {
    sub_depth_.unsubscribe();
    sub_intensity_.unsubscribe();
    sub_info_.unsubscribe();
  } else if (!sub_depth_.getSubscriber()) {
    // parameter for depth_image_transport hint
    std::string depth_image_transport_param = "depth_image_transport";

    // depth image can use different transport.(e.g. compressedDepth)
    image_transport::TransportHints depth_hints(this, "raw", depth_image_transport_param);
    sub_depth_.subscribe(this, "depth/image_raw", depth_hints.getTransport());

    // intensity uses normal ros transport hints.
    image_transport::TransportHints hints(this, "raw");
    sub_intensity_.subscribe(this, "intensity/image_raw", hints.getTransport());
    sub_info_.subscribe(this, "intensity/camera_info");
  }
}

void PointCloudXyziRadialNode::imageCb(
  const Image::ConstSharedPtr & depth_msg,
  const Image::ConstSharedPtr & intensity_msg,
  const CameraInfo::ConstSharedPtr & info_msg)
{
  auto cloud_msg = std::make_shared<PointCloud>();
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);


  if (info_msg->d != D_ || info_msg->k != K_ || width_ != info_msg->width ||
    height_ != info_msg->height)
  {
    D_ = info_msg->d;
    K_ = info_msg->k;
    width_ = info_msg->width;
    height_ = info_msg->height;
    transform_ = initMatrix(cv::Mat_<double>(3, 3, &K_[0]), cv::Mat(D_), width_, height_, true);
  }

  // Convert Depth Image to Pointcloud
  if (depth_msg->encoding == enc::TYPE_16UC1) {
    convertDepthRadial<uint16_t>(depth_msg, cloud_msg, transform_);
  } else if (depth_msg->encoding == enc::TYPE_32FC1) {
    convertDepthRadial<float>(depth_msg, cloud_msg, transform_);
  } else {
    RCLCPP_ERROR(logger_, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  if (intensity_msg->encoding == enc::MONO8) {
    convertIntensity<uint8_t>(intensity_msg, cloud_msg);
  } else if (intensity_msg->encoding == enc::MONO16) {
    convertIntensity<uint16_t>(intensity_msg, cloud_msg);
  } else if (intensity_msg->encoding == enc::TYPE_16UC1) {
    convertIntensity<uint16_t>(intensity_msg, cloud_msg);
  } else {
    RCLCPP_ERROR(
      logger_, "Intensity image has unsupported encoding [%s]", intensity_msg->encoding.c_str());
    return;
  }

  pub_point_cloud_->publish(*cloud_msg);
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::PointCloudXyziRadialNode)
