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
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_conversions.hpp>
#include <depth_image_proc/visibility.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <memory>

namespace depth_image_proc
{

namespace enc = sensor_msgs::image_encodings;

class PointCloudXyzNode : public rclcpp::Node
{
public:
  DEPTH_IMAGE_PROC_PUBLIC PointCloudXyzNode();

private:
  // Subscriptions
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_depth_;
  int queue_size_;

  // Publications
  std::mutex connect_mutex_;
  using PointCloud = sensor_msgs::msg::PointCloud2;
  rclcpp::Publisher<PointCloud>::SharedPtr pub_point_cloud_;

  image_geometry::PinholeCameraModel model_;

  void connectCb(rclcpp::Node::SharedPtr node);

  void depthCb(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

  rclcpp::Logger logger_ = rclcpp::get_logger("PointCloudXyzNode");
};

PointCloudXyzNode::PointCloudXyzNode()
: Node("PointCloudXyzNode")
{
  rclcpp::Node::SharedPtr node = std::shared_ptr<rclcpp::Node>(this);
  it_.reset(new image_transport::ImageTransport(node));

  // Read parameters
  this->get_parameter_or("queue_size", queue_size_, 5);

  // Monitor whether anyone is subscribed to the output
  // TODO(ros2) Implement when SubscriberStatusCallback is available
  // ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloudXyzNode::connectCb, this);
  connectCb(node);

  // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
  std::lock_guard<std::mutex> lock(connect_mutex_);
  // TODO(ros2) Implement when SubscriberStatusCallback is available
  // pub_point_cloud_ = nh.advertise<PointCloud>("points", 1, connect_cb, connect_cb);
  pub_point_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("points");
}

// Handles (un)subscribing when clients (un)subscribe
void PointCloudXyzNode::connectCb(rclcpp::Node::SharedPtr node)
{
  std::lock_guard<std::mutex> lock(connect_mutex_);
  // TODO(ros2) Implement getNumSubscribers when rcl/rmw support it
  // if (pub_point_cloud_->getNumSubscribers() == 0)
  if (0) {
    sub_depth_.shutdown();
  } else if (!sub_depth_) {
    image_transport::TransportHints hints(node, "raw");
    sub_depth_ =
      it_->subscribeCamera("image_rect", queue_size_, &PointCloudXyzNode::depthCb, this, &hints);
  }
}

void PointCloudXyzNode::depthCb(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  PointCloud::SharedPtr cloud_msg(new PointCloud);
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  // Update camera model
  model_.fromCameraInfo(info_msg);

  if (depth_msg->encoding == enc::TYPE_16UC1) {
    convert<uint16_t>(depth_msg, cloud_msg, model_);
  } else if (depth_msg->encoding == enc::TYPE_32FC1) {
    convert<float>(depth_msg, cloud_msg, model_);
  } else {
    RCLCPP_ERROR(logger_, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  pub_point_cloud_->publish(cloud_msg);
}

}  // namespace depth_image_proc

#include "class_loader/register_macro.hpp"

// Register the component with class_loader.
CLASS_LOADER_REGISTER_CLASS(depth_image_proc::PointCloudXyzNode, rclcpp::Node)
