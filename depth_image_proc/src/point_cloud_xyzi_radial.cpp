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
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.hpp>
#include <depth_image_proc/visibility.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <memory>
#include <vector>
#include <string>
#include <limits>

namespace depth_image_proc
{

using namespace std::placeholders;
namespace enc = sensor_msgs::image_encodings;
using SyncPolicy =
  message_filters::sync_policies::ExactTime<
  sensor_msgs::msg::Image,
  sensor_msgs::msg::Image,
  sensor_msgs::msg::CameraInfo>;

class PointCloudXyziRadialNode : public rclcpp::Node
{
public:
  DEPTH_IMAGE_PROC_PUBLIC PointCloudXyziRadialNode(const rclcpp::NodeOptions & options);

private:
  using PointCloud = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  // Subscriptions
  image_transport::SubscriberFilter sub_depth_, sub_intensity_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_info_;

  int queue_size_;

  // Publications
  std::mutex connect_mutex_;
  rclcpp::Publisher<PointCloud>::SharedPtr pub_point_cloud_;

  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Synchronizer> sync_;

  std::vector<double> D_;
  std::array<double, 9> K_;

  uint32_t width_;
  uint32_t height_;

  cv::Mat transform_;

  void connectCb();

  void imageCb(
    const Image::ConstSharedPtr & depth_msg,
    const Image::ConstSharedPtr & intensity_msg_in,
    const CameraInfo::ConstSharedPtr & info_msg);

  // Handles float or uint16 depths
  template<typename T>
  void convert_depth(
    const Image::ConstSharedPtr & depth_msg,
    PointCloud::SharedPtr & cloud_msg);

  template<typename T>
  void convert_intensity(
    const Image::ConstSharedPtr & inten_msg,
    PointCloud::SharedPtr & cloud_msg);

  cv::Mat initMatrix(cv::Mat cameraMatrix, cv::Mat distCoeffs, int width, int height, bool radial);

  rclcpp::Logger logger_ = rclcpp::get_logger("PointCloudXyziRadialNode");
};

cv::Mat PointCloudXyziRadialNode::initMatrix(
  cv::Mat cameraMatrix, cv::Mat distCoeffs, int width,
  int height, bool radial)
{
  int i, j;
  int totalsize = width * height;
  cv::Mat pixelVectors(1, totalsize, CV_32FC3);
  cv::Mat dst(1, totalsize, CV_32FC3);

  cv::Mat sensorPoints(cv::Size(height, width), CV_32FC2);
  cv::Mat undistortedSensorPoints(1, totalsize, CV_32FC2);

  std::vector<cv::Mat> ch;
  for (j = 0; j < height; j++) {
    for (i = 0; i < width; i++) {
      cv::Vec2f & p = sensorPoints.at<cv::Vec2f>(i, j);
      p[0] = i;
      p[1] = j;
    }
  }

  sensorPoints = sensorPoints.reshape(2, 1);

  cv::undistortPoints(sensorPoints, undistortedSensorPoints, cameraMatrix, distCoeffs);

  ch.push_back(undistortedSensorPoints);
  ch.push_back(cv::Mat::ones(1, totalsize, CV_32FC1));
  cv::merge(ch, pixelVectors);

  if (radial) {
    for (i = 0; i < totalsize; i++) {
      normalize(
        pixelVectors.at<cv::Vec3f>(i),
        dst.at<cv::Vec3f>(i));
    }
    pixelVectors = dst;
  }
  return pixelVectors.reshape(3, width);
}


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
  sync_->registerCallback(std::bind(&PointCloudXyziRadialNode::imageCb, this, _1, _2, _3));

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

  if (depth_msg->encoding == enc::TYPE_16UC1) {
    convert_depth<uint16_t>(depth_msg, cloud_msg);
  } else if (depth_msg->encoding == enc::TYPE_32FC1) {
    convert_depth<float>(depth_msg, cloud_msg);
  } else {
    RCLCPP_ERROR(logger_, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  if (intensity_msg->encoding == enc::TYPE_16UC1) {
    convert_intensity<uint16_t>(intensity_msg, cloud_msg);

  } else if (intensity_msg->encoding == enc::MONO8) {
    convert_intensity<uint8_t>(intensity_msg, cloud_msg);
  } else {
    RCLCPP_ERROR(
      logger_, "Intensity image has unsupported encoding [%s]",
      intensity_msg->encoding.c_str());
    return;
  }

  pub_point_cloud_->publish(*cloud_msg);
}

template<typename T>
void PointCloudXyziRadialNode::convert_depth(
  const Image::ConstSharedPtr & depth_msg,
  PointCloud::SharedPtr & cloud_msg)
{
  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  const T * depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);

  int row_step = depth_msg->step / sizeof(T);
  for (int v = 0; v < static_cast<int>(cloud_msg->height); ++v, depth_row += row_step) {
    for (int u = 0; u < static_cast<int>(cloud_msg->width); ++u, ++iter_x, ++iter_y, ++iter_z) {
      T depth = depth_row[u];

      // Missing points denoted by NaNs
      if (!DepthTraits<T>::valid(depth)) {
        *iter_x = *iter_y = *iter_z = bad_point;
        continue;
      }
      const cv::Vec3f & cvPoint = transform_.at<cv::Vec3f>(u, v) * DepthTraits<T>::toMeters(depth);
      // Fill in XYZ
      *iter_x = cvPoint(0);
      *iter_y = cvPoint(1);
      *iter_z = cvPoint(2);
    }
  }
}

template<typename T>
void PointCloudXyziRadialNode::convert_intensity(
  const Image::ConstSharedPtr & intensity_msg,
  PointCloud::SharedPtr & cloud_msg)
{
  sensor_msgs::PointCloud2Iterator<float> iter_i(*cloud_msg, "intensity");
  const T * inten_row = reinterpret_cast<const T *>(&intensity_msg->data[0]);

  const int i_row_step = intensity_msg->step / sizeof(T);
  for (int v = 0; v < static_cast<int>(cloud_msg->height); ++v, inten_row += i_row_step) {
    for (int u = 0; u < static_cast<int>(cloud_msg->width); ++u, ++iter_i) {
      *iter_i = inten_row[u];
    }
  }
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::PointCloudXyziRadialNode)
