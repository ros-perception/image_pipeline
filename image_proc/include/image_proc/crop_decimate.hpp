// Copyright 2008, 2019 Willow Garage, Inc., Steve Macenski, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
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

#ifndef IMAGE_PROC__CROP_DECIMATE_HPP_
#define IMAGE_PROC__CROP_DECIMATE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <thread>
#include <memory>
#include <vector>
#include <string>

namespace image_proc
{

enum class CropDecimateModes
{
  CropDecimate_NN = 0,
  CropDecimate_Linear = 1,
  CropDecimate_Cubic = 2,
  CropDecimate_Area = 3,
  CropDecimate_Lanczos4 = 4
};

using cv_bridge::CvImage;
using cv_bridge::CvImageConstPtr;
using cv_bridge::toCvShare;

class CropDecimateNode : public rclcpp::Node
{
public:
  explicit CropDecimateNode(const rclcpp::NodeOptions &);

private:
  image_transport::CameraSubscriber sub_;
  image_transport::CameraPublisher pub_;
  int queue_size_;
  std::string target_frame_id_;
  int decimation_x_, decimation_y_, offset_x_, offset_y_, width_, height_;
  CropDecimateModes interpolation_;

  void imageCb(
    const sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);
};

}  // namespace image_proc

#endif  // IMAGE_PROC__CROP_DECIMATE_HPP_
