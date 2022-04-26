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
#include <depth_image_proc/conversions.hpp>

#include <limits>
#include <vector>

namespace depth_image_proc
{

cv::Mat initMatrix(
  cv::Mat cameraMatrix, cv::Mat distCoeffs,
  int width, int height, bool radial)
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

void convertRgb(
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
  sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
  int red_offset, int green_offset, int blue_offset, int color_step)
{
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
  const uint8_t * rgb = &rgb_msg->data[0];
  int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;
  for (int v = 0; v < static_cast<int>(cloud_msg->height); ++v, rgb += rgb_skip) {
    for (int u = 0; u < static_cast<int>(cloud_msg->width); ++u,
      rgb += color_step, ++iter_r, ++iter_g, ++iter_b)
    {
      *iter_r = rgb[red_offset];
      *iter_g = rgb[green_offset];
      *iter_b = rgb[blue_offset];
    }
  }
}
}  // namespace depth_image_proc
