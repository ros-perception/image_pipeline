// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include <cmath>

#include "image_geometry/stereo_camera_model.hpp"
#include "stereo_image_proc/stereo_processor.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

namespace
{

constexpr int kWidth = 8;
constexpr int kHeight = 6;
constexpr double kFx = 500.0;
constexpr double kFy = 520.0;
constexpr double kCxLeft = 330.0;
constexpr double kCxRight = 310.0;  // principal points 20 px apart, as with verged cameras
constexpr double kCy = 240.0;
constexpr double kBaseline = 0.1;
// DisparityImage disparity, already adjusted for cx_l - cx_r.
constexpr float kDisparity = 25.0f;
constexpr float kInvalidDisparity = -1.0f;  // the image's minimum, marked as missing
constexpr double kTolerance = 1e-5;

sensor_msgs::msg::CameraInfo makeInfo(double cx, double Tx)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 640;
  info.height = 480;
  info.k = {kFx, 0.0, cx, 0.0, kFy, kCy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {kFx, 0.0, cx, Tx, 0.0, kFy, kCy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

class ProjectDisparityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(
      model_.fromCameraInfo(makeInfo(kCxLeft, 0.0), makeInfo(kCxRight, -kFx * kBaseline)));
    ASSERT_NE(model_.left().cx(), model_.right().cx());

    disparity_.f = static_cast<float>(model_.right().fx());
    disparity_.t = static_cast<float>(model_.baseline());
    sensor_msgs::msg::Image & dimage = disparity_.image;
    dimage.height = kHeight;
    dimage.width = kWidth;
    dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    dimage.step = kWidth * sizeof(float);
    dimage.data.resize(dimage.step * kHeight);
    cv::Mat_<float> dmat(kHeight, kWidth, reinterpret_cast<float *>(dimage.data.data()));
    dmat = kDisparity;
    dmat(0, 0) = kInvalidDisparity;

    color_ = cv::Mat(kHeight, kWidth, CV_8UC1, cv::Scalar(128));
  }

  // Z = fT/d, as stereo_msgs/DisparityImage specifies.
  double expectedZ() const {return disparity_.f * disparity_.t / kDisparity;}
  double expectedX(int u) const {return (u - kCxLeft) * expectedZ() / kFx;}
  double expectedY(int v) const {return (v - kCy) * expectedZ() / kFy;}

  image_geometry::StereoCameraModel model_;
  stereo_msgs::msg::DisparityImage disparity_;
  cv::Mat color_;
  stereo_image_proc::StereoProcessor processor_;
};

}  // namespace

TEST_F(ProjectDisparityTest, ProjectAdjustedDisparityImageTo3d)
{
  const cv::Mat_<float> dmat(
    kHeight, kWidth, reinterpret_cast<float *>(disparity_.image.data.data()));
  cv::Mat points;
  stereo_image_proc::projectAdjustedDisparityImageTo3d(model_, dmat, points);
  const cv::Mat_<cv::Vec3f> pts = points;

  EXPECT_FLOAT_EQ(
    pts(0, 0)[2], static_cast<float>(image_geometry::StereoCameraModel::MISSING_Z));
  for (int v = 0; v < kHeight; ++v) {
    for (int u = 0; u < kWidth; ++u) {
      if (u == 0 && v == 0) {
        continue;
      }
      EXPECT_NEAR(pts(v, u)[0], expectedX(u), kTolerance) << "u=" << u << " v=" << v;
      EXPECT_NEAR(pts(v, u)[1], expectedY(v), kTolerance) << "u=" << u << " v=" << v;
      EXPECT_NEAR(pts(v, u)[2], expectedZ(), kTolerance) << "u=" << u << " v=" << v;
    }
  }
}

TEST_F(ProjectDisparityTest, ProcessPoints)
{
  sensor_msgs::msg::PointCloud points;
  processor_.processPoints(
    disparity_, color_, sensor_msgs::image_encodings::MONO8, model_, points);

  // Every pixel but the missing one; channels 1 and 2 hold the pixel's row and column.
  ASSERT_EQ(points.points.size(), static_cast<size_t>(kWidth * kHeight - 1));
  ASSERT_EQ(points.channels.size(), 3u);
  for (size_t i = 0; i < points.points.size(); ++i) {
    const int v = static_cast<int>(points.channels[1].values[i]);
    const int u = static_cast<int>(points.channels[2].values[i]);
    EXPECT_NEAR(points.points[i].x, expectedX(u), kTolerance) << "u=" << u << " v=" << v;
    EXPECT_NEAR(points.points[i].y, expectedY(v), kTolerance) << "u=" << u << " v=" << v;
    EXPECT_NEAR(points.points[i].z, expectedZ(), kTolerance) << "u=" << u << " v=" << v;
  }
}

TEST_F(ProjectDisparityTest, ProcessPoints2)
{
  sensor_msgs::msg::PointCloud2 points;
  processor_.processPoints2(
    disparity_, color_, sensor_msgs::image_encodings::MONO8, model_, points);

  ASSERT_EQ(points.height, static_cast<uint32_t>(kHeight));
  ASSERT_EQ(points.width, static_cast<uint32_t>(kWidth));
  sensor_msgs::PointCloud2ConstIterator<float> it_x(points, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(points, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(points, "z");
  for (int v = 0; v < kHeight; ++v) {
    for (int u = 0; u < kWidth; ++u, ++it_x, ++it_y, ++it_z) {
      if (u == 0 && v == 0) {
        EXPECT_TRUE(std::isnan(*it_z));
        continue;
      }
      EXPECT_NEAR(*it_x, expectedX(u), kTolerance) << "u=" << u << " v=" << v;
      EXPECT_NEAR(*it_y, expectedY(v), kTolerance) << "u=" << u << " v=" << v;
      EXPECT_NEAR(*it_z, expectedZ(), kTolerance) << "u=" << u << " v=" << v;
    }
  }
}
