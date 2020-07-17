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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <camera_calibration_parsers/parse.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/distortion_models.hpp>

#include <algorithm>
#include <string>

class ImageProcRectifyTest
  : public testing::Test
{
protected:
  virtual void SetUp()
  {
    // Determine topic names
    std::string camera_ns = nh_.resolveName("camera") + "/";

    if (camera_ns == "/camera") {
      throw "Must remap 'camera' to the camera namespace.";
    }

    topic_raw_ = camera_ns + "image_raw";
    topic_mono_ = camera_ns + "image_mono";
    topic_rect_ = camera_ns + "image_rect";
    topic_color_ = camera_ns + "image_color";
    topic_rect_color_ = camera_ns + "image_rect_color";

    // Taken from vision_opencv/image_geometry/test/utest.cpp
    double D[] =
    {
      -0.363528858080088, 0.16117037733986861,
      -8.1109585007538829e-05, -0.00044776712298447841, 0.0
    };

    double K[] =
    {
      430.15433020105519, 0.0, 311.71339830549732,
      0.0, 430.60920415473657, 221.06824942698509,
      0.0, 0.0, 1.0
    };

    double R[] =
    {
      0.99806560714807102, 0.0068562422224214027, 0.061790256276695904,
      -0.0067522959054715113, 0.99997541519165112, -0.0018909025066874664,
      -0.061801701660692349, 0.0014700186639396652, 0.99808736527268516
    };

    double P[] =
    {
      295.53402059708782, 0.0, 285.55760765075684, 0.0,
      0.0, 295.53402059708782, 223.29617881774902, 0.0,
      0.0, 0.0, 1.0, 0.0
    };

    cam_info_.header.frame_id = "tf_frame";
    cam_info_.height = 480;
    cam_info_.width = 640;
    // No ROI
    cam_info_.D.resize(5);
    std::copy(D, D + 5, cam_info_.D.begin());
    std::copy(K, K + 9, cam_info_.K.begin());
    std::copy(R, R + 9, cam_info_.R.begin());
    std::copy(P, P + 12, cam_info_.P.begin());
    cam_info_.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    distorted_image_ = cv::Mat(cv::Size(cam_info_.width, cam_info_.height), CV_8UC3);
    // draw a grid
    const cv::Scalar color = cv::Scalar(255, 255, 255);
    // draw the lines thick so the proportion of error due to
    // interpolation is reduced
    const int thickness = 7;
    const int type = 8;
    for (size_t y = 0; y <= cam_info_.height; y += cam_info_.height / 10) {
      cv::line(
        distorted_image_, cv::Point(0, y), cv::Point(cam_info_.width, y),
        color, type, thickness);
    }

    for (size_t x = 0; x <= cam_info_.width; x += cam_info_.width / 10) {
      // draw the lines thick so the prorportion of interpolation error is reduced
      cv::line(
        distorted_image_, cv::Point(x, 0), cv::Point(x, cam_info_.height),
        color, type, thickness);
    }

    raw_image_ = cv_bridge::CvImage(
      std_msgs::Header(), "bgr8", distorted_image_).toImageMsg();

    // Create raw camera subscriber and publisher
    image_transport::ImageTransport it(nh_);
    cam_pub_ = it.advertiseCamera(topic_raw_, 1);
  }

  ros::NodeHandle nh_;
  std::string topic_raw_;
  std::string topic_mono_;
  std::string topic_rect_;
  std::string topic_color_;
  std::string topic_rect_color_;

  cv::Mat distorted_image_;
  sensor_msgs::ImagePtr raw_image_;
  bool has_new_image_;
  cv::Mat received_image_;
  sensor_msgs::CameraInfo cam_info_;
  image_transport::CameraPublisher cam_pub_;
  image_transport::Subscriber cam_sub_;

public:
  void imageCallback(const sensor_msgs::ImageConstPtr & msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
      ROS_FATAL("cv_bridge exception: %s", e.what());
      return;
    }

    received_image_ = cv_ptr->image.clone();
    has_new_image_ = true;
  }

  void publishRaw()
  {
    has_new_image_ = false;
    cam_pub_.publish(*raw_image_, cam_info_);
  }
};

TEST_F(ImageProcRectifyTest, rectifyTest)
{
  ROS_INFO("In test. Subscribing.");
  image_transport::ImageTransport it(nh_);
  cam_sub_ = it.subscribe(
    topic_rect_, 1, &ImageProcRectifyTest::imageCallback,
    dynamic_cast<ImageProcRectifyTest *>(this));

  // Wait for image_proc to be operational
  bool wait_for_topic = true;

  while (wait_for_topic) {
    // @todo this fails without the additional 0.5 second sleep after the
    // publisher comes online, which means on a slower or more heavily
    // loaded system it may take longer than 0.5 seconds, and the test
    // would hang until the timeout is reached and fail.
    if (cam_sub_.getNumPublishers() > 0) {
      wait_for_topic = false;
    }

    ros::Duration(0.5).sleep();
  }

  // All the tests are the same as from
  // vision_opencv/image_geometry/test/utest.cpp
  // default cam info

  // Just making this number up, maybe ought to be larger
  // since a completely different image would be on the order of
  // width * height * 255 = 78e6
  const double diff_threshold = 10000.0;
  double error;

  // use original cam_info
  publishRaw();
  while (!has_new_image_) {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }

  // Test that rectified image is sufficiently different
  // using default distortion
  error = cv::norm(distorted_image_, received_image_, cv::NORM_L1);
  // Just making this number up, maybe ought to be larger
  EXPECT_GT(error, diff_threshold);

  // Test that rectified image is sufficiently different
  // using default distortion but with first element zeroed
  // out.
  sensor_msgs::CameraInfo cam_info_orig = cam_info_;
  cam_info_.D[0] = 0.0;
  publishRaw();

  while (!has_new_image_) {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }

  error = cv::norm(distorted_image_, received_image_, cv::NORM_L1);
  EXPECT_GT(error, diff_threshold);

  // Test that rectified image is the same using zero distortion
  cam_info_.D.assign(cam_info_.D.size(), 0);
  publishRaw();

  while (!has_new_image_) {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }

  error = cv::norm(distorted_image_, received_image_, cv::NORM_L1);
  EXPECT_EQ(error, 0);

  // Test that rectified image is the same using empty distortion
  cam_info_.D.clear();
  publishRaw();

  while (!has_new_image_) {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }

  error = cv::norm(distorted_image_, received_image_, cv::NORM_L1);

  EXPECT_EQ(error, 0);

  // restore the original cam_info for other tests added in the future
  cam_info_ = cam_info_orig;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "image_proc_test_rectify");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
