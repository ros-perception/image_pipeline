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

#include <boost/foreach.hpp>

#include <string>

class ImageProcTest
  : public testing::Test
{
protected:
  virtual void SetUp()
  {
    ros::NodeHandle local_nh("~");

    // Determine topic names
    std::string camera_ns = nh.resolveName("camera") + "/";

    if (camera_ns == "/camera") {
      throw "Must remap 'camera' to the camera namespace.";
    }

    topic_raw = camera_ns + "image_raw";
    topic_mono = camera_ns + "image_mono";
    topic_rect = camera_ns + "image_rect";
    topic_color = camera_ns + "image_color";
    topic_rect_color = camera_ns + "image_rect_color";

    // Load raw image and cam info
    /// @todo Make these cmd-line args instead?
    std::string raw_image_file, cam_info_file;

    if (!local_nh.getParam("raw_image_file", raw_image_file)) {
      throw "Must set parameter ~raw_image_file.";
    }

    if (!local_nh.getParam("camera_info_file", cam_info_file)) {
      throw "Must set parameter ~camera_info_file.";
    }

    /// @todo Test variety of encodings for raw image (bayer, mono, color)
    cv::Mat img = cv::imread(raw_image_file, 0);
    raw_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    std::string cam_name;

    if (!camera_calibration_parsers::readCalibration(cam_info_file, cam_name, cam_info)) {
      throw "Failed to read camera info file.";
    }

    // Create raw camera publisher
    image_transport::ImageTransport it(nh);
    cam_pub = it.advertiseCamera(topic_raw, 1);

    // Wait for image_proc to be operational
    ros::master::V_TopicInfo topics;
    while (true) {
      if (ros::master::getTopics(topics)) {
        BOOST_FOREACH(ros::master::TopicInfo & topic, topics) {
          if (topic.name == topic_rect_color) {
            return;
          }
        }
      }

      ros::Duration(0.5).sleep();
    }
  }

  ros::NodeHandle nh;
  std::string topic_raw;
  std::string topic_mono;
  std::string topic_rect;
  std::string topic_color;
  std::string topic_rect_color;

  sensor_msgs::ImagePtr raw_image;
  sensor_msgs::CameraInfo cam_info;
  image_transport::CameraPublisher cam_pub;

  void publishRaw()
  {
    cam_pub.publish(*raw_image, cam_info);
  }
};

void callback(const sensor_msgs::ImageConstPtr & msg)
{
  ROS_FATAL("Got an image");
  ros::shutdown();
}

TEST_F(ImageProcTest, monoSubscription)
{
  ROS_INFO("In test. Subscribing.");
  ros::Subscriber mono_sub = nh.subscribe(topic_mono, 1, callback);
  ROS_INFO("Publishing.");
  publishRaw();

  ROS_INFO("Spinning.");
  ros::spin();
  ROS_INFO("Done.");
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "imageproc_rostest");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
