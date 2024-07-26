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

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <camera_calibration_parsers/parse.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.hpp>

using std::placeholders::_1;

class ImageProcTest
  : public testing::Test
{
protected:
  virtual void SetUp()
  {
    node = rclcpp::Node::make_shared("image_proc_test");

    // Determine topic names
    std::string camera_ns = node->get_node_topics_interface()->resolve_topic_name("camera") + "/";

    if (camera_ns == "/camera") {
      throw "Must remap 'camera' to the camera namespace.";
    }

    topic_raw = camera_ns + "image_raw";
    topic_mono = camera_ns + "image_mono";
    topic_rect = camera_ns + "image_rect";
    topic_color = camera_ns + "image_color";
    topic_rect_color = camera_ns + "image_rect_color";

    const std::filesystem::path base{_SRC_RESOURCES_DIR_PATH};
    const std::filesystem::path raw_image_file = base / "logo.png";
    const std::filesystem::path cam_info_file = base / "calibration_file.ini";

    /// @todo Test variety of encodings for raw image (bayer, mono, color)
    cv::Mat img = cv::imread(raw_image_file.string(), 0);
    raw_image = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img).toImageMsg();
    std::string cam_name;

    if (!camera_calibration_parsers::readCalibration(cam_info_file.string(), cam_name, cam_info)) {
      throw "Failed to read camera info file.";
    }

    // Create raw camera publisher
    image_transport::ImageTransport it(this->node);
    cam_pub = it.advertiseCamera(topic_raw, 1);

    while (true) {
      // Wait for image_proc to be operational
      auto topic_names_and_types = this->node->get_topic_names_and_types();
      for (const auto & map_pair : topic_names_and_types) {
        if (map_pair.first == topic_raw) {
          return;
        }
      }
    }
  }

  rclcpp::Node::SharedPtr node;
  std::string topic_raw;
  std::string topic_mono;
  std::string topic_rect;
  std::string topic_color;
  std::string topic_rect_color;

  sensor_msgs::msg::Image::SharedPtr raw_image;
  sensor_msgs::msg::CameraInfo cam_info;
  image_transport::CameraPublisher cam_pub;

  void publishRaw()
  {
    cam_pub.publish(*raw_image, cam_info);
  }

public:
  bool has_new_image_{false};
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr & /*msg*/)
  {
    RCLCPP_INFO(node->get_logger(), "Got an image");
    has_new_image_ = true;
  }
};

TEST_F(ImageProcTest, monoSubscription)
{
  RCLCPP_INFO(node->get_logger(), "In test. Subscribing.");
  auto mono_sub = node->create_subscription<sensor_msgs::msg::Image>(
    topic_raw, 1, std::bind(&ImageProcTest::callback, this, _1));

  RCLCPP_INFO(node->get_logger(), "Publishing");

  RCLCPP_INFO(node->get_logger(), "Spinning");
  while (!has_new_image_) {
    publishRaw();
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Done");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
