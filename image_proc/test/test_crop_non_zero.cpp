// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "image_proc/crop_non_zero.hpp"

using namespace std::chrono_literals;

// Regression test: publishing an all-zero 8UC1 image used to crash the
// CropNonZeroNode because cv::findContours returned an empty contour vector
// and the code then indexed it with [] (out-of-bounds on empty vector).
TEST(CropNonZeroNodeTest, AllBlackImageDoesNotCrash)
{
  const std::string ns = "/crop_non_zero_test";
  const std::string topic_raw = ns + "/image_raw";
  const std::string topic_out = ns + "/image";

  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "-r", std::string("__ns:=") + ns});
  auto crop_node = std::make_shared<image_proc::CropNonZeroNode>(options);

  // Spin the crop node in its own thread so matched_callback can fire
  // and the lazy subscription can activate. This matches the pattern
  // used by test_rectify.cpp.
  std::thread crop_spin_thread(
    [crop_node]() {
      rclcpp::spin(crop_node);
    });

  auto helper_node = rclcpp::Node::make_shared("crop_non_zero_test_helper");
  rclcpp::executors::SingleThreadedExecutor helper_exec;
  helper_exec.add_node(helper_node);

  std::atomic<int> output_count{0};
  image_transport::ImageTransport it_helper(helper_node);
  auto out_sub = it_helper.subscribe(
    topic_out, 1,
    [&output_count](const sensor_msgs::msg::Image::ConstSharedPtr &) {
      output_count.fetch_add(1);
    });
  auto pub = it_helper.advertise(topic_raw, 1);

  // Wait for the crop node to wire up its input subscription (triggered
  // by our subscription to the output topic above).
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (pub.getNumSubscribers() == 0 &&
    std::chrono::steady_clock::now() < deadline)
  {
    helper_exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_GT(pub.getNumSubscribers(), 0u)
    << "CropNonZeroNode did not subscribe to input topic in time.";

  // Publish an all-black 8UC1 image; pre-fix this crashed the node.
  cv::Mat black = cv::Mat::zeros(64, 64, CV_8UC1);
  auto black_msg = cv_bridge::CvImage(
    std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_8UC1, black)
    .toImageMsg();
  pub.publish(*black_msg);

  // Spin helper for a bit. We expect NO output (fix returns early) and
  // crucially no crash in the crop node's spin thread.
  auto spin_until = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < spin_until) {
    helper_exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_EQ(output_count.load(), 0)
    << "CropNonZeroNode should not publish output for an all-black image.";

  // Follow up with a valid image to confirm the node is still alive and
  // functional after the malformed input.
  cv::Mat valid = cv::Mat::zeros(64, 64, CV_8UC1);
  cv::rectangle(valid, cv::Rect(10, 10, 20, 20), cv::Scalar(255), cv::FILLED);
  auto valid_msg = cv_bridge::CvImage(
    std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_8UC1, valid)
    .toImageMsg();
  pub.publish(*valid_msg);

  spin_until = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < spin_until &&
    output_count.load() == 0)
  {
    helper_exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_GT(output_count.load(), 0)
    << "CropNonZeroNode should still process valid images after an all-black one.";

  // Shutdown stops the spin thread; join before the nodes go out of scope
  // so matched_callback cannot fire on a partially-destroyed node.
  rclcpp::shutdown();
  crop_spin_thread.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
