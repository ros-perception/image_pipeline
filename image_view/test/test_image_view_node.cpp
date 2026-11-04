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
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "image_view/image_view_node.hpp"

using namespace std::chrono_literals;

namespace
{

sensor_msgs::msg::Image make_image(
  const std::string & frame_id,
  const std::string & encoding,
  uint32_t width,
  uint32_t height,
  uint32_t step,
  std::vector<uint8_t> data)
{
  sensor_msgs::msg::Image msg;
  msg.header.frame_id = frame_id;
  msg.encoding = encoding;
  msg.width = width;
  msg.height = height;
  msg.step = step;
  msg.data = std::move(data);
  return msg;
}

}  // namespace

// Regression test: the image callback used to lock image_mutex_ by hand around the
// cv_bridge conversion. When the conversion threw, the mutex was never unlocked and
// the next image deadlocked the executor, so the node stopped republishing on
// 'output' (and rendering) for good.
TEST(ImageViewNodeTest, KeepsRunningAfterUnconvertibleImages)
{
  const std::string ns = "/image_view_test";

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", std::string("__ns:=") + ns});
  // No window, so the test runs headless. imageCb() takes the lock either way.
  options.parameter_overrides({rclcpp::Parameter("gui", false)});
  auto node = std::make_shared<image_view::ImageViewNode>(options);

  rclcpp::executors::SingleThreadedExecutor node_exec;
  node_exec.add_node(node);
  std::thread node_spin_thread([&node_exec]() {node_exec.spin();});

  auto helper_node = rclcpp::Node::make_shared("image_view_test_helper");
  rclcpp::executors::SingleThreadedExecutor helper_exec;
  helper_exec.add_node(helper_node);

  std::atomic<int> valid_output_count{0};
  auto out_sub = helper_node->create_subscription<sensor_msgs::msg::Image>(
    ns + "/output", 10,
    [&valid_output_count](sensor_msgs::msg::Image::ConstSharedPtr msg) {
      if (msg->header.frame_id == "valid") {
        valid_output_count.fetch_add(1);
      }
    });
  auto image_pub = helper_node->create_publisher<sensor_msgs::msg::Image>(
    ns + "/image", rclcpp::SensorDataQoS());

  // Wait for both sides to discover each other
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while ((image_pub->get_subscription_count() == 0 || out_sub->get_publisher_count() == 0) &&
    std::chrono::steady_clock::now() < deadline)
  {
    helper_exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_GT(image_pub->get_subscription_count(), 0u)
    << "ImageViewNode did not subscribe to 'image' in time";
  ASSERT_GT(out_sub->get_publisher_count(), 0u)
    << "ImageViewNode did not advertise 'output' in time";

  const uint32_t w = 8;
  const uint32_t h = 4;

  // cvtColorForDisplay() cannot produce a 32FC1 image for display and throws
  image_pub->publish(
    make_image(
      "bad", sensor_msgs::image_encodings::TYPE_32FC1,
      w, h, w * 4, std::vector<uint8_t>(w * h * 4, 0)));
  // toCvShare() rejects an image whose data does not match height * step and throws
  image_pub->publish(
    make_image(
      "bad", sensor_msgs::image_encodings::BGR8,
      w, h, w * 3, std::vector<uint8_t>(10, 0)));

  // Valid images must still be processed afterwards
  deadline = std::chrono::steady_clock::now() + 5s;
  while (valid_output_count.load() < 5 && std::chrono::steady_clock::now() < deadline) {
    image_pub->publish(
      make_image(
        "valid", sensor_msgs::image_encodings::BGR8,
        w, h, w * 3, std::vector<uint8_t>(w * h * 3, 0)));
    helper_exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_GE(valid_output_count.load(), 5)
    << "ImageViewNode stopped republishing after images it could not convert";

  node_exec.cancel();
  node_spin_thread.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
