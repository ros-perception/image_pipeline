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
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "depth_image_proc/convert_metric.hpp"

using namespace std::chrono_literals;

namespace
{

sensor_msgs::msg::Image make_image(
  const std::string & encoding,
  uint32_t width,
  uint32_t height,
  uint32_t step,
  std::vector<uint8_t> data)
{
  sensor_msgs::msg::Image msg;
  msg.header.frame_id = "camera";
  msg.encoding = encoding;
  msg.width = width;
  msg.height = height;
  msg.step = step;
  msg.data = std::move(data);
  return msg;
}

}  // namespace

// Regression test: publishing a 16UC1 image with empty data used to dereference
// &raw_msg->data[0] on an empty vector, producing a null-address read.
TEST(ConvertMetricNodeTest, MalformedImageDoesNotCrash)
{
  const std::string ns = "/convert_metric_test";
  const std::string topic_raw = ns + "/image_raw";
  const std::string topic_out = ns + "/image";

  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "-r", std::string("__ns:=") + ns});
  auto node = std::make_shared<depth_image_proc::ConvertMetricNode>(options);

  // Spin the node in its own thread so matched_callback can fire
  // and the lazy subscription can activate.
  std::thread node_spin_thread(
    [node]() {
      rclcpp::spin(node);
    });

  auto helper_node = rclcpp::Node::make_shared("convert_metric_test_helper");
  rclcpp::executors::SingleThreadedExecutor helper_exec;
  helper_exec.add_node(helper_node);

  std::atomic<int> output_count{0};
  auto out_sub = helper_node->create_subscription<sensor_msgs::msg::Image>(
    topic_out, rclcpp::SensorDataQoS(),
    [&output_count](sensor_msgs::msg::Image::ConstSharedPtr) {
      output_count.fetch_add(1);
    });
  auto raw_pub = helper_node->create_publisher<sensor_msgs::msg::Image>(
    topic_raw, rclcpp::SensorDataQoS());

  // Wait for the lazy subscription on the node side to come up.
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (raw_pub->get_subscription_count() == 0 &&
    std::chrono::steady_clock::now() < deadline)
  {
    helper_exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_GT(raw_pub->get_subscription_count(), 0u)
    << "ConvertMetricNode did not subscribe to input topic in time.";

  // Case 1: malformed 16UC1 with empty data (reported reproducer).
  raw_pub->publish(
    make_image(
      sensor_msgs::image_encodings::TYPE_16UC1,
      640, 480, 1280, {}));

  // Case 2: truncated 16UC1 (data.size() < width*height*2).
  raw_pub->publish(
    make_image(
      sensor_msgs::image_encodings::TYPE_16UC1,
      640, 480, 1280, std::vector<uint8_t>(100, 0)));

  // Case 3: truncated 32FC1.
  raw_pub->publish(
    make_image(
      sensor_msgs::image_encodings::TYPE_32FC1,
      640, 480, 640 * 4, std::vector<uint8_t>(100, 0)));

  // Pump the helper for a bit; none of the above should produce output
  // and, crucially, none should crash the node's spin thread.
  auto spin_until = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < spin_until) {
    helper_exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(output_count.load(), 0)
    << "Malformed inputs should not produce output.";

  // Case 4: a valid 16UC1 image - node should still be alive and functional.
  const uint32_t w = 8;
  const uint32_t h = 4;
  std::vector<uint8_t> valid_data(w * h * sizeof(uint16_t), 0);
  // Write a non-zero depth value (1000mm -> 1.0m after conversion) to one pixel.
  uint16_t * p = reinterpret_cast<uint16_t *>(valid_data.data());
  p[0] = 1000;
  raw_pub->publish(
    make_image(
      sensor_msgs::image_encodings::TYPE_16UC1,
      w, h, w * sizeof(uint16_t), std::move(valid_data)));

  spin_until = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < spin_until &&
    output_count.load() == 0)
  {
    helper_exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_GT(output_count.load(), 0)
    << "ConvertMetricNode should still process valid images after malformed ones.";

  rclcpp::shutdown();
  node_spin_thread.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
