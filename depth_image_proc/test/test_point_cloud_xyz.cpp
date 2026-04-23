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

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "depth_image_proc/point_cloud_xyz.hpp"

using namespace std::chrono_literals;

namespace
{

sensor_msgs::msg::CameraInfo make_camera_info(uint32_t width, uint32_t height)
{
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = "camera";
  info.width = width;
  info.height = height;
  // Minimal plausible pinhole model so PinholeCameraModel::fromCameraInfo works.
  info.distortion_model = "plumb_bob";
  info.d.assign(5, 0.0);
  info.k = {
    100.0, 0.0, static_cast<double>(width) / 2.0,
    0.0, 100.0, static_cast<double>(height) / 2.0,
    0.0, 0.0, 1.0};
  info.r = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  info.p = {
    100.0, 0.0, static_cast<double>(width) / 2.0, 0.0,
    0.0, 100.0, static_cast<double>(height) / 2.0, 0.0,
    0.0, 0.0, 1.0, 0.0};
  return info;
}

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

// Regression test: publishing a depth image with oversized width/height used
// to trigger a heap-buffer-overflow inside convertDepth() because the
// PointCloud2 backing buffer was too small for the declared number of points.
TEST(PointCloudXyzNodeTest, OversizedDimensionsDoNotCrash)
{
  const std::string ns = "/point_cloud_xyz_test";
  const std::string topic_image = ns + "/image_rect";
  const std::string topic_out = ns + "/points";

  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "-r", std::string("__ns:=") + ns});
  auto node = std::make_shared<depth_image_proc::PointCloudXyzNode>(options);

  std::thread node_spin_thread(
    [node]() {
      rclcpp::spin(node);
    });

  auto helper_node = rclcpp::Node::make_shared("point_cloud_xyz_test_helper");
  rclcpp::executors::SingleThreadedExecutor helper_exec;
  helper_exec.add_node(helper_node);

  std::atomic<int> output_count{0};
  auto out_sub = helper_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_out, rclcpp::SensorDataQoS(),
    [&output_count](sensor_msgs::msg::PointCloud2::ConstSharedPtr) {
      output_count.fetch_add(1);
    });

  image_transport::ImageTransport it_helper(helper_node);
  auto cam_pub = it_helper.advertiseCamera(topic_image, 1);

  // Wait for the lazy subscription on the node side to come up.
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (cam_pub.getNumSubscribers() == 0 &&
    std::chrono::steady_clock::now() < deadline)
  {
    helper_exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_GT(cam_pub.getNumSubscribers(), 0u)
    << "PointCloudXyzNode did not subscribe to input topic in time.";

  // Case 1: oversized dimensions matching the reported reproducer.
  // width * height = 65537 * 65536 overflows uint32_t and would otherwise
  // cause a heap-buffer-overflow in convertDepth.
  {
    const uint32_t w = 65537;
    const uint32_t h = 65536;
    const uint32_t step = w * sizeof(uint16_t);
    auto info = make_camera_info(w, h);
    // Send only ~5 MB of data (not the ~8 GB the dimensions imply).
    auto img = make_image(
      sensor_msgs::image_encodings::TYPE_16UC1, w, h, step,
      std::vector<uint8_t>(5 * 1024 * 1024, 1));
    cam_pub.publish(img, info);
  }

  // Case 2: zero dimensions.
  {
    auto info = make_camera_info(0, 0);
    auto img = make_image(
      sensor_msgs::image_encodings::TYPE_16UC1, 0, 0, 0, {});
    cam_pub.publish(img, info);
  }

  // Case 3: valid dimensions but data buffer too small.
  {
    const uint32_t w = 320;
    const uint32_t h = 240;
    auto info = make_camera_info(w, h);
    auto img = make_image(
      sensor_msgs::image_encodings::TYPE_16UC1, w, h,
      w * sizeof(uint16_t),
      std::vector<uint8_t>(100, 0));  // too small
    cam_pub.publish(img, info);
  }

  // Pump the helper; malformed inputs should produce no output and no crash.
  auto spin_until = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < spin_until) {
    helper_exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(output_count.load(), 0)
    << "Malformed inputs should not produce a point cloud.";

  // Case 4: a valid small depth image. Node should still be alive.
  {
    const uint32_t w = 8;
    const uint32_t h = 4;
    const uint32_t step = w * sizeof(uint16_t);
    std::vector<uint8_t> data(step * h, 0);
    uint16_t * p = reinterpret_cast<uint16_t *>(data.data());
    for (uint32_t i = 0; i < w * h; ++i) {
      p[i] = 1000;  // 1 m
    }
    auto info = make_camera_info(w, h);
    auto img = make_image(
      sensor_msgs::image_encodings::TYPE_16UC1, w, h, step, std::move(data));
    cam_pub.publish(img, info);
  }

  spin_until = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < spin_until &&
    output_count.load() == 0)
  {
    helper_exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_GT(output_count.load(), 0)
    << "PointCloudXyzNode should still process valid images after malformed ones.";

  rclcpp::shutdown();
  node_spin_thread.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
