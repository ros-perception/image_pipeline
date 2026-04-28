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

#include "depth_image_proc/point_cloud_xyzrgb.hpp"

using namespace std::chrono_literals;

namespace
{

sensor_msgs::msg::CameraInfo make_camera_info(uint32_t width, uint32_t height)
{
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = "camera";
  info.width = width;
  info.height = height;
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

sensor_msgs::msg::Image make_depth_16uc1(uint32_t width, uint32_t height, uint16_t value)
{
  const uint32_t step = width * static_cast<uint32_t>(sizeof(uint16_t));
  std::vector<uint8_t> data(static_cast<size_t>(step) * height, 0);
  uint16_t * p = reinterpret_cast<uint16_t *>(data.data());
  for (uint32_t i = 0; i < width * height; ++i) {
    p[i] = value;
  }
  return make_image(
    sensor_msgs::image_encodings::TYPE_16UC1, width, height, step, std::move(data));
}

sensor_msgs::msg::Image make_rgb8(uint32_t width, uint32_t height)
{
  const uint32_t step = width * 3;
  std::vector<uint8_t> data(static_cast<size_t>(step) * height, 128);
  return make_image(
    sensor_msgs::image_encodings::RGB8, width, height, step, std::move(data));
}

}  // namespace

// Regression test: when a synchronized depth/rgb/camera_info triplet has
// incompatible aspect ratios, the resize path used to call
//   cv::Mat::rowRange(0, depth_height / ratio)
// with an end row past the RGB image's actual rows. OpenCV would throw
// cv::Exception, which was not caught and aborted the component container.
// The fix validates the derived crop range up-front and skips the frame.
TEST(PointCloudXyzrgbNodeTest, MismatchedAspectRatioDoesNotCrash)
{
  const std::string ns = "/point_cloud_xyzrgb_test";
  const std::string topic_depth = ns + "/depth_registered/image_rect";
  const std::string topic_rgb = ns + "/rgb/image_rect_color";
  const std::string topic_out = ns + "/points";

  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "-r", std::string("__ns:=") + ns});
  auto node = std::make_shared<depth_image_proc::PointCloudXyzrgbNode>(options);

  std::thread node_spin_thread(
    [node]() {
      rclcpp::spin(node);
    });

  auto helper_node = rclcpp::Node::make_shared("point_cloud_xyzrgb_test_helper");
  rclcpp::executors::SingleThreadedExecutor helper_exec;
  helper_exec.add_node(helper_node);

  std::atomic<int> output_count{0};
  auto out_sub = helper_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_out, rclcpp::SensorDataQoS(),
    [&output_count](sensor_msgs::msg::PointCloud2::ConstSharedPtr) {
      output_count.fetch_add(1);
    });

  image_transport::ImageTransport it_helper{*helper_node};
  auto depth_pub = it_helper.advertise(topic_depth, 1);
  // advertiseCamera publishes both the rgb image and rgb/camera_info using
  // the same timestamp, matching what PointCloudXyzrgbNode subscribes to.
  auto rgb_cam_pub = it_helper.advertiseCamera(topic_rgb, 1);

  // Wait for the lazy subscriptions on the node side to come up.
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while ((depth_pub.getNumSubscribers() == 0 ||
    rgb_cam_pub.getNumSubscribers() == 0) &&
    std::chrono::steady_clock::now() < deadline)
  {
    helper_exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_GT(depth_pub.getNumSubscribers(), 0u)
    << "PointCloudXyzrgbNode did not subscribe to depth topic in time.";
  ASSERT_GT(rgb_cam_pub.getNumSubscribers(), 0u)
    << "PointCloudXyzrgbNode did not subscribe to rgb topic in time.";

  auto publish_triplet =
    [&](const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::Image & rgb,
    const sensor_msgs::msg::CameraInfo & info)
    {
      const auto stamp = helper_node->now();
      auto depth_with_stamp = depth;
      depth_with_stamp.header.stamp = stamp;
      auto rgb_with_stamp = rgb;
      rgb_with_stamp.header.stamp = stamp;
      auto info_with_stamp = info;
      info_with_stamp.header.stamp = stamp;
      info_with_stamp.header.frame_id = rgb_with_stamp.header.frame_id;
      depth_pub.publish(depth_with_stamp);
      rgb_cam_pub.publish(rgb_with_stamp, info_with_stamp);
    };

  // Case 1: incompatible aspect ratios (the reported reproducer).
  // depth 640x480, rgb 1280x100. ratio = 0.5, depth_height/ratio = 960 > 100.
  {
    auto depth = make_depth_16uc1(640, 480, 1000);
    auto rgb = make_rgb8(1280, 100);
    auto info = make_camera_info(1280, 100);
    publish_triplet(depth, rgb, info);
  }

  // Case 2: rgb height of zero -- another invalid input that would otherwise
  // produce a degenerate rowRange.
  {
    auto depth = make_depth_16uc1(640, 480, 1000);
    auto rgb = make_image(sensor_msgs::image_encodings::RGB8, 640, 0, 0, {});
    auto info = make_camera_info(640, 0);
    publish_triplet(depth, rgb, info);
  }

  // Pump the executor a bit; malformed inputs must not crash and must not
  // produce a point cloud.
  auto spin_until = std::chrono::steady_clock::now() + 1500ms;
  while (std::chrono::steady_clock::now() < spin_until) {
    helper_exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(output_count.load(), 0)
    << "Mismatched-aspect inputs should not produce a point cloud.";

  // Case 3: a valid matching depth/rgb pair. The node should still be alive
  // and able to publish a point cloud.
  {
    const uint32_t w = 8;
    const uint32_t h = 4;
    auto depth = make_depth_16uc1(w, h, 1000);
    auto rgb = make_rgb8(w, h);
    auto info = make_camera_info(w, h);
    // Publish a few times to give approximate-time sync something to lock on.
    for (int i = 0; i < 5; ++i) {
      publish_triplet(depth, rgb, info);
      std::this_thread::sleep_for(30ms);
      helper_exec.spin_some();
    }
  }

  spin_until = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < spin_until &&
    output_count.load() == 0)
  {
    helper_exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_GT(output_count.load(), 0)
    << "PointCloudXyzrgbNode should still process valid inputs after malformed ones.";

  rclcpp::shutdown();
  node_spin_thread.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
