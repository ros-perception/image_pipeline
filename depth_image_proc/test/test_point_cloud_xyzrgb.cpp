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
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "depth_image_proc/point_cloud_xyzrgb.hpp"
#include "depth_image_proc/point_cloud_xyzrgb_radial.hpp"

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

// Spins a node on its own thread and stops it on destruction.
class SpinThread
{
public:
  explicit SpinThread(rclcpp::Node::SharedPtr node)
  : thread_([node]() {rclcpp::spin(node);})
  {
  }

  ~SpinThread()
  {
    rclcpp::shutdown();
    thread_.join();
  }

private:
  std::thread thread_;
};

// Sends `node` a depth image of depth_w x depth_h at 1 m and an RGB8 image of
// rgb_w x rgb_h in the single color (10, 20, 30), with camera info for the RGB
// image, until it publishes a point cloud on ns + "/points". Returns that
// cloud, or nullptr if none arrives. Shuts rclcpp down before returning.
sensor_msgs::msg::PointCloud2::ConstSharedPtr publish_until_cloud(
  rclcpp::Node::SharedPtr node, const std::string & ns,
  const std::string & depth_topic, const std::string & rgb_topic,
  uint32_t depth_w, uint32_t depth_h, uint32_t rgb_w, uint32_t rgb_h)
{
  SpinThread spin_thread(node);

  auto helper_node = rclcpp::Node::make_shared(node->get_name() + std::string("_helper"));
  rclcpp::executors::SingleThreadedExecutor helper_exec;
  helper_exec.add_node(helper_node);

  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  auto out_sub = helper_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    ns + "/points", rclcpp::SensorDataQoS(),
    [&cloud](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      cloud = msg;
    });

  image_transport::ImageTransport it_helper{*helper_node};
  auto depth_pub = it_helper.advertise(ns + depth_topic, 1);
  auto rgb_cam_pub = it_helper.advertiseCamera(ns + rgb_topic, 1);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while ((depth_pub.getNumSubscribers() == 0 ||
    rgb_cam_pub.getNumSubscribers() == 0) &&
    std::chrono::steady_clock::now() < deadline)
  {
    helper_exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }
  if (depth_pub.getNumSubscribers() == 0 || rgb_cam_pub.getNumSubscribers() == 0) {
    ADD_FAILURE() << "The node did not subscribe to the depth and RGB topics in time.";
    return nullptr;
  }

  auto depth = make_depth_16uc1(depth_w, depth_h, 1000);
  auto rgb = make_rgb8(rgb_w, rgb_h);
  for (size_t i = 0; i < rgb.data.size(); i += 3) {
    rgb.data[i] = 10;
    rgb.data[i + 1] = 20;
    rgb.data[i + 2] = 30;
  }
  auto info = make_camera_info(rgb_w, rgb_h);

  deadline = std::chrono::steady_clock::now() + 3s;
  while (!cloud && std::chrono::steady_clock::now() < deadline) {
    const auto stamp = helper_node->now();
    depth.header.stamp = stamp;
    rgb.header.stamp = stamp;
    info.header.stamp = stamp;
    depth_pub.publish(depth);
    rgb_cam_pub.publish(rgb, info);
    for (int i = 0; i < 10 && !cloud; ++i) {
      helper_exec.spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }
  return cloud;
}

// Checks a cloud from publish_until_cloud() against the camera info of the
// rgb_w x rgb_h image scaled by depth_w / rgb_w, the depth resolution.
// `radial` selects the radial node's output, where the depth is the distance
// along the unit ray through the pixel.
void expect_scaled_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud, uint32_t depth_w, uint32_t depth_h,
  uint32_t rgb_w, uint32_t rgb_h, bool radial)
{
  ASSERT_EQ(cloud.width, depth_w);
  ASSERT_EQ(cloud.height, depth_h);

  // make_camera_info(): fx = fy = 100, cx = width / 2, cy = height / 2.
  const double ratio = static_cast<double>(depth_w) / rgb_w;
  const double f = 100.0 * ratio;
  const double cx = rgb_w / 2.0 * ratio;
  const double cy = rgb_h / 2.0 * ratio;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(cloud, "r");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(cloud, "g");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(cloud, "b");
  for (uint32_t v = 0; v < depth_h; ++v) {
    for (uint32_t u = 0; u < depth_w;
      ++u, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
    {
      const double ray_x = (u - cx) / f;
      const double ray_y = (v - cy) / f;
      const double scale = radial ? 1.0 / std::sqrt(ray_x * ray_x + ray_y * ray_y + 1.0) : 1.0;
      EXPECT_NEAR(*iter_x, ray_x * scale, 1e-6) << "u=" << u << " v=" << v;
      EXPECT_NEAR(*iter_y, ray_y * scale, 1e-6) << "u=" << u << " v=" << v;
      EXPECT_NEAR(*iter_z, scale, 1e-6) << "u=" << u << " v=" << v;
      EXPECT_EQ(*iter_r, 10) << "u=" << u << " v=" << v;
      EXPECT_EQ(*iter_g, 20) << "u=" << u << " v=" << v;
      EXPECT_EQ(*iter_b, 30) << "u=" << u << " v=" << v;
    }
  }
}

}  // namespace

// Per-test init/shutdown so each test owns the rclcpp global context. Without
// this, the first test that calls rclcpp::shutdown() leaves any subsequent
// test unable to create publishers/subscribers (guard-condition errors).
class PointCloudXyzrgbNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

// Regression test: when a synchronized depth/rgb/camera_info triplet has
// incompatible aspect ratios, the resize path used to call
//   cv::Mat::rowRange(0, depth_height / ratio)
// with an end row past the RGB image's actual rows. OpenCV would throw
// cv::Exception, which was not caught and aborted the component container.
// The fix validates the derived crop range up-front and skips the frame.
TEST_F(PointCloudXyzrgbNodeTest, MismatchedAspectRatioDoesNotCrash)
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

// Regression test for issue #1149: when a synchronized depth/rgb/camera_info
// triplet has a depth message whose declared dimensions (width/height/step)
// describe a larger image than the actual data payload, convertDepth<>()
// used to iterate past the end of depth_msg->data and trigger a heap-buffer-
// overflow read (caught by ASan in the original report). The fix validates
// the depth buffer against its declared dimensions and skips the frame.
TEST_F(PointCloudXyzrgbNodeTest, OversizedDepthMetadataDoesNotCrash)
{
  const std::string ns = "/point_cloud_xyzrgb_oversize_test";
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

  auto helper_node = rclcpp::Node::make_shared("point_cloud_xyzrgb_oversize_helper");
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
  auto rgb_cam_pub = it_helper.advertiseCamera(topic_rgb, 1);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while ((depth_pub.getNumSubscribers() == 0 ||
    rgb_cam_pub.getNumSubscribers() == 0) &&
    std::chrono::steady_clock::now() < deadline)
  {
    helper_exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_GT(depth_pub.getNumSubscribers(), 0u);
  ASSERT_GT(rgb_cam_pub.getNumSubscribers(), 0u);

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

  // Depth: metadata claims 640x480 but the payload only contains 320x240
  // worth of bytes -- the lagging-metadata window described in issue #1149.
  // RGB matches the declared depth dimensions so the resize branch is not
  // taken and the depth-side conversion is exercised directly.
  {
    const uint32_t declared_w = 640;
    const uint32_t declared_h = 480;
    const uint32_t declared_step = declared_w * sizeof(uint16_t);
    const uint32_t actual_w = 320;
    const uint32_t actual_h = 240;
    std::vector<uint8_t> short_data(
      static_cast<size_t>(actual_w) * actual_h * sizeof(uint16_t), 0);
    auto depth = make_image(
      sensor_msgs::image_encodings::TYPE_16UC1,
      declared_w, declared_h, declared_step, std::move(short_data));
    auto rgb = make_rgb8(declared_w, declared_h);
    auto info = make_camera_info(declared_w, declared_h);
    publish_triplet(depth, rgb, info);
  }

  // Also exercise the unreasonable-dimensions guard.
  {
    auto depth = make_image(
      sensor_msgs::image_encodings::TYPE_16UC1, 0, 0, 0, {});
    auto rgb = make_rgb8(8, 4);
    auto info = make_camera_info(8, 4);
    publish_triplet(depth, rgb, info);
  }

  auto spin_until = std::chrono::steady_clock::now() + 1500ms;
  while (std::chrono::steady_clock::now() < spin_until) {
    helper_exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(output_count.load(), 0)
    << "Inconsistent depth metadata/payload must not produce a point cloud.";

  // Final valid triplet to confirm the node is still alive after rejecting
  // the malformed input.
  {
    const uint32_t w = 8;
    const uint32_t h = 4;
    auto depth = make_depth_16uc1(w, h, 1000);
    auto rgb = make_rgb8(w, h);
    auto info = make_camera_info(w, h);
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

// When the depth and RGB images have different sizes, the nodes scale the
// camera intrinsics and resize the RGB image to the depth resolution. The
// resized image used to be thrown away: the nodes logged "Depth resolution
// (WxH) does not match RGB resolution (WxH)" with the already-resized size
// and returned without publishing.
TEST_F(PointCloudXyzrgbNodeTest, LargerRgbIsResizedToDepthResolution)
{
  const std::string ns = "/point_cloud_xyzrgb_resize_test";
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", std::string("__ns:=") + ns});
  auto node = std::make_shared<depth_image_proc::PointCloudXyzrgbNode>(options);

  auto cloud = publish_until_cloud(
    node, ns, "/depth_registered/image_rect", "/rgb/image_rect_color", 4, 2, 8, 4);
  ASSERT_NE(cloud, nullptr) << "No point cloud was published.";
  expect_scaled_cloud(*cloud, 4, 2, 8, 4, false);
}

// The radial node also has to build its table of rays from the intrinsics
// scaled to the depth resolution, and at that size: it looks the rays up by
// depth pixel.
TEST_F(PointCloudXyzrgbNodeTest, RadialLargerRgbIsResizedToDepthResolution)
{
  const std::string ns = "/point_cloud_xyzrgb_radial_resize_test";
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", std::string("__ns:=") + ns});
  auto node = std::make_shared<depth_image_proc::PointCloudXyzrgbRadialNode>(options);

  auto cloud = publish_until_cloud(
    node, ns, "/depth/image_raw", "/rgb/image_raw", 4, 2, 8, 4);
  ASSERT_NE(cloud, nullptr) << "No point cloud was published.";
  expect_scaled_cloud(*cloud, 4, 2, 8, 4, true);
}

// With an RGB image smaller than the depth image, a ray table built at the
// RGB resolution would be read past its end.
TEST_F(PointCloudXyzrgbNodeTest, RadialSmallerRgbIsResizedToDepthResolution)
{
  const std::string ns = "/point_cloud_xyzrgb_radial_upsample_test";
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", std::string("__ns:=") + ns});
  auto node = std::make_shared<depth_image_proc::PointCloudXyzrgbRadialNode>(options);

  auto cloud = publish_until_cloud(
    node, ns, "/depth/image_raw", "/rgb/image_raw", 8, 4, 4, 2);
  ASSERT_NE(cloud, nullptr) << "No point cloud was published.";
  expect_scaled_cloud(*cloud, 8, 4, 4, 2, true);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
