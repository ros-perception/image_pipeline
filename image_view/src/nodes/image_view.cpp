/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "rclcpp/rclcpp.hpp"

#include "image_transport/image_transport.h"
#include "rcutils/cmdline_parser.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include <exception> 
#include <memory>
#include <string>
#include <iostream>

// TODO: filesystem is not a part of the libc++ in macOS yet, 
// so as long as it move to macOS, we will enable this function ASAP.
// see: https://gcc.gnu.org/onlinedocs/libstdc++/manual/using_dynamic_or_shared.html#manual.intro.using.linkage.experimental
// #include <experimental/filesystem>

rclcpp::Node::SharedPtr node;
int g_count;
cv::Mat g_last_image;
std::mutex g_image_mutex;
std::string g_window_name;
std::string transport;
std::string topic;
char buf[1024];
bool g_gui;
image_transport::Publisher g_pub;
bool g_do_dynamic_scaling;
int g_colormap;
double g_min_image_value;
double g_max_image_value;
std::string format_string;
void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  std::unique_lock<std::mutex> lock(g_image_mutex);

  // Convert to OpenCV native BGR color
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_bridge::CvtColorForDisplayOptions options;
    options.do_dynamic_scaling = g_do_dynamic_scaling;
    options.colormap = g_colormap;
    // Set min/max value for scaling to visualize depth/float image.
    if (g_min_image_value == g_max_image_value) {
      // Not specified by rosparam, then set default value.
      // Because of current sensor limitation, we use 10m as default of max range of depth
      // with consistency to the configuration in rqt_image_view.
      options.min_image_value = 0;
      if (msg->encoding == "32FC1") {
        options.max_image_value = 10;  // 10 [m]
      } else if (msg->encoding == "16UC1") {
        options.max_image_value = 10 * 1000;  // 10 * 1000 [mm]
      }
    } else {
      options.min_image_value = g_min_image_value;
      options.max_image_value = g_max_image_value;
    }
    cv_ptr = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(msg), "", options);
    g_last_image = cv_ptr->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Unable to convert '%s' image for display: '%s'",
                       msg->encoding.c_str(), e.what());
  }
  if (g_gui && !g_last_image.empty()) {
    const cv::Mat &image = g_last_image;
    cv::imshow(g_window_name, image);
  }
  if (g_pub.getNumSubscribers() > 0) {
    g_pub.publish(cv_ptr->toImageMsg());
  }
}

static void mouseCb(int event, int x, int y, int flags, void* param)
{
  if (event == cv::EVENT_LBUTTONDOWN) {
    RCLCPP_WARN_ONCE(node->get_logger(), "Left-clicking no longer saves images. Right-click instead.");
    return;
  } else if (event != cv::EVENT_RBUTTONDOWN) {
    return;
  }

  std::unique_lock<std::mutex> lock(g_image_mutex);

  const cv::Mat &image = g_last_image;

  if (image.empty()) {
    RCLCPP_WARN(node->get_logger(), "Couldn't save image, no data!");
    return;
  }
  sprintf(buf, format_string.c_str(), g_count);
  std::string filename = buf;
  if (cv::imwrite(filename, image)) {
    RCLCPP_INFO(node->get_logger(), "Saved image %s", filename.c_str());
    g_count++;
  } else {
    // std::experimental::filesystem::path full_path = 
    //   std::experimental::filesystem::system_complete(filename);
    // TODO: filesystem is not a part of the libc++ in macOS yet, so as long as it move to macOS, we will enable this function ASAP.
    RCLCPP_ERROR(node->get_logger(), "Failed to save image. Have permission to write there?: %s", filename.c_str());
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  transport = "raw";
  if (rcutils_cli_option_exist(argv, argv + argc, "--topic")){
    topic = std::string(rcutils_cli_get_option(argv, argv + argc, "--topic"));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "--transport")){
    transport = std::string(rcutils_cli_get_option(argv, argv + argc, "--transport"));
  }
  // auto topic = std::string("/camera/color/image_raw");
  node = rclcpp::Node::make_shared("image_view");
  // Default window name is the resolved topic name
  node->get_parameter_or("window_name", g_window_name, topic);
  node->get_parameter_or("gui", g_gui, true);  // gui/no_gui mode
  node->get_parameter_or("max_image_value", g_max_image_value, 0.0);
  node->get_parameter_or("min_image_value", g_min_image_value, 0.0);  
  node->get_parameter_or("colormap", g_colormap, 11);
  node->get_parameter_or("do_dynamic_scaling", g_do_dynamic_scaling, false);
  if (g_gui) {
    node->get_parameter_or("filename_format", format_string, std::string("frame%04i.jpg"));
    

    // Handle window size
    bool autosize;
    node->get_parameter_or("autosize", autosize, false);
    cv::namedWindow(g_window_name, autosize ? (CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED) : 0);
    cv::setMouseCallback(g_window_name, &mouseCb);
    if(autosize == false)
    {
      int width;
      int height;
      if(node->get_parameter("width", width) && node->get_parameter("height", height))
      {
        width = node->get_parameter("width").as_int();
        height = node->get_parameter("height").as_int();
        cv::resizeWindow(g_window_name, width, height);
      }
    }

    // Start the OpenCV window thread so we don't have to waitKey() somewhere
    cv::startWindowThread();
  }

  // Handle transport
  // priority:
  //    1. command line argument
  //    2. rosparam '~image_transport'
  RCLCPP_INFO(node->get_logger(), "Using transport \"%s\"", transport.c_str());
  RCLCPP_INFO(node->get_logger(), "Image topic \"%s\"", topic.c_str());
  image_transport::Subscriber sub;
  sub = image_transport::create_subscription(node.get(), 
    topic, &imageCb, transport);
  g_pub = image_transport::create_publisher(node.get(), "output");
  rclcpp::spin(node);

  rclcpp::shutdown();
  if (g_gui) {
    cv::destroyWindow(g_window_name);
  }
  return 0;
}