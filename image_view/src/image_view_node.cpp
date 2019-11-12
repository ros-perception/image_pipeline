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

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/format.hpp>

#include <chrono>
#include <mutex>
#include <thread>

#include "image_view/image_view_node.hpp"
#include "image_view/window_thread.hpp"

namespace image_view
{

void ThreadSafeImage::set(const cv::Mat& image)
{
  std::lock_guard<std::mutex> lock(mutex_);
  image_ = image;
  condition_.notify_one();
}

cv::Mat ThreadSafeImage::get()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return image_;
}

cv::Mat ThreadSafeImage::pop()
{
  cv::Mat image;

  {
    std::unique_lock<std::mutex> lock(mutex_);

    while (image_.empty()) {
      condition_.wait(lock);
    }

    image = image_;
    image_.release();
  }

  return image;
}

ImageViewNode::ImageViewNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("image_view_node", options)
{
  auto transport = this->declare_parameter("image_transport", "raw");
  RCLCPP_INFO(this->get_logger(), "Using transport \"%s\"", transport.c_str());

  // Default window name is the resolved topic name
  std::string topic = rclcpp::expand_topic_or_service_name("image", this->get_name(), this->get_namespace());
  window_name_ = this->declare_parameter("window_name", topic);

  if (topic == "image") {
    RCLCPP_WARN(
      this->get_logger(), "Topic 'image' has not been remapped! Typical command-line usage:\n"
      "\t$ rosrun image_view image_view image:=<image topic> [transport]");
  }

  g_gui = this->declare_parameter("gui", true);  // gui/no_gui mode

  autosize_ = this->declare_parameter("autosize", false);
  
  std::string format_string = this->declare_parameter("filename_format", std::string("frame%04i.jpg"));
  filename_format_.parse(format_string);

  // Since cv::startWindowThread() triggers a crash in cv::waitKey()
  // if OpenCV is compiled against GTK, we call cv::waitKey() from
  // the ROS event loop periodically, instead.
  /*cv::startWindowThread();*/
  gui_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), &ImageViewNode::guiCb);

  if(g_gui) {
    window_thread_ = std::thread(&ImageViewNode::windowThread, this);
  }

  image_transport::TransportHints hints(this, transport);
  pub_ = this->create_publisher<sensor_msgs::msg::Image>("output", 1);
  sub_ = image_transport::create_subscription(this, topic, std::bind(&ImageViewNode::imageCb, this, std::placeholders::_1), hints.getTransport());
}

ImageViewNode::~ImageViewNode()
{
  if (window_thread_.joinable()) {
    // TODO(jwhitleyastuff): Figure out if interruption is necessary
    // window_thread_.interrupt();
    window_thread_.join();
  }

  if (g_gui) {
    cv::destroyWindow(window_name_);
  }
}

void ImageViewNode::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // We want to scale floating point images so that they display nicely
  bool do_dynamic_scaling = (msg->encoding.find("F") != std::string::npos);

  std::lock_guard<std::mutex> lock(g_image_mutex);

  // Convert to OpenCV native BGR color
  cv_bridge::CvImageConstPtr cv_ptr;

  // Convert to OpenCV native BGR color
  try {
    cv_bridge::CvtColorForDisplayOptions options;
    options.do_dynamic_scaling = do_dynamic_scaling;
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
    RCLCPP_ERROR_EXPRESSION(
      this->get_logger(), (static_cast<int>(this->now().seconds()) % 30 == 0),
      "Unable to convert '%s' image for display: '%s'",
      msg->encoding.c_str(), e.what());
  }

  if (g_gui && !g_last_image.empty()) {
    const cv::Mat &image = g_last_image;
    cv::imshow(window_name_, image);
    cv::waitKey(1);
  }

  if (pub_->get_subscription_count() > 0) {
    pub_->publish(*(cv_ptr->toImageMsg()));
  }
}

void ImageViewNode::guiCb()
{
  // Process pending GUI events and return immediately
  cv::waitKey(1);
}

void ImageViewNode::mouseCb(int event, int x, int y, int flags, void * param)
{
  (void)x;
  (void)y;
  (void)flags;

  ImageViewNode *this_ = reinterpret_cast<ImageViewNode *>(param);

  if (event == cv::EVENT_LBUTTONDOWN) {
    RCLCPP_WARN_ONCE(this_->get_logger(), "Left-clicking no longer saves images. Right-click instead.");
    return;
  }

  if (event != cv::EVENT_RBUTTONDOWN) {
    return;
  }
  
  cv::Mat image(this_->shown_image_.get());

  if (image.empty()) {
    RCLCPP_WARN(this_->get_logger(), "Couldn't save image, no data!");
    return;
  }

  std::string filename = (this_->filename_format_ % this_->count_).str();

  if (cv::imwrite(filename, image)) {
    RCLCPP_INFO(this_->get_logger(), "Saved image %s", filename.c_str());
    this_->count_++;
  } else {
    /// @todo Show full path, ask if user has permission to write there
    RCLCPP_ERROR(this_->get_logger(), "Failed to save image.");
  }
}

void ImageViewNode::windowThread()
{
  cv::namedWindow(window_name_, autosize_ ? cv::WND_PROP_AUTOSIZE : 0);
  cv::setMouseCallback(window_name_, &ImageViewNode::mouseCb, this);

  if(!autosize_) {
    int width = this->declare_parameter("width", -1);
    int height = this->declare_parameter("height", -1);

    if (width > -1 && height > -1) {
      cv::resizeWindow(window_name_, width, height);
    }
  }

  while (rclcpp::ok()) {
    cv::Mat image(queued_image_.pop());
    cv::imshow(window_name_, image);
    cv::waitKey(1);
    shown_image_.set(image);
  }

  cv::destroyWindow(window_name_);
}

} // namespace image_view

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_view::ImageViewNode)
