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

#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "rcutils/cmdline_parser.h"
#include <thread>
#include <memory>
#include <string>

using std::placeholders::_1;
class ExtractImages
{
private:
  rclcpp::Node::SharedPtr node;
  image_transport::Subscriber sub_;
  sensor_msgs::msg::Image::ConstSharedPtr last_msg_;
  std::mutex image_mutex_;
  
  char buf[1024];
  std::string format_string;
  std::string window_name_;
  int count_;
  rclcpp::Time _time;
  rclcpp::Time sec_per_frame_;

#if defined(_VIDEO)
  CvVideoWriter* video_writer;
#endif //_VIDEO

public:
  ExtractImages(const std::string& topic,
                const std::string& transport)
    : format_string(""), count_(0), _time(rclcpp::Clock().now())
  {
    // auto clock_ = rclcpp::Clock();
    node = rclcpp::Node::make_shared("extract_images");
    std::string format_string;
    node->get_parameter_or("filename_format", format_string, std::string("frame%04i.jpg"));
    double sec_per_frame;
    node->get_parameter_or("sec_per_frame", sec_per_frame, 0.1);
    sec_per_frame_ = rclcpp::Time(sec_per_frame);
    sub_ = image_transport::create_subscription(
      node.get(), topic, std::bind(&ExtractImages::image_cb, this, _1), transport);

#if defined(_VIDEO)
    video_writer = 0;
#endif

    RCLCPP_INFO(node->get_logger(), "Initialized sec per frame to %f", sec_per_frame_);
    rclcpp::spin(node);
  }

  ~ExtractImages()
  {
  }

  void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    std::lock_guard<std::mutex> guard(image_mutex_);

    // Hang on to message pointer for sake of mouse_cb
    last_msg_ = msg;

    // May want to view raw bayer data
    // NB: This is hacky, but should be OK since we have only one image CB.
    if (msg->encoding.find("bayer") != std::string::npos)
      std::const_pointer_cast<sensor_msgs::msg::Image>(msg)->encoding = "mono8";

    cv::Mat image;
    try
    {
      image = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch(cv_bridge::Exception)
    {
      RCLCPP_ERROR(node->get_logger(), "Unable to convert %s image to bgr8", msg->encoding.c_str());
    }

    auto delay = rclcpp::Clock().now() -_time;
    if(delay.seconds() >= sec_per_frame_.seconds())
    {
      _time = rclcpp::Clock().now();

      if (!image.empty()) {
        sprintf(buf, format_string.c_str(), count_);
        std::string filename =buf;

#if !defined(_VIDEO)
        cv::imwrite(filename, image);
#else
        if(!video_writer)
        {
            video_writer = cvCreateVideoWriter("video.avi", CV_FOURCC('M','J','P','G'),
                int(1.0/sec_per_frame_.seconds(), cvSize(image->width, image->height));
        }

        cvWriteFrame(video_writer, image);
#endif // _VIDEO

        RCLCPP_INFO(node->get_logger(), "Saved image %s", filename.c_str());
        count_++;
      } else {
        RCLCPP_WARN(node->get_logger(), "Couldn't save image, no data!");
      }
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::string topic, transport;
  transport = "raw";
  if (rcutils_cli_option_exist(argv, argv + argc, "--topic")){
    topic = std::string(rcutils_cli_get_option(argv, argv + argc, "--topic"));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "--transport")){
    transport = std::string(rcutils_cli_get_option(argv, argv + argc, "--transport"));
  }
  ExtractImages view(topic, transport);
  return 0;
}