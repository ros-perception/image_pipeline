// Copyright 2008, 2019, Willow Garage, Inc., Andreas Klintberg, Joshua Whitley
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

#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <string>
#include <thread>
#include <memory>
#include <vector>

#include "image_proc/debayer.hpp"
// Until merged into OpenCV
#include "image_proc/edge_aware.hpp"

namespace image_proc
{

namespace enc = sensor_msgs::image_encodings;

DebayerNode::DebayerNode(const rclcpp::NodeOptions & options)
: Node("DebayerNode", options)
{
  sub_raw_ = image_transport::create_subscription(
    this, "image_raw",
    std::bind(
      &DebayerNode::imageCb, this,
      std::placeholders::_1), "raw");

  pub_mono_ = image_transport::create_publisher(this, "image_mono");
  pub_color_ = image_transport::create_publisher(this, "image_color");
  debayer_ = this->declare_parameter("debayer", 3);
}

void DebayerNode::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & raw_msg)
{
  int bit_depth = enc::bitDepth(raw_msg->encoding);
  // TODO(someone): Fix as soon as bitDepth fixes it
  if (raw_msg->encoding == enc::YUV422) {
    bit_depth = 8;
  }

  // First publish to mono if needed
  if (pub_mono_.getNumSubscribers()) {
    if (enc::isMono(raw_msg->encoding)) {
      pub_mono_.publish(raw_msg);
    } else {
      if ((bit_depth != 8) && (bit_depth != 16)) {
        RCLCPP_WARN(
          this->get_logger(),
          "Raw image data from topic '%s' has unsupported depth: %d",
          sub_raw_.getTopic().c_str(), bit_depth);
      } else {
        // Use cv_bridge to convert to Mono. If a type is not supported,
        // it will error out there
        sensor_msgs::msg::Image::SharedPtr gray_msg;

        try {
          if (bit_depth == 8) {
            gray_msg = cv_bridge::toCvCopy(raw_msg, enc::MONO8)->toImageMsg();
          } else {
            gray_msg = cv_bridge::toCvCopy(raw_msg, enc::MONO16)->toImageMsg();
          }

          pub_mono_.publish(gray_msg);
        } catch (cv_bridge::Exception & e) {
          RCLCPP_WARN(this->get_logger(), "cv_bridge conversion error: '%s'", e.what());
        }
      }
    }
  }

  // Next, publish to color
  if (!pub_color_.getNumSubscribers()) {
    return;
  }

  if (enc::isMono(raw_msg->encoding)) {
    // For monochrome, no processing needed!
    pub_color_.publish(raw_msg);

    // Warn if the user asked for color
    RCLCPP_WARN(
      this->get_logger(),
      "Color topic '%s' requested, but raw image data from topic '%s' is grayscale",
      pub_color_.getTopic().c_str(), sub_raw_.getTopic().c_str());
  } else if (enc::isColor(raw_msg->encoding)) {
    pub_color_.publish(raw_msg);
  } else if (enc::isBayer(raw_msg->encoding)) {
    int type = bit_depth == 8 ? CV_8U : CV_16U;
    const cv::Mat bayer(
      raw_msg->height, raw_msg->width, CV_MAKETYPE(type, 1),
      const_cast<uint8_t *>(&raw_msg->data[0]), raw_msg->step);

    sensor_msgs::msg::Image::SharedPtr color_msg =
      std::make_shared<sensor_msgs::msg::Image>();
    color_msg->header = raw_msg->header;
    color_msg->height = raw_msg->height;
    color_msg->width = raw_msg->width;
    color_msg->encoding = bit_depth == 8 ? enc::BGR8 : enc::BGR16;
    color_msg->step = color_msg->width * 3 * (bit_depth / 8);
    color_msg->data.resize(color_msg->height * color_msg->step);

    cv::Mat color(
      color_msg->height, color_msg->width, CV_MAKETYPE(type, 3),
      &color_msg->data[0], color_msg->step);

    int algorithm;
    // std::loc_guard<std::recursive_mutex> loc(config_mutex_)
    algorithm = debayer_;

    if (algorithm == debayer_edgeaware_ ||
      algorithm == debayer_edgeaware_weighted_)
    {
      // These algorithms are not in OpenCV yet
      if (raw_msg->encoding != enc::BAYER_GRBG8) {
        RCLCPP_WARN(
          this->get_logger(), "Edge aware algorithms currently only support GRBG8 Bayer. "
          "Falling back to bilinear interpolation.");
        algorithm = debayer_bilinear_;
      } else {
        if (algorithm == debayer_edgeaware_) {
          debayerEdgeAware(bayer, color);
        } else {
          debayerEdgeAwareWeighted(bayer, color);
        }
      }
    }

    if (algorithm == debayer_bilinear_ || algorithm == debayer_vng_) {
      int code = -1;

      if (raw_msg->encoding == enc::BAYER_RGGB8 || raw_msg->encoding == enc::BAYER_RGGB16) {
        code = cv::COLOR_BayerBG2BGR;
      } else if (raw_msg->encoding == enc::BAYER_BGGR8 || raw_msg->encoding == enc::BAYER_BGGR16) {
        code = cv::COLOR_BayerRG2BGR;
      } else if (raw_msg->encoding == enc::BAYER_GBRG8 || raw_msg->encoding == enc::BAYER_GBRG16) {
        code = cv::COLOR_BayerGR2BGR;
      } else if (raw_msg->encoding == enc::BAYER_GRBG8 || raw_msg->encoding == enc::BAYER_GRBG16) {
        code = cv::COLOR_BayerGB2BGR;
      }

      if (algorithm == debayer_vng_) {
        code += cv::COLOR_BayerBG2BGR_VNG - cv::COLOR_BayerBG2BGR;
      }

      cv::cvtColor(bayer, color, code);
    }

    pub_color_.publish(color_msg);
  } else if (raw_msg->encoding == enc::YUV422 || raw_msg->encoding == enc::YUV422_YUY2) {
    // Use cv_bridge to convert to BGR8
    sensor_msgs::msg::Image::SharedPtr color_msg;

    try {
      color_msg = cv_bridge::toCvCopy(raw_msg, enc::BGR8)->toImageMsg();
      pub_color_.publish(color_msg);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "cv_bridge conversion error: '%s'", e.what());
    }
  } else if (raw_msg->encoding == enc::TYPE_8UC3) {
    // 8UC3 does not specify a color encoding. Is it BGR, RGB, HSV, XYZ, LUV...?
    RCLCPP_WARN(
      this->get_logger(),
      "Raw image topic '%s' has ambiguous encoding '8UC3'. The "
      "source should set the encoding to 'bgr8' or 'rgb8'.",
      sub_raw_.getTopic().c_str());
  } else {
    RCLCPP_WARN(
      this->get_logger(), "Raw image topic '%s' has unsupported encoding '%s'",
      sub_raw_.getTopic().c_str(), raw_msg->encoding.c_str());
  }
}

}  // namespace image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_proc::DebayerNode)
