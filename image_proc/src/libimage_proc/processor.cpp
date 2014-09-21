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
#include "image_proc/processor.h"
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>

namespace image_proc {

namespace enc = sensor_msgs::image_encodings;

bool Processor::process(const sensor_msgs::ImageConstPtr& raw_image,
                        const image_geometry::PinholeCameraModel& model,
                        ImageSet& output, int flags) const
{
  static const int MONO_EITHER = MONO | RECT;
  static const int COLOR_EITHER = COLOR | RECT_COLOR;
  if (!(flags & ALL)) return true;
  
  // Check if raw_image is color
  const std::string& raw_encoding = raw_image->encoding;
  int raw_type = CV_8UC1;
  if (raw_encoding == enc::BGR8 || raw_encoding == enc::RGB8) {
    raw_type = CV_8UC3;
    output.color_encoding = raw_encoding;
  }
  // Construct cv::Mat pointing to raw_image data
  const cv::Mat raw(raw_image->height, raw_image->width, raw_type,
                    const_cast<uint8_t*>(&raw_image->data[0]), raw_image->step);

  ///////////////////////////////////////////////////////
  // Construct colorized (unrectified) images from raw //
  ///////////////////////////////////////////////////////
  
  // Bayer case
  if (raw_encoding.find("bayer") != std::string::npos) {
    // Convert to color BGR
    /// @todo Faster to convert directly to mono when color is not requested, but OpenCV doesn't support
    int code = 0;
    if (raw_encoding == enc::BAYER_RGGB8)
#if OPENCV3
      code = cv::COLOR_BayerBG2BGR;
#else
      code = CV_BayerBG2BGR;
#endif
    else if (raw_encoding == enc::BAYER_BGGR8)
#if OPENCV3
      code = cv::COLOR_BayerRG2BGR;
#else
      code = CV_BayerRG2BGR;
#endif
    else if (raw_encoding == enc::BAYER_GBRG8)
#if OPENCV3
      code = cv::COLOR_BayerGR2BGR;
#else
      code = CV_BayerGR2BGR;
#endif
    else if (raw_encoding == enc::BAYER_GRBG8)
#if OPENCV3
      code = cv::COLOR_BayerGB2BGR;
#else
      code = CV_BayerGB2BGR;
#endif
    else {
      ROS_ERROR("[image_proc] Unsupported encoding '%s'", raw_encoding.c_str());
      return false;
    }
    cv::cvtColor(raw, output.color, code);
    output.color_encoding = enc::BGR8;
    
    if (flags & MONO_EITHER)
#if OPENCV3
      cv::cvtColor(output.color, output.mono, cv::COLOR_BGR2GRAY);
#else
      cv::cvtColor(output.color, output.mono, CV_BGR2GRAY);
#endif
  }
  // Color case
  else if (raw_type == CV_8UC3) {
    output.color = raw;
    if (flags & MONO_EITHER) {
#if OPENCV3
      int code = (raw_encoding == enc::BGR8) ? cv::COLOR_BGR2GRAY : cv::COLOR_RGB2GRAY;
#else
      int code = (raw_encoding == enc::BGR8) ? CV_BGR2GRAY : CV_RGB2GRAY;
#endif
      cv::cvtColor(output.color, output.mono, code);
    }
  }
  // Mono case
  else if (raw_encoding == enc::MONO8) {
    output.mono = raw;
    if (flags & COLOR_EITHER) {
      output.color_encoding = enc::MONO8;
      output.color = raw;
    }
  }
  // 8UC3 does not specify a color encoding. Is it BGR, RGB, HSV, XYZ, LUV...?
  else if (raw_encoding == enc::TYPE_8UC3) {
    ROS_ERROR("[image_proc] Ambiguous encoding '8UC3'. The camera driver "
              "should set the encoding to 'bgr8' or 'rgb8'.");
    return false;
  }
  // Something else we can't handle
  else {
    ROS_ERROR("[image_proc] Unsupported encoding '%s'", raw_encoding.c_str());
    return false;
  }

  //////////////////////////////////////////////////////
  // Construct rectified images from colorized images //
  //////////////////////////////////////////////////////
  
  /// @todo If no distortion, could just point to the colorized data. But copy is
  /// already way faster than remap.
  if (flags & RECT)
    model.rectifyImage(output.mono, output.rect, interpolation_);
  if (flags & RECT_COLOR)
    model.rectifyImage(output.color, output.rect_color, interpolation_);

  return true;
}

} //namespace image_proc
