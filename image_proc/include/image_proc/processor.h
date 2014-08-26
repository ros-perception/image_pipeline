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
#ifndef IMAGE_PROC_PROCESSOR_H
#define IMAGE_PROC_PROCESSOR_H

#include <opencv2/core/core.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>

namespace image_proc {

struct ImageSet
{
  std::string color_encoding;
  cv::Mat mono;
  cv::Mat rect;
  cv::Mat color;
  cv::Mat rect_color;
};

class Processor
{
public:
  Processor()
    : interpolation_(cv::INTER_LINEAR)
  {
  }
  
  int interpolation_;

  enum {
    MONO       = 1 << 0,
    RECT       = 1 << 1,
    COLOR      = 1 << 2,
    RECT_COLOR = 1 << 3,
    ALL = MONO | RECT | COLOR | RECT_COLOR
  };
  
  bool process(const sensor_msgs::ImageConstPtr& raw_image,
               const image_geometry::PinholeCameraModel& model,
               ImageSet& output, int flags = ALL) const;
};

} //namespace image_proc

#endif
