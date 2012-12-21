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
#ifndef STEREO_IMAGE_PROC_PROCESSOR_H
#define STEREO_IMAGE_PROC_PROCESSOR_H

#include <image_proc/processor.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

namespace stereo_image_proc {

struct StereoImageSet
{
  image_proc::ImageSet left;
  image_proc::ImageSet right;
  stereo_msgs::DisparityImage disparity;
  sensor_msgs::PointCloud points;
  sensor_msgs::PointCloud2 points2;
};

class StereoProcessor
{
public:
  
  StereoProcessor()
    : block_matcher_(cv::StereoBM::BASIC_PRESET)
  {
  }

  enum {
    LEFT_MONO        = 1 << 0,
    LEFT_RECT        = 1 << 1,
    LEFT_COLOR       = 1 << 2,
    LEFT_RECT_COLOR  = 1 << 3,
    RIGHT_MONO       = 1 << 4,
    RIGHT_RECT       = 1 << 5,
    RIGHT_COLOR      = 1 << 6,
    RIGHT_RECT_COLOR = 1 << 7,
    DISPARITY        = 1 << 8,
    POINT_CLOUD      = 1 << 9,
    POINT_CLOUD2     = 1 << 10,

    LEFT_ALL = LEFT_MONO | LEFT_RECT | LEFT_COLOR | LEFT_RECT_COLOR,
    RIGHT_ALL = RIGHT_MONO | RIGHT_RECT | RIGHT_COLOR | RIGHT_RECT_COLOR,
    STEREO_ALL = DISPARITY | POINT_CLOUD | POINT_CLOUD2,
    ALL = LEFT_ALL | RIGHT_ALL | STEREO_ALL
  };

  int getInterpolation() const;
  void setInterpolation(int interp);

  // Disparity pre-filtering parameters

  int getPreFilterSize() const;
  void setPreFilterSize(int size);

  int getPreFilterCap() const;
  void setPreFilterCap(int cap);

  // Disparity correlation parameters
  
  int getCorrelationWindowSize() const;
  void setCorrelationWindowSize(int size);

  int getMinDisparity() const;
  void setMinDisparity(int min_d);

  int getDisparityRange() const;
  void setDisparityRange(int range); // Number of pixels to search

  // Disparity post-filtering parameters
  
  int getTextureThreshold() const;
  void setTextureThreshold(int threshold);

  float getUniquenessRatio() const;
  void setUniquenessRatio(float ratio);

  int getSpeckleSize() const;
  void setSpeckleSize(int size);

  int getSpeckleRange() const;
  void setSpeckleRange(int range);

  // Do all the work!
  bool process(const sensor_msgs::ImageConstPtr& left_raw,
               const sensor_msgs::ImageConstPtr& right_raw,
               const image_geometry::StereoCameraModel& model,
               StereoImageSet& output, int flags) const;

  void processDisparity(const cv::Mat& left_rect, const cv::Mat& right_rect,
                        const image_geometry::StereoCameraModel& model,
                        stereo_msgs::DisparityImage& disparity) const;

  void processPoints(const stereo_msgs::DisparityImage& disparity,
                     const cv::Mat& color, const std::string& encoding,
                     const image_geometry::StereoCameraModel& model,
                     sensor_msgs::PointCloud& points) const;
  void processPoints2(const stereo_msgs::DisparityImage& disparity,
                      const cv::Mat& color, const std::string& encoding,
                      const image_geometry::StereoCameraModel& model,
                      sensor_msgs::PointCloud2& points) const;

private:
  image_proc::Processor mono_processor_;
  
  mutable cv::Mat_<int16_t> disparity16_; // scratch buffer for 16-bit signed disparity image
  mutable cv::StereoBM block_matcher_; // contains scratch buffers for block matching
  // scratch buffers for speckle filtering
  mutable cv::Mat_<uint32_t> labels_;
  mutable cv::Mat_<uint32_t> wavefront_;
  mutable cv::Mat_<uint8_t> region_types_;
  // scratch buffer for dense point cloud
  mutable cv::Mat_<cv::Vec3f> dense_points_;
};


inline int StereoProcessor::getInterpolation() const
{
  return mono_processor_.interpolation_;
}

inline void StereoProcessor::setInterpolation(int interp)
{
  mono_processor_.interpolation_ = interp;
}

inline int StereoProcessor::getPreFilterSize() const
{
  return block_matcher_.state->preFilterSize;
}

inline void StereoProcessor::setPreFilterSize(int size)
{
  block_matcher_.state->preFilterSize = size;
}

inline int StereoProcessor::getPreFilterCap() const
{
  return block_matcher_.state->preFilterCap;
}

inline void StereoProcessor::setPreFilterCap(int cap)
{
  block_matcher_.state->preFilterCap = cap;
}

inline int StereoProcessor::getCorrelationWindowSize() const
{
  return block_matcher_.state->SADWindowSize;
}

inline void StereoProcessor::setCorrelationWindowSize(int size)
{
  block_matcher_.state->SADWindowSize = size;
}

inline int StereoProcessor::getMinDisparity() const
{
  return block_matcher_.state->minDisparity;
}

inline void StereoProcessor::setMinDisparity(int min_d)
{
  block_matcher_.state->minDisparity = min_d;
}

inline int StereoProcessor::getDisparityRange() const
{
  return block_matcher_.state->numberOfDisparities;
}

inline void StereoProcessor::setDisparityRange(int range)
{
  block_matcher_.state->numberOfDisparities = range;
}

inline int StereoProcessor::getTextureThreshold() const
{
  return block_matcher_.state->textureThreshold;
}

inline void StereoProcessor::setTextureThreshold(int threshold)
{
  block_matcher_.state->textureThreshold = threshold;
}

inline float StereoProcessor::getUniquenessRatio() const
{
  return block_matcher_.state->uniquenessRatio;
}

inline void StereoProcessor::setUniquenessRatio(float ratio)
{
  block_matcher_.state->uniquenessRatio = ratio;
}

inline int StereoProcessor::getSpeckleSize() const
{
  return block_matcher_.state->speckleWindowSize;
}

inline void StereoProcessor::setSpeckleSize(int size)
{
  block_matcher_.state->speckleWindowSize = size;
}

inline int StereoProcessor::getSpeckleRange() const
{
  return block_matcher_.state->speckleRange;
}

inline void StereoProcessor::setSpeckleRange(int range)
{
  block_matcher_.state->speckleRange = range;
}

} //namespace stereo_image_proc

#endif
