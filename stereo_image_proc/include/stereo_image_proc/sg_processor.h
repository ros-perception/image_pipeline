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
#ifndef SG_STEREO_IMAGE_PROC_PROCESSOR_H
#define SG_STEREO_IMAGE_PROC_PROCESSOR_H

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

class SGStereoProcessor
{
public:
  
  SGStereoProcessor()
#if OPENCV3
  {
    block_matcher_ = cv::StereoSGBM::create();
    block_matcher_->fullDP=false;
#else
    : block_matcher_()
  {
      block_matcher_.fullDP=false;
#endif
      ROS_ERROR("Semiglobal BM constructor.");
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
#if OPENCV3
  mutable cv::Ptr<cv::StereoBM> block_matcher_; // contains scratch buffers for block matching
#else
  mutable cv::StereoSGBM block_matcher_; // contains scratch buffers for block matching
#endif
  // scratch buffers for speckle filtering
  mutable cv::Mat_<uint32_t> labels_;
  mutable cv::Mat_<uint32_t> wavefront_;
  mutable cv::Mat_<uint8_t> region_types_;
  // scratch buffer for dense point cloud
  mutable cv::Mat_<cv::Vec3f> dense_points_;
};


inline int SGStereoProcessor::getInterpolation() const
{
  return mono_processor_.interpolation_;
}

inline void SGStereoProcessor::setInterpolation(int interp)
{
  mono_processor_.interpolation_ = interp;
}

inline int SGStereoProcessor::getPreFilterSize() const
{
#if OPENCV3
  return 0
#else
  return 0;
#endif
}

inline void SGStereoProcessor::setPreFilterSize(int size)
{
#if OPENCV3
  ;
#else
  ;
#endif
}

inline int SGStereoProcessor::getPreFilterCap() const
{
#if OPENCV3
  return block_matcher_->preFilterCap;
#else
  return block_matcher_.preFilterCap;
#endif
}

inline void SGStereoProcessor::setPreFilterCap(int cap)
{
#if OPENCV3
  block_matcher_->preFilterCap = cap;
#else
  block_matcher_.preFilterCap = cap;
#endif
}

inline int SGStereoProcessor::getCorrelationWindowSize() const
{
#if OPENCV3
  return block_matcher_->SADWindowSize;
#else
  return block_matcher_.SADWindowSize;
#endif
}

inline void SGStereoProcessor::setCorrelationWindowSize(int size)
{
#if OPENCV3
  block_matcher_->SADWindowSize =size;
#else
  block_matcher_.SADWindowSize = size;
#endif
}

inline int SGStereoProcessor::getMinDisparity() const
{
#if OPENCV3
  return block_matcher_->minDisparity;
#else
  return block_matcher_.minDisparity;
#endif
}

inline void SGStereoProcessor::setMinDisparity(int min_d)
{
#if OPENCV3
  block_matcher_->minDisparity = min_d;
#else
  block_matcher_.minDisparity = min_d;
#endif
}

inline int SGStereoProcessor::getDisparityRange() const
{
#if OPENCV3
  return block_matcher_->numberOfDisparities;
#else
  return block_matcher_.numberOfDisparities;
#endif
}

inline void SGStereoProcessor::setDisparityRange(int range)
{
#if OPENCV3
  block_matcher_->numberOfDisparities = range;
#else
  block_matcher_.numberOfDisparities = range;
#endif
}

inline int SGStereoProcessor::getTextureThreshold() const
{
#if OPENCV3
  return 0;
#else
  return 0;
#endif
}

inline void SGStereoProcessor::setTextureThreshold(int threshold)
{
#if OPENCV3
  ;
#else
  ;
#endif
}

inline float SGStereoProcessor::getUniquenessRatio() const
{
#if OPENCV3
  return block_matcher_->uniquenessRatio;
#else
  return block_matcher_.uniquenessRatio;
#endif
}

inline void SGStereoProcessor::setUniquenessRatio(float ratio)
{
#if OPENCV3
  block_matcher_->uniquenessRatio = ratio;
#else
  block_matcher_.uniquenessRatio = ratio;
#endif
}

inline int SGStereoProcessor::getSpeckleSize() const
{
#if OPENCV3
  return block_matcher_->speckleWindowSize;
#else
  return block_matcher_.speckleWindowSize;
#endif
}

inline void SGStereoProcessor::setSpeckleSize(int size)
{
#if OPENCV3
  block_matcher_->speckleWindowSize = size;
#else
  block_matcher_.speckleWindowSize = size;
#endif
}

inline int SGStereoProcessor::getSpeckleRange() const
{
#if OPENCV3
  return block_matcher_->speckleRange;
#else
  return block_matcher_.speckleRange;
#endif
}

inline void SGStereoProcessor::setSpeckleRange(int range)
{
#if OPENCV3
  block_matcher_->speckleRange = range;
#else
  block_matcher_.speckleRange = range;
#endif
}

} //namespace stereo_image_proc

#endif
