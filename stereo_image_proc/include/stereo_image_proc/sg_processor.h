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
    block_matcher_->SADWindowSize = 5;
    block_matcher_->numberOfDisparities = 192;
    block_matcher_->preFilterCap = 4;
    block_matcher_->minDisparity = -64;
    block_matcher_->uniquenessRatio = 1;
    block_matcher_->speckleWindowSize = 150;
    block_matcher_->speckleRange = 2;
    block_matcher_->disp12MaxDiff = 10;
    block_matcher_->fullDP = false;
    block_matcher_->P1 = 600;
    block_matcher_->P2 = 2400;

#else
    : block_matcher_()
  {
      block_matcher_.SADWindowSize = 5;
      block_matcher_.numberOfDisparities = 192;
      block_matcher_.preFilterCap = 4;
      block_matcher_.minDisparity = -64;
      block_matcher_.uniquenessRatio = 1;
      block_matcher_.speckleWindowSize = 150;
      block_matcher_.speckleRange = 2;
      block_matcher_.disp12MaxDiff = 10;
      block_matcher_.fullDP = false;
      block_matcher_.P1 = 600;
      block_matcher_.P2 = 2400;
#endif

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

  // Semi-global specific parameters

  int getSmoothnessP1() const;
  void setSmoothnessP1(int P1);

  int getSmoothnessP2() const;
  void setSmoothnessP2(int P2);

  int getMaxDiff() const;
  void setMaxDiff(int maxDiff);

  int getFullDP() const;
  void setFullDP(bool fullDP);

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
  mutable cv::Ptr<cv::StereoSGBM> block_matcher_; // contains scratch buffers for block matching
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



inline int SGStereoProcessor::getSmoothnessP1() const
{
#if OPENCV3
  return block_matcher_->P1;
#else
  return block_matcher_.P1;
#endif
}

inline void SGStereoProcessor::setSmoothnessP1(int P1)
{
#if OPENCV3
  block_matcher_->P1 = P1;
#else
  block_matcher_.P1 = P1;
#endif
}

inline int SGStereoProcessor::getSmoothnessP2() const
{
#if OPENCV3
  return block_matcher_->P2;
#else
  return block_matcher_.P2;
#endif
}

inline void SGStereoProcessor::setSmoothnessP2(int P2)
{
#if OPENCV3
  block_matcher_->P2 = P2;
#else
  block_matcher_.P2 = P2;
#endif
}

inline int SGStereoProcessor::getMaxDiff() const
{
#if OPENCV3
  return block_matcher_->disp12MaxDiff;
#else
  return block_matcher_.disp12MaxDiff;
#endif
}

inline void SGStereoProcessor::setMaxDiff(int maxDiff)
{
#if OPENCV3
  block_matcher_->disp12MaxDiff = maxDiff;
#else
  block_matcher_.disp12MaxDiff = maxDiff;
#endif
}

inline int SGStereoProcessor::getFullDP() const
{
#if OPENCV3
  return block_matcher_->fullDP;
#else
  return block_matcher_.fullDP;
#endif
}

inline void SGStereoProcessor::setFullDP(bool fullDP)
{
#if OPENCV3
  block_matcher_->fullDP = fullDP;
#else
  block_matcher_.fullDP = fullDP;
#endif
}

} //namespace stereo_image_proc

//PARAMETER DESCRIPTION
/*
    * minDisparity – Minimum possible disparity value. Normally, it is zero but sometimes rectification algorithms can shift images, so this parameter needs to be adjusted accordingly.
    * numDisparities – Maximum disparity minus minimum disparity. The value is always greater than zero. In the current implementation, this parameter must be divisible by 16.
    * SADWindowSize – Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
    * P1 – The first parameter controlling the disparity smoothness. See below.
    * P2 – The second parameter controlling the disparity smoothness. The larger the values are, the smoother the disparity is. P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels. P2 is the penalty on the disparity change by more than 1 between neighbor pixels. The algorithm requires P2 > P1 . See stereo_match.cpp sample where some reasonably good P1 and P2 values are shown (like 8*number_of_image_channels*SADWindowSize*SADWindowSize and 32*number_of_image_channels*SADWindowSize*SADWindowSize , respectively).
    * disp12MaxDiff – Maximum allowed difference (in integer pixel units) in the left-right disparity check. Set it to a non-positive value to disable the check.
    * preFilterCap – Truncation value for the prefiltered image pixels. The algorithm first computes x-derivative at each pixel and clips its value by [-preFilterCap, preFilterCap] interval. The result values are passed to the Birchfield-Tomasi pixel cost function.
    * uniquenessRatio – Margin in percentage by which the best (minimum) computed cost function value should “win” the second best value to consider the found match correct. Normally, a value within the 5-15 range is good enough.
    * speckleWindowSize – Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
    * speckleRange – Maximum disparity variation within each connected component. If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16. Normally, 1 or 2 is good enough.
    * fullDP – Set it to true to run the full-scale two-pass dynamic programming algorithm. It will consume O(W*H*numDisparities) bytes, which is large for 640x480 stereo and huge for HD-size pictures. By default, it is set to false .
*/

#endif
