// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
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
#ifndef STEREO_IMAGE_PROC__STEREO_PROCESSOR_HPP_
#define STEREO_IMAGE_PROC__STEREO_PROCESSOR_HPP_

#include <image_geometry/stereo_camera_model.h>
#include <image_proc/processor.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include <string>

namespace stereo_image_proc
{

struct StereoImageSet
{
  image_proc::ImageSet left;
  image_proc::ImageSet right;
  stereo_msgs::msg::DisparityImage disparity;
  sensor_msgs::msg::PointCloud points;
  sensor_msgs::msg::PointCloud2 points2;
};

class StereoProcessor
{
public:
  StereoProcessor()
  {
    block_matcher_ = cv::StereoBM::create();
    sg_block_matcher_ = cv::StereoSGBM::create(1, 1, 10);
  }

  enum StereoType
  {
    BM, SGBM
  };

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

  inline
  StereoType getStereoType() const
  {
    return current_stereo_algorithm_;
  }

  inline
  void setStereoType(StereoType type)
  {
    current_stereo_algorithm_ = type;
  }

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
  void setDisparityRange(int range);  // Number of pixels to search

  // Disparity post-filtering parameters

  int getTextureThreshold() const;
  void setTextureThreshold(int threshold);

  float getUniquenessRatio() const;
  void setUniquenessRatio(float ratio);

  int getSpeckleSize() const;
  void setSpeckleSize(int size);

  int getSpeckleRange() const;
  void setSpeckleRange(int range);

  // SGBM only
  int getSgbmMode() const;
  void setSgbmMode(int fullDP);

  int getP1() const;
  void setP1(int P1);

  int getP2() const;
  void setP2(int P2);

  int getDisp12MaxDiff() const;
  void setDisp12MaxDiff(int disp12MaxDiff);

  // Do all the work!
  bool process(
    const sensor_msgs::msg::Image::ConstSharedPtr & left_raw,
    const sensor_msgs::msg::Image::ConstSharedPtr & right_raw,
    const image_geometry::StereoCameraModel & model,
    StereoImageSet & output,
    int flags) const;

  void processDisparity(
    const cv::Mat & left_rect,
    const cv::Mat & right_rect,
    const image_geometry::StereoCameraModel & model,
    stereo_msgs::msg::DisparityImage & disparity) const;

  void processPoints(
    const stereo_msgs::msg::DisparityImage & disparity,
    const cv::Mat & color,
    const std::string & encoding,
    const image_geometry::StereoCameraModel & model,
    sensor_msgs::msg::PointCloud & points) const;

  void processPoints2(
    const stereo_msgs::msg::DisparityImage & disparity,
    const cv::Mat & color,
    const std::string & encoding,
    const image_geometry::StereoCameraModel & model,
    sensor_msgs::msg::PointCloud2 & points) const;

private:
  image_proc::Processor mono_processor_;

  /// Scratch buffer for 16-bit signed disparity image
  mutable cv::Mat_<int16_t> disparity16_;
  /// Contains scratch buffers for block matching.
  mutable cv::Ptr<cv::StereoBM> block_matcher_;
  mutable cv::Ptr<cv::StereoSGBM> sg_block_matcher_;
  StereoType current_stereo_algorithm_;
  /// Scratch buffer for dense point cloud.
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

// For once, a macro is used just to avoid errors
#define STEREO_IMAGE_PROC_OPENCV2(GET, SET, TYPE, PARAM) \
inline TYPE StereoProcessor::GET() const \
{ \
  if (current_stereo_algorithm_ == BM) { \
    return block_matcher_.state->PARAM; \
  } \
  return sg_block_matcher_.PARAM; \
} \
\
inline void StereoProcessor::SET(TYPE param) \
{ \
  block_matcher_.state->PARAM = param; \
  sg_block_matcher_.PARAM = param; \
}

#define STEREO_IMAGE_PROC_OPENCV3(GET, SET, TYPE, GET_OPENCV, SET_OPENCV) \
inline TYPE StereoProcessor::GET() const \
{ \
  if (current_stereo_algorithm_ == BM) { \
    return block_matcher_->GET_OPENCV(); \
  } \
  return sg_block_matcher_->GET_OPENCV(); \
} \
\
inline void StereoProcessor::SET(TYPE param) \
{ \
  block_matcher_->SET_OPENCV(param); \
  sg_block_matcher_->SET_OPENCV(param); \
}

STEREO_IMAGE_PROC_OPENCV3(getPreFilterCap, setPreFilterCap, int, getPreFilterCap, setPreFilterCap)
STEREO_IMAGE_PROC_OPENCV3(
  getCorrelationWindowSize, setCorrelationWindowSize, int, getBlockSize, setBlockSize)
STEREO_IMAGE_PROC_OPENCV3(getMinDisparity, setMinDisparity, int, getMinDisparity, setMinDisparity)
STEREO_IMAGE_PROC_OPENCV3(
  getDisparityRange, setDisparityRange, int, getNumDisparities, setNumDisparities)
STEREO_IMAGE_PROC_OPENCV3(
  getUniquenessRatio, setUniquenessRatio, float, getUniquenessRatio, setUniquenessRatio)
STEREO_IMAGE_PROC_OPENCV3(
  getSpeckleSize, setSpeckleSize, int, getSpeckleWindowSize, setSpeckleWindowSize)
STEREO_IMAGE_PROC_OPENCV3(getSpeckleRange, setSpeckleRange, int, getSpeckleRange, setSpeckleRange)

#define STEREO_IMAGE_PROC_BM_ONLY_OPENCV2(GET, SET, TYPE, PARAM) \
inline TYPE StereoProcessor::GET() const \
{ \
  return block_matcher_.state->PARAM; \
} \
\
inline void StereoProcessor::SET(TYPE param) \
{ \
  block_matcher_.state->PARAM = param; \
}

#define STEREO_IMAGE_PROC_SGBM_ONLY_OPENCV2(GET, SET, TYPE, PARAM) \
inline TYPE StereoProcessor::GET() const \
{ \
  return sg_block_matcher_.PARAM; \
} \
\
inline void StereoProcessor::SET(TYPE param) \
{ \
  sg_block_matcher_.PARAM = param; \
}

#define STEREO_IMAGE_PROC_ONLY_OPENCV3(MEMBER, GET, SET, TYPE, GET_OPENCV, SET_OPENCV) \
inline TYPE StereoProcessor::GET() const \
{ \
  return MEMBER->GET_OPENCV(); \
} \
\
inline void StereoProcessor::SET(TYPE param) \
{ \
  MEMBER->SET_OPENCV(param); \
}

// BM only
STEREO_IMAGE_PROC_ONLY_OPENCV3(
  block_matcher_, getPreFilterSize, setPreFilterSize, int, getPreFilterSize, setPreFilterSize)
STEREO_IMAGE_PROC_ONLY_OPENCV3(
  block_matcher_, getTextureThreshold, setTextureThreshold,
  int, getTextureThreshold, setTextureThreshold)

// SGBM specific
// getSgbmMode can return MODE_SGBM = 0, MODE_HH = 1. FullDP == 1 was MODE_HH so we're good
STEREO_IMAGE_PROC_ONLY_OPENCV3(sg_block_matcher_, getSgbmMode, setSgbmMode, int, getMode, setMode)
STEREO_IMAGE_PROC_ONLY_OPENCV3(sg_block_matcher_, getP1, setP1, int, getP1, setP1)
STEREO_IMAGE_PROC_ONLY_OPENCV3(sg_block_matcher_, getP2, setP2, int, getP2, setP2)
STEREO_IMAGE_PROC_ONLY_OPENCV3(
  sg_block_matcher_, getDisp12MaxDiff, setDisp12MaxDiff, int, getDisp12MaxDiff, setDisp12MaxDiff)

}  // namespace stereo_image_proc

#endif  // STEREO_IMAGE_PROC__STEREO_PROCESSOR_HPP_
