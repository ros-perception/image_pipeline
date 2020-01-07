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

  enum
  {
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

  inline StereoType getStereoType() const
  {
    return current_stereo_algorithm_;
  }

  inline void setStereoType(StereoType type)
  {
    current_stereo_algorithm_ = type;
  }

  inline int getInterpolation() const
  {
    return mono_processor_.interpolation_;
  }

  inline void setInterpolation(int interp)
  {
    mono_processor_.interpolation_ = interp;
  }

  inline int getPreFilterCap() const
  {
    if (current_stereo_algorithm_ == BM) {
      return block_matcher_->getPreFilterCap();
    }
    return sg_block_matcher_->getPreFilterCap();
  }

  inline void setPreFilterCap(int param)
  {
    block_matcher_->setPreFilterCap(param);
    sg_block_matcher_->setPreFilterCap(param);
  }

  inline int getCorrelationWindowSize() const
  {
    if (current_stereo_algorithm_ == BM) {
      return block_matcher_->getBlockSize();
    }
    return sg_block_matcher_->getBlockSize();
  }

  inline void setCorrelationWindowSize(int param)
  {
    block_matcher_->setBlockSize(param);
    sg_block_matcher_->setBlockSize(param);
  }

  inline int getMinDisparity() const
  {
    if (current_stereo_algorithm_ == BM) {
      return block_matcher_->getMinDisparity();
    }
    return sg_block_matcher_->getMinDisparity();
  }

  inline void setMinDisparity(int param)
  {
    block_matcher_->setMinDisparity(param);
    sg_block_matcher_->setMinDisparity(param);
  }

  inline int getDisparityRange() const
  {
    if (current_stereo_algorithm_ == BM) {
      return block_matcher_->getNumDisparities();
    }
    return sg_block_matcher_->getNumDisparities();
  }

  inline void setDisparityRange(int param)
  {
    block_matcher_->setNumDisparities(param);
    sg_block_matcher_->setNumDisparities(param);
  }

  inline float getUniquenessRatio() const
  {
    if (current_stereo_algorithm_ == BM) {
      return block_matcher_->getUniquenessRatio();
    }
    return sg_block_matcher_->getUniquenessRatio();
  }

  inline void setUniquenessRatio(float param)
  {
    block_matcher_->setUniquenessRatio(param);
    sg_block_matcher_->setUniquenessRatio(param);
  }

  inline int getSpeckleSize() const
  {
    if (current_stereo_algorithm_ == BM) {
      return block_matcher_->getSpeckleWindowSize();
    }
    return sg_block_matcher_->getSpeckleWindowSize();
  }

  inline void setSpeckleSize(int param)
  {
    block_matcher_->setSpeckleWindowSize(param);
    sg_block_matcher_->setSpeckleWindowSize(param);
  }

  inline int getSpeckleRange() const
  {
    if (current_stereo_algorithm_ == BM) {
      return block_matcher_->getSpeckleRange();
    }
    return sg_block_matcher_->getSpeckleRange();
  }

  inline void setSpeckleRange(int param)
  {
    block_matcher_->setSpeckleRange(param);
    sg_block_matcher_->setSpeckleRange(param);
  }

  // BM only

  inline int getPreFilterSize() const
  {
    return block_matcher_->getPreFilterSize();
  }

  inline void setPreFilterSize(int param)
  {
    block_matcher_->setPreFilterSize(param);
  }

  inline int getTextureThreshold() const
  {
    return block_matcher_->getTextureThreshold();
  }

  inline void setTextureThreshold(int param)
  {
    block_matcher_->setTextureThreshold(param);
  }

  // SGBM specific

  // getSgbmMode can return MODE_SGBM = 0, MODE_HH = 1. FullDP == 1 was MODE_HH so we're good
  inline int getSgbmMode() const
  {
    return sg_block_matcher_->getMode();
  }

  inline void setSgbmMode(int param)
  {
    sg_block_matcher_->setMode(param);
  }

  inline int getP1() const
  {
    return sg_block_matcher_->getP1();
  }

  inline void setP1(int param)
  {
    sg_block_matcher_->setP1(param);
  }

  inline int getP2() const
  {
    return sg_block_matcher_->getP2();
  }

  inline void setP2(int param)
  {
    sg_block_matcher_->setP2(param);
  }

  inline int getDisp12MaxDiff() const
  {
    return sg_block_matcher_->getDisp12MaxDiff();
  }

  inline void setDisp12MaxDiff(int param)
  {
    sg_block_matcher_->setDisp12MaxDiff(param);
  }

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

}  // namespace stereo_image_proc

#endif  // STEREO_IMAGE_PROC__STEREO_PROCESSOR_HPP_
