#ifndef STEREO_IMAGE_PROC_PROCESSOR_H
#define STEREO_IMAGE_PROC_PROCESSOR_H

#include <image_proc/processor.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud.h>

namespace stereo_image_proc {

struct StereoImageSet
{
  image_proc::ImageSet left;
  image_proc::ImageSet right;
  stereo_msgs::DisparityImage disparity;
  sensor_msgs::PointCloud points;
};

class StereoProcessor
{
public:
  enum {
    OPENCV_BLOCK_MATCHER,
    WG_BLOCK_MATCHER
  };

  
  StereoProcessor()
    : disparity_algorithm_(WG_BLOCK_MATCHER),
      block_matcher_(cv::StereoBM::BASIC_PRESET)
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

    LEFT_ALL = LEFT_MONO | LEFT_RECT | LEFT_COLOR | LEFT_RECT_COLOR,
    RIGHT_ALL = RIGHT_MONO | RIGHT_RECT | RIGHT_COLOR | RIGHT_RECT_COLOR,
    STEREO_ALL = DISPARITY | POINT_CLOUD,
    ALL = LEFT_ALL | RIGHT_ALL | STEREO_ALL
  };

  int getDisparityAlgorithm() const { return disparity_algorithm_; }
  void setDisparityAlgorithm(int alg) { disparity_algorithm_ = alg; }

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

private:
  image_proc::Processor mono_processor_;
  int disparity_algorithm_;
  
  mutable cv::Mat_<int16_t> disparity16_; // scratch buffer for 16-bit signed disparity image
  mutable cv::StereoBM block_matcher_; // contains scratch buffers for block matching
  // scratch buffers for speckle filtering
  mutable cv::Mat_<uint32_t> labels_;
  mutable cv::Mat_<uint32_t> wavefront_;
  mutable cv::Mat_<uint8_t> region_types_;
  // scratch buffer for dense point cloud
  mutable cv::Mat_<cv::Vec3f> dense_points_;
  // scratch buffers for stereolib functions (if not using OpenCV)
  mutable cv::Mat_<uint8_t> left_feature_;
  mutable cv::Mat_<uint8_t> right_feature_;
  mutable cv::Mat_<uint8_t> disp_buffer_;

  void processDisparity(const cv::Mat& left_rect, const cv::Mat& right_rect,
                        const image_geometry::StereoCameraModel& model,
                        stereo_msgs::DisparityImage& disparity) const;

  void processPoints(const stereo_msgs::DisparityImage& disparity,
                     const cv::Mat& color, const std::string& encoding,
                     const image_geometry::StereoCameraModel& model,
                     sensor_msgs::PointCloud& points) const;
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
