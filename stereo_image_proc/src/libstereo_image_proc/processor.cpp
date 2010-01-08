#include "stereo_image_proc/processor.h"
#include "stereo_image_proc/stereolib.h"
#include <sensor_msgs/image_encodings.h>

namespace stereo_image_proc {

bool StereoProcessor::process(const sensor_msgs::ImageConstPtr& left_raw,
                              const sensor_msgs::ImageConstPtr& right_raw,
                              const image_geometry::StereoCameraModel& model,
                              StereoImageSet& output, int flags) const
{
  // Do monocular processing on left and right images
  int left_flags = flags & LEFT_ALL;
  int right_flags = flags & RIGHT_ALL;
  if (flags & STEREO_ALL) {
    // Need the rectified images for stereo processing
    left_flags |= LEFT_RECT;
    right_flags |= RIGHT_RECT;
  }
  if (flags & POINT_CLOUD) {
    flags |= DISPARITY;
    // Need the color channels for the point cloud
    left_flags |= LEFT_RECT_COLOR;
  }
  if (!mono_processor_.process(left_raw, model.left(), output.left, left_flags))
    return false;
  if (!mono_processor_.process(right_raw, model.right(), output.right, right_flags >> 4))
    return false;

  // Do block matching to produce the disparity image
  if (flags & DISPARITY) {
    processDisparity(output.left.rect, output.right.rect, model, output.disparity);
  }

  /// @todo Project disparity image to 3d point cloud
  /*
  if (flags & POINT_CLOUD)
    processPoints();
  */

  return true;
}

void StereoProcessor::processDisparity(const cv::Mat& left_rect, const cv::Mat& right_rect,
                                       const image_geometry::StereoCameraModel& model,
                                       stereo_msgs::DisparityImage& disparity) const
{
  // Fixed-point disparity is 16 times the true value: d = d_fp / 16.0 = x_l - x_r.
  static const int DPP = 16; // disparities per pixel
  static const double inv_dpp = 1.0 / DPP;
  
  // Block matcher produces 16-bit signed (fixed point) disparity image
  block_matcher_(left_rect, right_rect, disparity16_);

  // OpenCV doesn't do speckle filtering yet! Do it ourselves.
  labels_.create(disparity16_.size());
  wavefront_.create(disparity16_.size());
  region_types_.create(disparity16_.size());
  int speckle_size = getSpeckleSize(); // actually region size
  int speckle_diff = getSpeckleRange();
  int16_t bad_value = getMinDisparity() - DPP;
  do_speckle(disparity16_[0], bad_value, disparity16_.cols, disparity16_.rows, speckle_diff,
             speckle_size, labels_[0], wavefront_[0], region_types_[0]);

  // Fill in DisparityImage image data, converting to 32-bit float
  sensor_msgs::Image& dimage = disparity.image;
  dimage.height = disparity16_.rows;
  dimage.width = disparity16_.cols;
  dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  dimage.step = dimage.width * sizeof(float);
  dimage.data.resize(dimage.step * dimage.height);
  cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  // We convert from fixed-point to float disparity and also adjust for any x-offset between
  // the principal points: d = d_fp*inv_dpp - (cx_l - cx_r)
  disparity16_.convertTo(dmat, dmat.type(), inv_dpp, -(model.left().cx() - model.right().cx()));
  ROS_ASSERT(dmat.data == &dimage.data[0]);
  /// @todo is_bigendian? :)

  // Stereo parameters
  disparity.f = model.right().fx();
  disparity.T = model.baseline();

  /// @todo Window of (potentially) valid disparities

  // Disparity search range
  disparity.min_disparity = getMinDisparity();
  disparity.max_disparity = getMinDisparity() + getDisparityRange() - 1;
  disparity.delta_d = inv_dpp;
}

} //namespace stereo_image_proc
