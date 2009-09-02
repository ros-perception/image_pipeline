#include "camera_calibration/pinhole.h"
#include <camera_calibration_parsers/parse.h>

#include <algorithm>

namespace camera_calibration {

PinholeCameraModel::PinholeCameraModel()
{
  std::fill(K, K+9, 0.0);
  std::fill(D, D+5, 0.0);
}

PinholeCameraModel::PinholeCameraModel(const PinholeCameraModel& other)
  : camera_name_(other.camera_name_),
    image_width_(other.image_width_), image_height_(other.image_height_),
    distorted_(other.distorted_)
{
  std::copy(other.K, other.K+9, K);
  std::copy(other.D, other.D+5, D);

  undistort_map_x_.CloneFrom(other.undistort_map_x_);
  undistort_map_y_.CloneFrom(other.undistort_map_y_);
}

void PinholeCameraModel::setParameters(int width, int height, double fx,
                                       double fy, double cx, double cy)
{
  std::fill(K, K+9, 0.0);
  image_width_ = width;
  image_height_ = height;
  this->fx() = fx;
  this->fy() = fy;
  this->cx() = cx;
  this->cy() = cy;

  initUndistortMap();
}

void PinholeCameraModel::setDistortion(const double* D_new)
{
  if (D_new) {
    std::copy(D_new, D_new+9, D);
    initUndistortMap();
  } else {
    std::fill(D, D+5, 0.0);
    distorted_ = false;
  }
}

PinholeCameraModel PinholeCameraModel::withRoi(int x, int y, int width, int height) const
{
  PinholeCameraModel roi_model;
  std::copy(K, K+9, roi_model.K);
  std::copy(D, D+5, roi_model.D);
  roi_model.image_width_ = width;
  roi_model.image_height_ = height;

  // Move principal point
  roi_model.cx() -= x;
  roi_model.cy() -= y;

  // Compute ROI undistort map
  roi_model.distorted_ = distorted_;
  if (distorted_) {
    roi_model.undistort_map_x_.Allocate(width, height);
    /** @todo: need const version of View in cvwimage.h */
    cvSubS(const_cast<PinholeCameraModel*>(this)->undistort_map_x_.View(x, y, width, height).Ipl(),
           cvScalar(x), roi_model.undistort_map_x_.Ipl());
    roi_model.undistort_map_y_.Allocate(width, height);
    cvSubS(const_cast<PinholeCameraModel*>(this)->undistort_map_y_.View(x, y, width, height).Ipl(),
           cvScalar(y), roi_model.undistort_map_y_.Ipl());
  }

  return roi_model;
}

void PinholeCameraModel::undistort(IplImage* src, IplImage* dst) const
{
  if (distorted_)
    cvRemap(src, dst, undistort_map_x_.Ipl(), undistort_map_y_.Ipl(),
            CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);
  else
    cvCopy(src, dst);
}

bool PinholeCameraModel::load(const std::string& file_name)
{
  using namespace camera_calibration_parsers;
  bool success = readCalibration(file_name, camera_name_, image_width_, image_height_, K, D);
  if (success)
    initUndistortMap();
  return success;
}

bool PinholeCameraModel::save(const std::string& file_name) const
{
  using namespace camera_calibration_parsers;
  return writeCalibration(file_name, camera_name_, image_width_, image_height_, K, D);
}

bool PinholeCameraModel::parse(const std::string& buffer, const std::string& format)
{
  using namespace camera_calibration_parsers;
  bool success = parseCalibration(buffer, format, camera_name_, image_width_, image_height_, K, D);
  if (success)
    initUndistortMap();
  return success;
}

void PinholeCameraModel::fillCameraInfo(sensor_msgs::CameraInfo &info) const
{
  std::copy(K, K+9, &info.K[0]);
  std::copy(D, D+5, &info.D[0]);
  /** @todo: set R, P? */

  info.height = image_height_;
  info.width = image_width_;
}

void PinholeCameraModel::initUndistortMap()
{
  distorted_ = D[0]!=0.0 || D[1]!=0.0 || D[2]!=0.0 || D[3]!=0.0 || D[4]!=0.0;
  if (!distorted_)
    return;

  undistort_map_x_.Allocate(image_width_, image_height_);
  undistort_map_y_.Allocate(image_width_, image_height_);
  const CvMat Km = K_cv();
  const CvMat Dm = D_cv();
  cvInitUndistortMap(&Km, &Dm, undistort_map_x_.Ipl(), undistort_map_y_.Ipl());
}

} //namespace camera_calibration
