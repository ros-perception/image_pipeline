// license

#ifndef _CAMERA_CALIBRATION_INTRINSICS_H_
#define _CAMERA_CALIBRATION_INTRINSICS_H_

#include <string>
#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <sensor_msgs/CameraInfo.h>

namespace camera_calibration {

class PinholeCameraModel
{
public:
  double K[3*3];
  double D[5];

  // No-initialization constructor
  PinholeCameraModel();

  // Copy constructor
  PinholeCameraModel(const PinholeCameraModel& other);

  void setParameters(int width, int height, double fx, double fy,
                     double cx, double cy);
  // Can pass NULL for zero distortion
  void setDistortion(const double* D);
  
  const std::string& name() const;

  int width() const;
  int height() const;
  
  // getters for focal length, principal point
  double& fx();
  double fx() const;
  double& fy();
  double fy() const;
  double& cx();
  double cx() const;
  double& cy();
  double cy() const;
  
  // to CvMat functions
  CvMat K_cv();
  const CvMat K_cv() const;
  CvMat D_cv();
  const CvMat D_cv() const;

  // move principal point, update distortion matrices...
  // NOTE: distortion params won't really be right, will they?
  PinholeCameraModel withRoi(int x, int y, int width, int height) const;

  // rescale
  //virtual void setResolution(int new_width, int new_height);

  // undistort (& rectify)
  bool hasDistortion() const;
  virtual void undistort(IplImage* src, IplImage* dst) const;
  
  // file I/O
  virtual bool load(const std::string& file_name);
  virtual bool save(const std::string& file_name) const;
  virtual bool parse(const std::string& buffer, const std::string& format = "ini");

  virtual void fillCameraInfo(sensor_msgs::CameraInfo &info) const;

protected:
  std::string camera_name_;
  int image_width_;
  int image_height_;

  bool distorted_;
  cv::WImageBuffer1_f undistort_map_x_, undistort_map_y_;

  void initUndistortMap();
};


inline const std::string& PinholeCameraModel::name() const
{
  return camera_name_;
}

inline int PinholeCameraModel::width() const { return image_width_; }
inline int PinholeCameraModel::height() const { return image_height_; }

inline double& PinholeCameraModel::fx() { return K[0]; }
inline double PinholeCameraModel::fx() const { return K[0]; }
inline double& PinholeCameraModel::fy() { return K[4]; }
inline double PinholeCameraModel::fy() const { return K[4]; }
inline double& PinholeCameraModel::cx() { return K[2]; }
inline double PinholeCameraModel::cx() const { return K[2]; }
inline double& PinholeCameraModel::cy() { return K[5]; }
inline double PinholeCameraModel::cy() const { return K[5]; }

inline CvMat PinholeCameraModel::K_cv()
{
  return cvMat(3, 3, CV_64FC1, K);
}

inline const CvMat PinholeCameraModel::K_cv() const
{
  return cvMat(3, 3, CV_64FC1, const_cast<double*>(K));
}

inline CvMat PinholeCameraModel::D_cv()
{
  return cvMat(1, 5, CV_64FC1, D);
}

inline const CvMat PinholeCameraModel::D_cv() const
{
  return cvMat(1, 5, CV_64FC1, const_cast<double*>(D));
}

inline bool PinholeCameraModel::hasDistortion() const { return distorted_; }

} //namespace camera_calibration

#endif
