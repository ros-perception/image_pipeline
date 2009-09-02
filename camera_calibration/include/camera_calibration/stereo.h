// license

#ifndef _CAMERA_CALIBRATION_STEREO_H_
#define _CAMERA_CALIBRATION_STEREO_H_

#include "camera_calibration/pinhole.h"

namespace camera_calibration {

class StereoPinholeCameraModel : public PinholeCameraModel
{
public:
  double R[3*3];
  double P[3*4];

  StereoPinholeCameraModel();

  CvMat R_cv();
  const CvMat R_cv() const;
  CvMat P_cv();
  const CvMat P_cv() const;

  // file I/O
  virtual bool load(const std::string& file_name);
  virtual bool save(const std::string& file_name);
  virtual bool parse(const std::string& buffer, const std::string& format = "ini");
};

// class StereoPair

inline CvMat StereoPinholeCameraModel::R_cv()
{
  return cvMat(3, 3, CV_64FC1, R);
}

inline const CvMat PinholeCameraModel::R_cv() const
{
  return cvMat(3, 3, CV_64FC1, const_cast<double*>(R));
}

inline CvMat StereoPinholeCameraModel::P_cv()
{
  return cvMat(3, 4, CV_64FC1, P);
}

inline const CvMat PinholeCameraModel::P_cv() const
{
  return cvMat(3, 4, CV_64FC1, const_cast<double*>(P));
}

} //namespace camera_calibration

#endif
