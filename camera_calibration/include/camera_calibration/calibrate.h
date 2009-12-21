// license

#ifndef _CAMERA_CALIBRATION_CALIBRATE_H_
#define _CAMERA_CALIBRATION_CALIBRATE_H_

#include "camera_calibration/pinhole.h"
#include <vector>

namespace camera_calibration {

class Calibrater
{
public:
  Calibrater();

  ~Calibrater();

  //void useIntrinsicGuess(bool on = true);
  //void fixPrincipalPoint(bool on = true);
  //void fixAspectRatio(bool on = true);
  //void zeroTangentialDistortion(bool on = true);
  //void useDistortionCoefficients(int);
  void setFlags(int flags);

  size_t views() const;
  
  void addView(const CvPoint2D32f* img_pts, const CvPoint3D32f* obj_pts, size_t n);

  void calibrate(int image_width, int image_height);

  /** @todo: per-view errors, extrinsics */
  double reprojectionError() const;
  
  PinholeCameraModel& model();
  const PinholeCameraModel& model() const;

private:
  PinholeCameraModel model_;
  std::vector<CvPoint3D32f> object_points_;
  std::vector<CvPoint2D32f> image_points_;
  std::vector<int> point_counts_;
  CvMat* extrinsics_;
  int flags_;

  void getExtrinsics(CvMat &rot, CvMat &trans) const;
};

class CheckerboardDetector
{
public:
  CheckerboardDetector();
  CheckerboardDetector(int width, int height, float square_size);

  void setDimensions(int width, int height, float square_size);
  void setFlags(int flags);
  void setSearchRadius(int radius); // 0 to disable

  bool findCorners(const IplImage* image, CvPoint2D32f* corners, int* ncorners = NULL) const;

  const CvPoint3D32f* objectPoints() const;
  //const CvMat objectPointsMat() const;

  int width() const;
  int height() const;
  int corners() const;
  float squareSize() const;
  
private:
  int board_w_, board_h_;
  float square_size_;
  std::vector<CvPoint3D32f> grid_;
  int flags_;
  int radius_;
};


inline size_t Calibrater::views() const { return point_counts_.size(); }

inline void Calibrater::setFlags(int flags) { flags_ = flags; }

inline PinholeCameraModel& Calibrater::model() { return model_; }
inline const PinholeCameraModel& Calibrater::model() const { return model_; }

inline void CheckerboardDetector::setFlags(int flags) { flags_ = flags; }
inline void CheckerboardDetector::setSearchRadius(int radius) { radius_ = radius; }

inline int CheckerboardDetector::width()   const { return board_w_; }
inline int CheckerboardDetector::height()  const { return board_h_; }
inline int CheckerboardDetector::corners() const { return board_w_*board_h_; }
inline float CheckerboardDetector::squareSize() const { return square_size_; }

inline const CvPoint3D32f* CheckerboardDetector::objectPoints() const
{
  return &grid_[0];
}

} //namespace camera_calibration

#endif
