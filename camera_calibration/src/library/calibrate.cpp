#include "camera_calibration/calibrate.h"

namespace camera_calibration {

Calibrater::Calibrater()
  : extrinsics_(NULL), flags_(0)
{
}

Calibrater::~Calibrater()
{
  cvReleaseMat(&extrinsics_);
}

void Calibrater::addView(const CvPoint2D32f* img_pts, const CvPoint3D32f* obj_pts, size_t n)
{
  image_points_.insert(image_points_.end(), img_pts, img_pts + n);
  object_points_.insert(object_points_.end(), obj_pts, obj_pts + n);
  point_counts_.push_back(n);
}

void Calibrater::getExtrinsics(CvMat &rot, CvMat &trans) const
{
  cvGetCols(extrinsics_, &rot, 0, 3);
  cvGetCols(extrinsics_, &trans, 3, 6);
}

void Calibrater::calibrate(int image_width, int image_height)
{
  model_.setParameters(image_width, image_height, 0.0, 0.0, 0.0, 0.0);
  
  CvMat obj_pts = cvMat(1, object_points_.size(), CV_32FC3, &object_points_[0]);
  CvMat img_pts = cvMat(1, image_points_.size(), CV_32FC2, &image_points_[0]);
  CvMat counts = cvMat(1, views(), CV_32SC1, &point_counts_[0]);
  CvMat K = model_.K_cv();
  CvMat D = model_.D_cv();
  cvReleaseMat(&extrinsics_);
  extrinsics_ = cvCreateMat(views(), 6, CV_32FC1);
  CvMat rot, trans;
  getExtrinsics(rot, trans);

  cvCalibrateCamera2(&obj_pts, &img_pts, &counts, cvSize(image_width, image_height),
                     &K, &D, &rot, &trans, flags_);
}

double Calibrater::reprojectionError() const
{
  const CvMat K = model_.K_cv();
  const CvMat D = model_.D_cv();
  std::vector<CvPoint2D32f> proj_pts(image_points_.size());
  CvMat rotations, translations;
  getExtrinsics(rotations, translations);
  
  int points_so_far = 0;
  double total_err = 0.0;
  
  for (size_t i = 0; i < views(); ++i) {
    int pt_count = point_counts_[i];
    const CvMat obj_pts_i = cvMat(1, pt_count, CV_32FC3,
                                  (void*)&object_points_[points_so_far]);
    const CvMat img_pts_i = cvMat(1, pt_count, CV_32FC2,
                                  (void*)&image_points_[points_so_far]);
    CvMat proj_pts_i = cvMat(1, pt_count, CV_32FC2, &proj_pts[points_so_far]);
    points_so_far += pt_count;

    CvMat rot_vect, trans_vect;
    cvGetRow(&rotations, &rot_vect, i);
    cvGetRow(&translations, &trans_vect, i);

    cvProjectPoints2(&obj_pts_i, &rot_vect, &trans_vect, &K, &D, &proj_pts_i);

    double err = cvNorm(&img_pts_i, &proj_pts_i, CV_L1);
    //per_view_errors[i] = err / pt_count;
    total_err += err;
  }

  return total_err / points_so_far;
}


CheckerboardDetector::CheckerboardDetector()
  : flags_(0), radius_(0)
{}

CheckerboardDetector::CheckerboardDetector(int width, int height, float square_size)
  : flags_(0), radius_(0)
{
  setDimensions(width, height, square_size);
}

void CheckerboardDetector::setDimensions(int width, int height, float square_size)
{
  board_w_ = width;
  board_h_ = height;
  square_size_ = square_size;
  grid_.clear();
  grid_.reserve( corners() );
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x)
      grid_.push_back(cvPoint3D32f(y*square_size, x*square_size, 0.0));
}

bool CheckerboardDetector::findCorners(const IplImage* image, CvPoint2D32f* corners,
                                       int* ncorners) const
{
  if ( !cvFindChessboardCorners(image, cvSize(board_w_, board_h_), corners,
                                ncorners, flags_) )
    return false;

  if (radius_ > 0) {
    cvFindCornerSubPix(image, corners, this->corners(), cvSize(radius_, radius_), cvSize(-1, -1),
                       cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
  }

  return true;
}

} //namespace camera_calibration
