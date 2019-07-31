#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "popt_pp.h"

using namespace std;
using namespace cv;

vector< vector< Point3d > > object_points;
vector< vector< Point2f > > imagePoints1, imagePoints2;
vector< Point2f > corners1, corners2;
vector< vector< Point2d > > left_img_points, right_img_points;

Mat img1, img2, gray1, gray2, spl1, spl2;

void load_image_points(int board_width, int board_height, float square_size, int num_imgs, 
                      char* img_dir, char* leftimg_filename, char* rightimg_filename) {
  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;

  for (int i = 1; i <= num_imgs; i++) {
    char left_img[100], right_img[100];
    sprintf(left_img, "%s%s%02d.png", img_dir, leftimg_filename, i);
    sprintf(right_img, "%s%s%02d.png", img_dir, rightimg_filename, i);
    img1 = imread(left_img, CV_LOAD_IMAGE_COLOR);
    img2 = imread(right_img, CV_LOAD_IMAGE_COLOR);
    cv::cvtColor(img1, gray1, CV_BGR2GRAY);
    cv::cvtColor(img2, gray2, CV_BGR2GRAY);

    bool found1 = false, found2 = false;

    found1 = cv::findChessboardCorners(img1, board_size, corners1,
  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    found2 = cv::findChessboardCorners(img2, board_size, corners2,
  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if (found1)
    {
      cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
  cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      cv::drawChessboardCorners(gray1, board_size, corners1, found1);
    }
    if (found2)
    {
      cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
  cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      cv::drawChessboardCorners(gray2, board_size, corners2, found2);
    }

    vector<cv::Point3d> obj;
    for( int i = 0; i < board_height; ++i )
      for( int j = 0; j < board_width; ++j )
        obj.push_back(Point3d(double( (float)j * square_size ), double( (float)i * square_size ), 0));

    if (found1 && found2) {
      cout << i << ". Found corners!" << endl;
      imagePoints1.push_back(corners1);
      imagePoints2.push_back(corners2);
      object_points.push_back(obj);
    }
  }
  for (int i = 0; i < imagePoints1.size(); i++) {
    vector< Point2d > v1, v2;
    for (int j = 0; j < imagePoints1[i].size(); j++) {
      v1.push_back(Point2d((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
      v2.push_back(Point2d((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
    }
    left_img_points.push_back(v1);
    right_img_points.push_back(v2);
  }
}

int main(int argc, char const *argv[])
{
  int board_width, board_height, num_imgs;
  float square_size;
  char* img_dir;
  char* leftimg_filename;
  char* rightimg_filename;
  char* out_file;

  static struct poptOption options[] = {
    { "board_width",'w',POPT_ARG_INT,&board_width,0,"Checkerboard width","NUM" },
    { "board_height",'h',POPT_ARG_INT,&board_height,0,"Checkerboard height","NUM" },
    { "square_size",'s',POPT_ARG_FLOAT,&square_size,0,"Checkerboard square size","NUM" },
    { "num_imgs",'n',POPT_ARG_INT,&num_imgs,0,"Number of checkerboard images","NUM" },
    { "img_dir",'d',POPT_ARG_STRING,&img_dir,0,"Directory containing images","STR" },
    { "leftimg_filename",'l',POPT_ARG_STRING,&leftimg_filename,0,"Left image prefix","STR" },
    { "rightimg_filename",'r',POPT_ARG_STRING,&rightimg_filename,0,"Right image prefix","STR" },
    { "out_file",'o',POPT_ARG_STRING,&out_file,0,"Output calibration filename (YML)","STR" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}

  load_image_points(board_width, board_height, square_size, num_imgs, img_dir, leftimg_filename, rightimg_filename);

  printf("Starting Calibration\n");
  cv::Matx33d K1, K2, R;
  cv::Vec3d T;
  cv::Vec4d D1, D2;
  int flag = 0;
  flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
  flag |= cv::fisheye::CALIB_CHECK_COND;
  flag |= cv::fisheye::CALIB_FIX_SKEW;
  //flag |= cv::fisheye::CALIB_FIX_K2;
  //flag |= cv::fisheye::CALIB_FIX_K3;
  //flag |= cv::fisheye::CALIB_FIX_K4;
  cv::fisheye::stereoCalibrate(object_points, left_img_points, right_img_points,
      K1, D1, K2, D2, img1.size(), R, T, flag,
      cv::TermCriteria(3, 12, 0));

  cv::FileStorage fs1(out_file, cv::FileStorage::WRITE);
  fs1 << "K1" << Mat(K1);
  fs1 << "K2" << Mat(K2);
  fs1 << "D1" << D1;
  fs1 << "D2" << D2;
  fs1 << "R" << Mat(R);
  fs1 << "T" << T;
  printf("Done Calibration\n");

  printf("Starting Rectification\n");

  cv::Mat R1, R2, P1, P2, Q;
  cv::fisheye::stereoRectify(K1, D1, K2, D2, img1.size(), R, T, R1, R2, P1, P2, 
Q, CV_CALIB_ZERO_DISPARITY, img1.size(), 0.0, 1.1);

  fs1 << "R1" << R1;
  fs1 << "R2" << R2;
  fs1 << "P1" << P1;
  fs1 << "P2" << P2;
  fs1 << "Q" << Q;

  printf("Done Rectification\n");
  return 0;
}
