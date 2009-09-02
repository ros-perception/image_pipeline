#include "camera_calibration/pinhole.h"

#include <opencv/highgui.h>
#include <boost/algorithm/string.hpp>
#include <cstdio>

using namespace camera_calibration;

int main(int argc, char** argv)
{
  if (argc < 2) {
    printf("Usage: %s intrinsics.yml [FILES]\n", argv[0]);
    return 0;
  }

  // Read camera model from file
  PinholeCameraModel model;
  model.load(argv[1]);

  // Undistort images
  cv::WImageBuffer1_f undistorted(model.width(), model.height());
  for (int i = 2; i < argc; ++i) {
    cv::WImageBuffer3_b img( cvLoadImage(argv[i]) );
    model.undistort(img.Ipl(), undistorted.Ipl());
    std::string name(argv[i]);
    boost::replace_last(name, ".", "_rect.");
    if (cvSaveImage(name.c_str(), undistorted.Ipl()))
      printf("Saved %s\n", name.c_str());
  }

  return 0;
}
