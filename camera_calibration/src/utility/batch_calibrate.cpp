#include <cstdio>
#include "camera_calibration/calibrate.h"
#include <opencv/highgui.h>

using namespace camera_calibration;

static const char wnd_name[] = "Calibration";

int main(int argc, char** argv)
{
  /** @todo: full-blown Boost.ProgramOptions interface */
  int board_width = 6;
  int board_height = 9;
  float square_size = 0.0225;
  int image_width = 0, image_height = 0;

  CheckerboardDetector detector(board_width, board_height, square_size);
  detector.setFlags(CV_CALIB_CB_ADAPTIVE_THRESH);
  detector.setSearchRadius(5);
  std::vector<CvPoint2D32f> corners(detector.corners());
  
  Calibrater cal;
  cal.setFlags(CV_CALIB_FIX_PRINCIPAL_POINT);

  cvNamedWindow(wnd_name, 0);
  
  for (int i = 1; i < argc; ++i) {
    cv::WImageBuffer1_b image( cvLoadImage(argv[i], CV_LOAD_IMAGE_GRAYSCALE) );
    image_width = image.Width();
    image_height = image.Height();

    int ncorners = 0;
    bool success = detector.findCorners(image.Ipl(), &corners[0], &ncorners);
    if (success) {
      cal.addView(&corners[0], detector.objectPoints(), corners.size());
      printf("%s success\n", argv[i]);
    } else {
      printf("ERROR: Couldn't find checkerboard in image %s\n", argv[i]);
    }

    cv::WImageBuffer3_b display(image.Width(), image.Height());
    cvCvtColor(image.Ipl(), display.Ipl(), CV_GRAY2BGR);
    cvDrawChessboardCorners(display.Ipl(), cvSize(board_width, board_height),
                            &corners[0], ncorners, success);
    cvShowImage(wnd_name, display.Ipl());
    cvWaitKey(20);
  }

  cal.calibrate(image_width, image_height);
  cal.model().save("out.ini");
  printf("Ave reprojection error: %f\n", cal.reprojectionError());

  cvDestroyAllWindows();
  
  return 0;
}
