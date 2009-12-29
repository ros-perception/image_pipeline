#ifndef IMAGE_PROC_PROCESSOR_H
#define IMAGE_PROC_PROCESSOR_H

#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>

namespace image_proc {

struct ImageSet
{
  std::string color_encoding;
  cv::Mat mono;
  cv::Mat rect;
  cv::Mat color;
  cv::Mat rect_color;
};

class Processor
{
public:
  Processor()
    : interpolation_(CV_INTER_LINEAR)
  {
  }
  
  int interpolation_;

  enum {
    MONO = 1 << 0,
    RECT = 1 << 1,
    COLOR = 1 << 2,
    RECT_COLOR = 1 << 3,
    ALL = MONO | RECT | COLOR | RECT_COLOR
  };
  
  bool process(const sensor_msgs::ImageConstPtr& raw_image,
               const image_geometry::PinholeCameraModel& model,
               ImageSet& output, int flags = ALL);
};

} //namespace image_proc

#endif
