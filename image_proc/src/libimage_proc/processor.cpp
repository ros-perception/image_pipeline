#include "image_proc/processor.h"
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>

namespace image_proc {

namespace enc = sensor_msgs::image_encodings;

bool Processor::process(const sensor_msgs::ImageConstPtr& raw_image,
                        const image_geometry::PinholeCameraModel& model,
                        ImageSet& output, int flags) const
{
  static const int MONO_EITHER = MONO | RECT;
  static const int COLOR_EITHER = COLOR | RECT_COLOR;
  if (!(flags & ALL)) return true;
  
  // Check if raw_image is color
  const std::string& raw_encoding = raw_image->encoding;
  int raw_type = CV_8UC1;
  if (raw_encoding == enc::BGR8 || raw_encoding == enc::RGB8) {
    raw_type = CV_8UC3;
    output.color_encoding = raw_encoding;
  }
  // Construct cv::Mat pointing to raw_image data
  const cv::Mat raw(raw_image->height, raw_image->width, raw_type,
                    const_cast<uint8_t*>(&raw_image->data[0]), raw_image->step);

  ///////////////////////////////////////////////////////
  // Construct colorized (unrectified) images from raw //
  ///////////////////////////////////////////////////////
  
  // Bayer case
  if (raw_encoding.find("bayer") != std::string::npos) {
    // Convert to color BGR
    /// @todo Faster to convert directly to mono when color is not requested, but OpenCV doesn't support
    int code = 0;
    if (raw_encoding == enc::BAYER_RGGB8)
      code = CV_BayerBG2BGR;
    else if (raw_encoding == enc::BAYER_BGGR8)
      code = CV_BayerRG2BGR;
    else if (raw_encoding == enc::BAYER_GBRG8)
      code = CV_BayerGR2BGR;
    else if (raw_encoding == enc::BAYER_GRBG8)
      code = CV_BayerGB2BGR;
    else {
      ROS_ERROR("[image_proc] Unsupported encoding '%s'", raw_encoding.c_str());
      return false;
    }
    cv::cvtColor(raw, output.color, code);
    output.color_encoding = enc::BGR8;
    
    if (flags & MONO_EITHER)
      cv::cvtColor(output.color, output.mono, CV_BGR2GRAY);
  }
  // Color case
  else if (raw_type == CV_8UC3) {
    output.color = raw;
    if (flags & MONO_EITHER) {
      int code = (raw_encoding == enc::BGR8) ? CV_BGR2GRAY : CV_RGB2GRAY;
      cv::cvtColor(output.color, output.mono, code);
    }
  }
  // Mono case
  else if (raw_encoding == enc::MONO8) {
    output.mono = raw;
    if (flags & COLOR_EITHER) {
      output.color_encoding = enc::MONO8;
      output.color = raw;
    }
  }
  // 8UC3 does not specify a color encoding. Is it BGR, RGB, HSV, XYZ, LUV...?
  else if (raw_encoding == enc::TYPE_8UC3) {
    ROS_ERROR("[image_proc] Ambiguous encoding '8UC3'. The camera driver "
              "should set the encoding to 'bgr8' or 'rgb8'.");
    return false;
  }
  // Something else we can't handle
  else {
    ROS_ERROR("[image_proc] Unsupported encoding '%s'", raw_encoding.c_str());
    return false;
  }

  //////////////////////////////////////////////////////
  // Construct rectified images from colorized images //
  //////////////////////////////////////////////////////
  
  /// @todo If no distortion, could just point to the colorized data. But copy is
  /// already way faster than remap.
  if (flags & RECT)
    model.rectifyImage(output.mono, output.rect, interpolation_);
  if (flags & RECT_COLOR)
    model.rectifyImage(output.color, output.rect_color, interpolation_);

  return true;
}

} //namespace image_proc
