#ifndef IMAGE_PROC_YUV422
#define IMAGE_PROC_YUV422

#include <opencv2/core/core.hpp>

// YUV422 conversion, intended for eventual inclusion in OpenCV

namespace image_proc {

void yuv422ToGray(const cv::Mat& yuv, cv::Mat& gray);

void yuv422ToColor(const cv::Mat& yuv, cv::Mat& color);

} // namespace image_proc

#endif
