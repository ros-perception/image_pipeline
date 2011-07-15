#ifndef IMAGE_PROC_EDGE_AWARE
#define IMAGE_PROC_EDGE_AWARE

#include <opencv2/core/core.hpp>

// Edge-aware debayering algorithms, intended for eventual inclusion in OpenCV.

namespace image_proc {

void debayerEdgeAware(const cv::Mat& bayer, cv::Mat& color);

void debayerEdgeAwareWeighted(const cv::Mat& bayer, cv::Mat& color);

} // namespace image_proc

#endif
