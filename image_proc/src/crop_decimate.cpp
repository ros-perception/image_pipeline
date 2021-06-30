// Copyright 2008, 2019 Willow Garage, Inc., Steve Macenski, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "image_proc/crop_decimate.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <thread>

namespace image_proc
{

template<typename T>
void debayer2x2toBGR(
  const cv::Mat & src, cv::Mat & dst,
  int R, int G1, int G2, int B)
{
  typedef cv::Vec<T, 3> DstPixel;  // 8- or 16-bit BGR
#if CV_VERSION_MAJOR >= 4 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION > 2)
  dst.create(src.rows / 2, src.cols / 2, cv::traits::Type<DstPixel>::value);
#else
  dst.create(src.rows / 2, src.cols / 2, cv::DataType<DstPixel>::type);
#endif

  int src_row_step = src.step1();
  int dst_row_step = dst.step1();
  const T * src_row = src.ptr<T>();
  T * dst_row = dst.ptr<T>();

  // 2x2 downsample and debayer at once
  for (int y = 0; y < dst.rows; ++y) {
    for (int x = 0; x < dst.cols; ++x) {
      dst_row[x * 3 + 0] = src_row[x * 2 + B];
      dst_row[x * 3 + 1] = (src_row[x * 2 + G1] + src_row[x * 2 + G2]) / 2;
      dst_row[x * 3 + 2] = src_row[x * 2 + R];
    }

    src_row += src_row_step * 2;
    dst_row += dst_row_step;
  }
}

// Templated on pixel size, in bytes (MONO8 = 1, BGR8 = 3, RGBA16 = 8, ...)
template<int N>
void decimate(const cv::Mat & src, cv::Mat & dst, int decimation_x, int decimation_y)
{
  dst.create(src.rows / decimation_y, src.cols / decimation_x, src.type());

  int src_row_step = src.step[0] * decimation_y;
  int src_pixel_step = N * decimation_x;
  int dst_row_step = dst.step[0];

  const uint8_t * src_row = src.ptr();
  uint8_t * dst_row = dst.ptr();

  for (int y = 0; y < dst.rows; ++y) {
    const uint8_t * src_pixel = src_row;
    uint8_t * dst_pixel = dst_row;

    for (int x = 0; x < dst.cols; ++x) {
      memcpy(dst_pixel, src_pixel, N);  // Should inline with small, fixed N
      src_pixel += src_pixel_step;
      dst_pixel += N;
    }

    src_row += src_row_step;
    dst_row += dst_row_step;
  }
}

CropDecimateNode::CropDecimateNode(const rclcpp::NodeOptions & options)
: Node("CropNonZeroNode", options)
{
  queue_size_ = this->declare_parameter("queue_size", 5);
  target_frame_id_ = this->declare_parameter("target_frame_id", std::string());

  // default: do nothing
  decimation_x_ = this->declare_parameter("decimation_x", 1);
  decimation_y_ = this->declare_parameter("decimation_y", 1);

  // default: use full image
  width_ = this->declare_parameter("width", 0);
  height_ = this->declare_parameter("height", 0);
  offset_x_ = this->declare_parameter("offset_x", 0);
  offset_y_ = this->declare_parameter("offset_y", 0);

  // default: CropDecimate_NN
  int interpolation = this->declare_parameter("interpolation", 0);
  interpolation_ = static_cast<CropDecimateModes>(interpolation);

  pub_ = image_transport::create_camera_publisher(this, "out/image_raw");
  sub_ = image_transport::create_camera_subscription(
    this, "in/image_raw", std::bind(
      &CropDecimateNode::imageCb, this,
      std::placeholders::_1, std::placeholders::_2), "raw");
}

void CropDecimateNode::imageCb(
  const sensor_msgs::msg::Image::ConstSharedPtr image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
{
  /// @todo Check image dimensions match info_msg

  if (pub_.getNumSubscribers() < 1) {
    return;
  }

  int decimation_x = decimation_x_;
  int decimation_y = decimation_y_;

  // Compute the ROI we'll actually use
  bool is_bayer = sensor_msgs::image_encodings::isBayer(image_msg->encoding);

  if (is_bayer) {
    // Odd offsets for Bayer images basically change the Bayer pattern, but that's
    // unnecessarily complicated to support
    offset_x_ &= ~0x1;
    offset_y_ &= ~0x1;
    width_ &= ~0x1;
    height_ &= ~0x1;
  }

  int max_width = image_msg->width - offset_x_;

  if (max_width <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "x offset is outside the input image width: "
      "%i, x offset: %i.", image_msg->width, offset_x_);
    return;
  }

  int max_height = image_msg->height - offset_y_;

  if (max_height <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "y offset is outside the input image height: "
      "%i, y offset: %i", image_msg->height, offset_y_);
    return;
  }

  int width = width_;
  int height = height_;

  if (width == 0 || width > max_width) {
    width = max_width;
  }

  if (height == 0 || height > max_height) {
    height = max_height;
  }

  // On no-op, just pass the messages along
  if (
    decimation_x == 1 && decimation_y == 1 &&
    offset_x_ == 0 && offset_y_ == 0 &&
    width == static_cast<int>(image_msg->width) &&
    height == static_cast<int>(image_msg->height))
  {
    pub_.publish(image_msg, info_msg);
    return;
  }

  // Get a cv::Mat view of the source data
  CvImageConstPtr source = toCvShare(image_msg);

  // Except in Bayer downsampling case, output has same encoding as the input
  CvImage output(source->header, source->encoding);
  // Apply ROI (no copy, still a view of the image_msg data)
  output.image = source->image(cv::Rect(offset_x_, offset_y_, width, height));

  // Special case: when decimating Bayer images, we first do a 2x2 decimation to BGR
  if (is_bayer && (decimation_x > 1 || decimation_y > 1)) {
    if (decimation_x % 2 != 0 || decimation_y % 2 != 0) {
      RCLCPP_ERROR(
        get_logger(),
        "Odd decimation not supported for Bayer images");
      return;
    }

    cv::Mat bgr;
    int step = output.image.step1();
    if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_RGGB8) {
      debayer2x2toBGR<uint8_t>(output.image, bgr, 0, 1, step, step + 1);
    } else if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_BGGR8) {
      debayer2x2toBGR<uint8_t>(output.image, bgr, step + 1, 1, step, 0);
    } else if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_GBRG8) {
      debayer2x2toBGR<uint8_t>(output.image, bgr, step, 0, step + 1, 1);
    } else if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_GRBG8) {
      debayer2x2toBGR<uint8_t>(output.image, bgr, 1, 0, step + 1, step);
    } else if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_RGGB16) {
      debayer2x2toBGR<uint16_t>(output.image, bgr, 0, 1, step, step + 1);
    } else if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_BGGR16) {
      debayer2x2toBGR<uint16_t>(output.image, bgr, step + 1, 1, step, 0);
    } else if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_GBRG16) {
      debayer2x2toBGR<uint16_t>(output.image, bgr, step, 0, step + 1, 1);
    } else if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_GRBG16) {
      debayer2x2toBGR<uint16_t>(output.image, bgr, 1, 0, step + 1, step);
    } else {
      RCLCPP_ERROR(
        get_logger(), "Unrecognized Bayer encoding '%s'",
        image_msg->encoding.c_str());
      return;
    }

    output.image = bgr;
    output.encoding = (bgr.depth() == CV_8U) ? sensor_msgs::image_encodings::BGR8 :
      sensor_msgs::image_encodings::BGR16;
    decimation_x /= 2;
    decimation_y /= 2;
  }

  // Apply further downsampling, if necessary
  if (decimation_x > 1 || decimation_y > 1) {
    cv::Mat decimated;

    if (interpolation_ == image_proc::CropDecimateModes::CropDecimate_NN) {
      // Use optimized method instead of OpenCV's more general NN resize
      int pixel_size = output.image.elemSize();

      switch (pixel_size) {
        // Currently support up through 4-channel float
        case 1:
          decimate<1>(output.image, decimated, decimation_x, decimation_y);
          break;
        case 2:
          decimate<2>(output.image, decimated, decimation_x, decimation_y);
          break;
        case 3:
          decimate<3>(output.image, decimated, decimation_x, decimation_y);
          break;
        case 4:
          decimate<4>(output.image, decimated, decimation_x, decimation_y);
          break;
        case 6:
          decimate<6>(output.image, decimated, decimation_x, decimation_y);
          break;
        case 8:
          decimate<8>(output.image, decimated, decimation_x, decimation_y);
          break;
        case 12:
          decimate<12>(output.image, decimated, decimation_x, decimation_y);
          break;
        case 16:
          decimate<16>(output.image, decimated, decimation_x, decimation_y);
          break;
        default:
          RCLCPP_ERROR(
            get_logger(),
            "Unsupported pixel size, %d bytes", pixel_size);
          return;
      }
    } else {
      // Linear, cubic, area, ...
      cv::Size size(output.image.cols / decimation_x, output.image.rows / decimation_y);
      cv::resize(output.image, decimated, size, 0.0, 0.0, static_cast<int>(interpolation_));
    }

    output.image = decimated;
  }

  // Create output Image message
  /// @todo Could save copies by allocating this above and having output.image alias it
  sensor_msgs::msg::Image::SharedPtr out_image = output.toImageMsg();

  // Create updated CameraInfo message
  sensor_msgs::msg::CameraInfo::SharedPtr out_info =
    std::make_shared<sensor_msgs::msg::CameraInfo>(*info_msg);
  int binning_x = std::max(static_cast<int>(info_msg->binning_x), 1);
  int binning_y = std::max(static_cast<int>(info_msg->binning_y), 1);
  out_info->binning_x = binning_x * decimation_x_;
  out_info->binning_y = binning_y * decimation_y_;
  out_info->roi.x_offset += offset_x_ * binning_x;
  out_info->roi.y_offset += offset_y_ * binning_y;
  out_info->roi.height = height * binning_y;
  out_info->roi.width = width * binning_x;

  // If no ROI specified, leave do_rectify as-is. If ROI specified, set do_rectify = true.
  if (width != static_cast<int>(image_msg->width) ||
    height != static_cast<int>(image_msg->height))
  {
    out_info->roi.do_rectify = true;
  }

  if (!target_frame_id_.empty()) {
    out_image->header.frame_id = target_frame_id_;
    out_info->header.frame_id = target_frame_id_;
  }

  pub_.publish(out_image, out_info);
}

}  // namespace image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_proc::CropDecimateNode)
