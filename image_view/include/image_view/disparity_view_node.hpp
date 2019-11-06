#ifndef IMAGE_VIEW__DISPARITY_VIEW_NODE_HPP_
#define IMAGE_VIEW__DISPARITY_VIEW_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include <memory>
#include <string>

namespace image_view
{

class DisparityViewNode
  : public rclcpp::Node
{
  // colormap for disparities, RGB order
  static unsigned char colormap[];

  std::string window_name_;
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr sub_;
  cv::Mat_<cv::Vec3b> disparity_color_;
  bool initialized;

  virtual void onInit();

  void imageCb(const stereo_msgs::msg::DisparityImage::SharedPtr& msg);

  public:
  explicit DisparityViewNode(const rclcpp::NodeOptions & options);
  ~DisparityViewNode();
};

}  // namespace image_view

#endif  // IMAGE_VIEW__DISPARITY_VIEW_NODE_HPP_
