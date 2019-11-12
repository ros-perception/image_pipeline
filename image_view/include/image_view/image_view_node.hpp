#ifndef IMAGE_VIEW__IMAGE_VIEW_NODE_HPP_
#define IMAGE_VIEW__IMAGE_VIEW_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <boost/format.hpp>

#include <memory>
#include <mutex>
#include <thread>
#include <string>

namespace image_view
{

class ThreadSafeImage
{
  std::mutex mutex_;
  std::condition_variable condition_;
  cv::Mat image_;

public:
  void set(const cv::Mat & image);
  cv::Mat get();
  cv::Mat pop();
};

class ImageViewNode
  : public rclcpp::Node
{
  ThreadSafeImage queued_image_, shown_image_;
  bool autosize_;
  bool g_do_dynamic_scaling;
  bool g_gui;
  boost::format filename_format_;
  boost::format g_filename_format;
  cv::Mat g_last_image;
  double g_max_image_value;
  double g_min_image_value;
  image_transport::Subscriber sub_;
  int count_;
  int g_colormap;
  int g_count;
  rclcpp::TimerBase::SharedPtr gui_timer_;
  std::mutex g_image_mutex;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_;
  std::string window_name_;
  std::thread window_thread_;

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  static void mouseCb(int event, int x, int y, int flags, void * param);
  static void guiCb();
  void windowThread();
  
public:
  explicit ImageViewNode(const rclcpp::NodeOptions & options);
  ~ImageViewNode();
};

}  // namespace image_view

#endif  // IMAGE_VIEW__IMAGE_VIEW_NODE_HPP_
