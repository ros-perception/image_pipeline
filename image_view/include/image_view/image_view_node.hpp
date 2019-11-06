#ifndef IMAGE_VIEW__IMAGE_VIEW_NODE_HPP_
#define IMAGE_VIEW__IMAGE_VIEW_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <mutex>
#include <thread>

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
  image_transport::Subscriber sub_;

  std::thread window_thread_;

  ThreadSafeImage queued_image_, shown_image_;

  std::string window_name_;
  bool autosize_;
  boost::format filename_format_;
  int count_;

  ros::WallTimer gui_timer_;

  virtual void onInit();

  void imageCb(const sensor_msgs::msg::Image::SharedPtr& msg);

  static void mouseCb(int event, int x, int y, int flags, void * param);
  static void guiCb(const ros::WallTimerEvent&);

  void windowThread();
  
public:
  explicit ImageViewNode(const rclcpp::NodeOptions & options);
  ~ImageViewNode();
};

}  // namespace image_view

#endif  // IMAGE_VIEW__IMAGE_VIEW_NODE_HPP_
