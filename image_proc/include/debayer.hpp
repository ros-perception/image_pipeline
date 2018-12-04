
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>

#include <ament_index_cpp/get_resource.hpp>
#include "edge_aware.h"
#include "rclcpp/rclcpp.hpp"
#include "../src/visibility.h"
#ifndef IMAGE_PROC_DEBAYER_HPP
#define IMAGE_PROC_DEBAYER_HPP


namespace image_proc
{
class DebayerNode : public rclcpp::Node
{
  // ROS communication
  public:
    IMAGE_PROC_PUBLIC DebayerNode();
  private:
    image_transport::Subscriber sub_raw_;
    std::string camera_namespace_;
    std::mutex connect_mutex_;
    image_transport::Publisher pub_mono_;
    image_transport::Publisher pub_color_;
    void connectCb();
    void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & raw_msg);
};
} // namespace image_proc
#endif