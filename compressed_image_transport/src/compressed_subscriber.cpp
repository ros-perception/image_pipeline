#include "compressed_image_transport/compressed_subscriber.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv_latest/CvBridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

namespace compressed_image_transport {

CompressedSubscriber::CompressedSubscriber()
{
}

CompressedSubscriber::~CompressedSubscriber()
{
}

void
CompressedSubscriber::subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                                const Callback& callback, const ros::VoidPtr& tracked_object,
                                const ros::TransportHints& transport_hints)
{
  typedef boost::function<void(const sensor_msgs::CompressedImageConstPtr&)> InternalCallback;
  InternalCallback decompress_fn = boost::bind(&CompressedSubscriber::decompress, this, _1, callback);
  sub_ = nh.subscribe<>(base_topic, queue_size, decompress_fn, tracked_object, transport_hints);
}

std::string CompressedSubscriber::getTopic() const
{
  return sub_.getTopic();
}

void CompressedSubscriber::shutdown()
{
  sub_.shutdown();
}

void CompressedSubscriber::decompress(const sensor_msgs::CompressedImageConstPtr& message,
                                      const Callback& callback)
{
  // Decompress
  const CvMat compressed = cvMat(1, message->data.size(), CV_8UC1,
                                 const_cast<unsigned char*>(&message->data[0]));
  cv::WImageBuffer_b decompressed( cvDecodeImage(&compressed, CV_LOAD_IMAGE_ANYCOLOR) );

  // Copy into ROS image message
  boost::shared_ptr<sensor_msgs::Image> image_ptr(new sensor_msgs::Image);
  if ( !sensor_msgs::CvBridge::fromIpltoRosImage(decompressed.Ipl(), *image_ptr) ) {
    ROS_ERROR("Unable to create image message");
    return;
  }
  image_ptr->header = message->header;
  // @todo: don't assume 8-bit channels
  if (decompressed.Channels() == 1) {
    image_ptr->encoding = sensor_msgs::image_encodings::MONO8;
  }
  else if (decompressed.Channels() == 3) {
    image_ptr->encoding = sensor_msgs::image_encodings::RGB8;
  }
  else {
    ROS_ERROR("Unsupported number of channels: %i", decompressed.Channels());
    return;
  }
  
  callback(image_ptr);
}

} //namespace compressed_image_transport
