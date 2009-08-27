#include "image_transport/subscriber_plugin.h"
#include <sensor_msgs/CompressedImage.h>

namespace compressed_image_transport {

class CompressedSubscriber : public image_transport::SubscriberPlugin
{
public:
  CompressedSubscriber();
  
  virtual ~CompressedSubscriber();

  virtual void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                         const Callback& callback, const ros::VoidPtr& tracked_object,
                         const ros::TransportHints& transport_hints);
  
  virtual std::string getTopic() const;

  virtual void shutdown();

private:
  ros::Subscriber sub_;

  void decompress(const sensor_msgs::CompressedImageConstPtr& message, const Callback& callback);
};

} //namespace image_transport
