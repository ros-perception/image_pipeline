#include "image_transport/publisher_plugin.h"

namespace compressed_image_transport {

class CompressedPublisher : public image_transport::PublisherPlugin
{
public:
  virtual ~CompressedPublisher();

  virtual std::string getTransportType() const;

  virtual void advertise(ros::NodeHandle& nh, const std::string& base_topic,
                         uint32_t queue_size, bool latch);

  virtual uint32_t getNumSubscribers() const;
  virtual std::string getTopic() const;

  virtual void publish(const sensor_msgs::Image& message) const;

  virtual void shutdown();

protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};

} //namespace compressed_image_transport
