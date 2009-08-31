#include "image_transport/subscriber_plugin.h"
#include <theora_image_transport/packet.h>
#include <opencv_latest/CvBridge.h>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

namespace theora_image_transport {

class TheoraSubscriber : public image_transport::SubscriberPlugin
{
public:
  TheoraSubscriber();
  virtual ~TheoraSubscriber();

  virtual void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                         const Callback& callback, const ros::VoidPtr& tracked_object,
                         const ros::TransportHints& transport_hints);
  
  virtual std::string getTopic() const;

  virtual void shutdown();

private:
  void msgToOggPacket(const theora_image_transport::packet &msg, ogg_packet &oggpacketOutput);

  ros::Subscriber sub_;

  bool received_header_;
  th_dec_ctx* decoding_context_;
  th_info header_info_;
  th_comment header_comment_;
  th_setup_info* setup_info_;
  sensor_msgs::CvBridge img_bridge_;

  void decompress(const theora_image_transport::packetConstPtr& message, const Callback& callback);
};

} //namespace theora_image_transport
