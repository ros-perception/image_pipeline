#include "image_transport/publisher_plugin.h"
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv_latest/CvBridge.h>
#include <theora_image_transport/packet.h>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

namespace theora_image_transport {

class CompressedPublisher : public image_transport::PublisherPlugin
{
public:
  CompressedPublisher();
  virtual ~CompressedPublisher();

  virtual std::string getTransportType() const;

  virtual void advertise(ros::NodeHandle& nh, const std::string& base_topic,
                         uint32_t queue_size, bool latch);

  virtual uint32_t getNumSubscribers() const;
  virtual std::string getTopic() const;

  virtual void publish(const sensor_msgs::Image& message) const;

  virtual void shutdown();

protected:
  void sendHeader(const ros::SingleSubscriberPublisher& pub) const;
  void ensure_encoding_context(const CvSize &size) const;
  void oggPacketToMsg(const ogg_packet &oggpacket, theora_image_transport::packet &msgOutput) const;

  ros::NodeHandle nh_;
  ros::Publisher pub_;

  //I have some reservations about making everything mutable like this but
  //from the users perspective the publisher is essentially stateless except for differences in
  //bitrate of the resulting stream and required image format.  Thus in order to match the
  //ros API in image_publisher the publish method is still const

  mutable sensor_msgs::CvBridge img_bridge_;
  mutable th_enc_ctx* encoding_context_;
  mutable std::vector<ogg_packet> stream_header_;

  //Offsets to make image size into multiple of 16 (with alignment of image data to even pixels I believe)
  mutable int nearestWidth;
  mutable int nearestHeight;
  mutable int nearestXoff;
  mutable int nearestYoff;
};

} //namespace compressed_image_transport
