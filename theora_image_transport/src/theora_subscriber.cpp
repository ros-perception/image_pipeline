#include "theora_image_transport/theora_subscriber.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#include <vector>

#define null 0

using namespace std;

namespace theora_image_transport {

TheoraSubscriber::TheoraSubscriber()
{
  decoding_context_ = null;
  received_header_ = false;
  setup_info_ = null;
  th_info_init(&header_info_);
}

TheoraSubscriber::~TheoraSubscriber()
{
}


void TheoraSubscriber::subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                                     const Callback& callback, const ros::VoidPtr& tracked_object,
                                     const ros::TransportHints& transport_hints)
{
  typedef boost::function<void(const theora_image_transport::packetConstPtr&)> InternalCallback;
  InternalCallback decompress_fn = boost::bind(&TheoraSubscriber::decompress, this, _1, callback);
  sub_ = nh.subscribe<>(base_topic, queue_size, decompress_fn, tracked_object, transport_hints);
}

std::string TheoraSubscriber::getTopic() const
{
  return sub_.getTopic();
}

void TheoraSubscriber::shutdown()
{
  sub_.shutdown();
}

//When using this caller is responsible for deleting oggpacket.packet!!
void TheoraSubscriber::msgToOggPacket(const theora_image_transport::packet &msg, ogg_packet &oggpacketOutput)
{
  oggpacketOutput.bytes = msg.bytes;
  oggpacketOutput.b_o_s = msg.b_o_s;
  oggpacketOutput.e_o_s = msg.e_o_s;
  oggpacketOutput.granulepos = msg.granulepos;
  oggpacketOutput.packetno = msg.packetno;
  oggpacketOutput.packet = new unsigned char[msg.bytes];
  memcpy(oggpacketOutput.packet, &msg.blob[0], msg.bytes);

  //ROS_DEBUG("Received %d bytes in packet#%d and granule%d (and this is BOS: %d).", oggpacketOutput.bytes, oggpacketOutput.packetno, oggpacketOutput.granulepos, oggpacketOutput.b_o_s);
  /*unsigned int i = 0;
  for (int j = 0; j < msg.bytes; j++)
    i = i * 2 % 91 + oggpacketOutput.packet[j];
  ROS_DEBUG("Checksum is: %d", i);*/
}

void TheoraSubscriber::decompress(const theora_image_transport::packetConstPtr& message, const Callback& callback)
{
  const theora_image_transport::packet &pkt = *message;
  ogg_packet oggpacket;
  msgToOggPacket(pkt, oggpacket);
  sensor_msgs::Image *rosMsg = new sensor_msgs::Image();

  if (received_header_ == false) //still receiving header info
  {
    if ((int)oggpacket.packetno == 999999)
    {
      ROS_DEBUG("Dropping flush packet.");
      return;
    }

    /*if(oggpacket.packetno != 0)
     {
     ROS_DEBUG("Dumping header packet because packet# is non-zero: %d.", (int)oggpacket.packetno);
     return;
     }*/
    //static th_setup_info* setup_info_ptr = null;
    //static th_setup_info** setup_info_ = &setup_info_ptr;
    ROS_DEBUG("Setup_info: %p", setup_info_);
    int rval = th_decode_headerin(&header_info_, &header_comment_, &setup_info_, &oggpacket);
    ROS_DEBUG("Setup_info: %p", setup_info_);
    if (rval == 0)
    {
      ROS_DEBUG("This should happen on correct receipt of a header packet but never seems to in practice");
      //received_header_ = true;
      //decoding_context_ = th_decode_alloc(&header_info_, setup_info_);
    }
    else if (rval == TH_EFAULT)
      ROS_DEBUG("EFault when processing header.");
    else if (rval == TH_EBADHEADER) //Oddly, I seem to always get one of these...
      ROS_DEBUG("Bad header when processing header.");
    else if (rval == TH_EVERSION)
      ROS_DEBUG("Bad version when processing header.");
    else if (rval == TH_ENOTFORMAT)
      ROS_DEBUG("Received packet which was not a Theora header.");
    else if (rval < 0)
      ROS_DEBUG("Error code when processing header: %d.", rval);

    if (setup_info_ != null)  //Because rval != 0 as specified in the docs, this is used instead to check that
    {                         // the header has been fully received
      received_header_ = true;
      decoding_context_ = th_decode_alloc(&header_info_, setup_info_);
    }
  }

  if (received_header_ == true)
  {
    int rval = th_decode_packetin(decoding_context_, &oggpacket, null);

    if (rval == 0) //Successfully got a frame
    {
      th_ycbcr_buffer ycbcr_image;
      th_decode_ycbcr_out(decoding_context_, ycbcr_image);

      //convert image to IplImage
      IplImage* img = cvCreateImage(cvSize(ycbcr_image[0].width, ycbcr_image[0].height), IPL_DEPTH_8U, 3);

      //Do scaling of chroma planes, straight copy for Y plane
      for (int planeIdx = 0; planeIdx < 3; planeIdx++)
      {
        int swappedIdx = planeIdx;
        if (planeIdx == 1)
          swappedIdx = 2;
        else if (planeIdx == 2)
          swappedIdx = 1;
        for (int i = 0; i < img->width; i++)
          for (int j = 0; j < img->height; j++)
          {
            int ci = planeIdx > 0 ? i / 2 : i; //Do simple pixel to 2x2 block scaling
            int cj = planeIdx > 0 ? j / 2 : j;
            ((uchar*)(img->imageData + img->widthStep * j))[i * 3 + planeIdx] = ycbcr_image[swappedIdx].data[ci + cj * ycbcr_image[swappedIdx].stride];
          }
      }

      IplImage* img2 = cvCreateImage(cvGetSize(img), img->depth, img->nChannels);
      cvCvtColor(img, img2, CV_YCrCb2BGR);

      img_bridge_.fromIpltoRosImage(img2, *rosMsg);

      cvReleaseImage(&img);
      cvReleaseImage(&img2);
    }
    else if (rval == TH_DUPFRAME)
      ROS_DEBUG("Got a duplicate frame.");
    else
      ROS_DEBUG("Error code when decoding packet: %d.", rval);
  }

  delete oggpacket.packet;

  //The shared pointer will take care of freeing rosMsg
  boost::shared_ptr<sensor_msgs::Image> image_ptr(rosMsg);

  //Manually set encoding to be correct
  //TODO: the packet message could be extended with a flag that indicates the original type for better handling of
  //      B&W images
  image_ptr->encoding = sensor_msgs::image_encodings::BGR8;
  callback(image_ptr);
}

} //namespace theora_image_transport
