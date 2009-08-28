#include "theora_image_transport/compressed_publisher.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv/highgui.h>

#include <vector>
#include <stdio.h> //for memcpy
#define null 0

using namespace std;

namespace theora_image_transport
{

CompressedPublisher::CompressedPublisher()
{
  encoding_context_ = null;
}

CompressedPublisher::~CompressedPublisher()
{
  if (encoding_context_ != null)
    th_encode_free(encoding_context_);
}

std::string CompressedPublisher::getTransportType() const
{
  return "theora";
}

void CompressedPublisher::advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, bool latch)
{
  nh_ = nh;
  pub_ = nh_.advertise<theora_image_transport::packet> (topic, queue_size, boost::bind(&CompressedPublisher::sendHeader, this, _1),
                                                                     ros::SubscriberStatusCallback(), ros::VoidPtr(), latch);
}

uint32_t CompressedPublisher::getNumSubscribers() const
{
  return pub_.getNumSubscribers();
}

std::string CompressedPublisher::getTopic() const
{
  return pub_.getTopic();
}

//Sends the header packets to new subscribers (unless there are no header packets to send yet)
void CompressedPublisher::sendHeader(const ros::SingleSubscriberPublisher& pub) const
{
  theora_image_transport::packet msg;
  for (unsigned int i = 0; i < stream_header_.size(); i++)
  {
    oggPacketToMsg(stream_header_[i], msg);
    pub.publish(msg);
    ROS_DEBUG("Published header packet, sleeping for 0.1 second");
    ros::Duration d = ros::Duration(0, 100000000);
    d.sleep();
  }
}

void CompressedPublisher::publish(const sensor_msgs::Image& message) const
{
  if (img_bridge_.fromImage(message, "bgr8"))
  {
    IplImage* img = img_bridge_.toIpl();
    //cvShowImage(window_name_.c_str(), img);
    IplImage* img2 = cvCreateImage(cvGetSize(img), img->depth, img->nChannels);
    cvCvtColor(img, img2, CV_BGR2YCrCb);
    ensure_encoding_context(cvGetSize(img2));

    //convert image
    th_ycbcr_buffer ycbcr_image;
    vector<unsigned char> planes[3]; //color planes
    for (int i = 0; i < 3; i++) //Size is number of pixels in padded image (chroma subsampled by factor of 2 in each dimension, which means area is 1/4)
      planes[i].resize(nearestWidth * nearestHeight / (i > 0 ? 4 : 1));

    //ROS_DEBUG("Image size: %d image width: %d image height: %d", img2->imageSize, img2->width, img2->height);
    for (int i = 0; i < img2->imageSize / 3; i++) //imageSize/3 is the number of pixels
    {
      planes[0][i] = *(unsigned char*)(img2->imageData + i * 3); //Y
      //planes[2][i] = *(unsigned char*)(img2->imageData + i*3 + 1);  //Cr
      //planes[1][i] = *(unsigned char*)(img2->imageData + i*3 + 2);  //Cb
    }

    //Note that while OpenCV uses YCbCr, theora uses YCrCb... this can make things a little confusing
    //CHROMA subsampling
    for (int planeIdx = 1; planeIdx < 3; planeIdx++)
    {
      int swappedIdx = planeIdx;
      if (planeIdx == 1)
        swappedIdx = 2;
      else if (planeIdx == 2)
        swappedIdx = 1;
      for (int i = 0; i < img2->width; i += 2)
        for (int j = 0; j < img2->height; j += 2)
        {
          int planeDataIdx = i / 2 + (j * img2->width) / 4;
          //planes[2][planeDataIdx]
          unsigned int total = (unsigned int)((uchar*)(img2->imageData + img2->widthStep * j))[i * 3 + planeIdx];
          unsigned int count = 1;
          if (i < img2->width - 1)
          {
            total += (unsigned int)((uchar*)(img2->imageData + img2->widthStep * j))[(i + 1) * 3 + planeIdx];
            count++;
          }
          if (j < img2->height - 1)
          {
            total += (unsigned int)((uchar*)(img2->imageData + img2->widthStep * (j + 1)))[i * 3 + planeIdx];
            count++;
          }
          if (i < img2->width - 1 && j < img2->height - 1)
          {
            total += (unsigned int)((uchar*)(img2->imageData + img2->widthStep * (j + 1)))[(i + 1) * 3 + planeIdx];
            count++;
          }
          planes[swappedIdx][planeDataIdx] = (uchar)(total / count);
        }
    }

    for (int i = 0; i < 3; i++)
    {
      ycbcr_image[i].width = nearestWidth / (i > 0 ? 2 : 1); //
      ycbcr_image[i].height = nearestHeight / (i > 0 ? 2 : 1); //chroma is subsampled by a factor of 2
      ycbcr_image[i].stride = img2->width / (i > 0 ? 2 : 1); //
      ycbcr_image[i].data = &planes[i][0];
    }
    ROS_DEBUG("Width: %d, Height: %d, xOff: %d, yOff: %d", nearestWidth, nearestHeight, nearestXoff, nearestYoff);

    cvReleaseImage(&img2);

    int rval;
    if (encoding_context_ == null)
      ROS_DEBUG("About to encode with null encoding context.");
    rval = th_encode_ycbcr_in(encoding_context_, ycbcr_image);
    if (rval == TH_EFAULT)
      ROS_DEBUG("EFault in encoding.");
    if (rval == TH_EINVAL)
      ROS_DEBUG("EInval in encoding.");
    ROS_DEBUG("Encoding resulted in: %d", rval);

    ogg_packet oggpacket;
    theora_image_transport::packet output;
    ROS_DEBUG("Ready to get encoded packets.");
    while ((rval = th_encode_packetout(encoding_context_, 0, &oggpacket)) > 0)
    {
      oggPacketToMsg(oggpacket, output);
      ROS_DEBUG("Publishing packet!");
      pub_.publish(output);
    }
    ROS_DEBUG("Punted from while loop with rval %d", rval);
  }
  else
    ROS_ERROR("Unable to convert from %s to bgr", message.encoding.c_str());
}

void CompressedPublisher::shutdown()
{
  pub_.shutdown();
}

void CompressedPublisher::ensure_encoding_context(const CvSize &size) const
{
  if (encoding_context_ == null)
  {
    th_info encoder_setup;
    th_info_init(&encoder_setup);
    nearestWidth = size.width + size.width % 16 == 0 ? 0 : (16 - size.width % 16);
    nearestHeight = size.height + size.height % 16 == 0 ? 0 : (16 - size.height % 16);

    /* Theora has a divisible-by-sixteen restriction for the encoded frame size */
    /* scale the picture size up to the nearest /16 and calculate offsets */
    nearestWidth = size.width + 15 & ~0xF;
    nearestHeight = size.height + 15 & ~0xF;
    /*Force the offsets to be even so that chroma samples line up like we
     expect.*/
    nearestXoff = nearestWidth - size.width >> 1 & ~1;
    nearestYoff = nearestHeight - size.height >> 1 & ~1;

    encoder_setup.frame_width = nearestWidth;
    encoder_setup.frame_height = nearestHeight;
    encoder_setup.pic_width = size.width;
    encoder_setup.pic_height = size.height;
    encoder_setup.pic_x = nearestXoff;
    encoder_setup.pic_y = nearestYoff;
    ROS_DEBUG("Creating context with Width: %d, Height: %d", nearestWidth, nearestHeight);
    encoder_setup.colorspace = TH_CS_UNSPECIFIED;
    //encoder_setup.colorspace = TH_CS_ITU_REC_470M;     //TH_CS_ITU_REC_470M     A color space designed for NTSC content.
    //TH_CS_ITU_REC_470BG     A color space designed for PAL/SECAM content.
    encoder_setup.pixel_fmt = TH_PF_420; //see bottom of http://www.theora.org/doc/libtheora-1.1beta1/codec_8h.html
    int bitrate;
    nh_.param("theora_bitrate", bitrate, 800000);
    encoder_setup.target_bitrate =
    //encoder_setup.quality = 63;    //On a scale of 0 to 63, to use this set target bitrate to 0
        encoder_setup.aspect_numerator = 1;
    encoder_setup.aspect_denominator = 1;
    encoder_setup.fps_numerator = 0;
    encoder_setup.fps_denominator = 0;
    encoder_setup.keyframe_granule_shift = 6; //Apparently a good default

    encoding_context_ = th_encode_alloc(&encoder_setup);

    if (encoding_context_ == null)
      ROS_DEBUG("No encoding context immediately after alloc.");

    th_comment comment;
    th_comment_init(&comment);
    th_comment_add(&comment, (char*)"Compression node written by Ethan.");
    comment.vendor = (char*)"Encoded by Willow Garage image_compression_node.";

    if (encoding_context_ == null)
      ROS_DEBUG("Encoding context not successfully created.");

    ogg_packet oggpacket;
    while (th_encode_flushheader(encoding_context_, &comment, &oggpacket) > 0)
    {
      stream_header_.push_back(oggpacket);
      //Have to deep copy the packet since theora owns the packet memory, not doing this causes nasty bugs and is very hard to track down!!!
      stream_header_.back().packet = new unsigned char[oggpacket.bytes];
      memcpy(stream_header_.back().packet, oggpacket.packet, oggpacket.bytes);
    }
    //ROS_DEBUG("Published %d header packets.", stream_header_.size());
    //th_comment_clear(&comment);  //TODO: this should happen but is causing crazy seg faults (probably trying to free a string literal)

    //Stream the header in case anyone is already listening
    theora_image_transport::packet msg;
    for (unsigned int i = 0; i < stream_header_.size(); i++)
    {
      oggPacketToMsg(stream_header_[i], msg);
      pub_.publish(msg);
      ROS_DEBUG("Published header packet, sleeping for 0.1 second");
      ros::Duration d = ros::Duration(0, 100000000);
      d.sleep();
    }

    if (encoding_context_ == null)
      ROS_DEBUG("Encoding context killed by header flushing.");
  }
}

void CompressedPublisher::oggPacketToMsg(const ogg_packet &oggpacket, theora_image_transport::packet &msgOutput) const
{
  msgOutput.blob.resize(oggpacket.bytes);
  memcpy(&msgOutput.blob[0], oggpacket.packet, oggpacket.bytes);
  msgOutput.bytes = oggpacket.bytes;
  msgOutput.b_o_s = oggpacket.b_o_s;
  msgOutput.e_o_s = oggpacket.e_o_s;
  msgOutput.granulepos = oggpacket.granulepos;
  msgOutput.packetno = oggpacket.packetno;
  //ROS_DEBUG("Ready to send %d bytes in packet#%d and granule%d (and this is BOS: %d).", msgOutput.bytes, msgOutput.packetno, msgOutput.granulepos, msgOutput.b_o_s);
  /*unsigned int i = 0;
  for (int j = 0; j < msgOutput.bytes; j++)
    i = i * 2 % 91 + msgOutput.blob[j];
  ROS_DEBUG("Checksum is: %d", i);*/
}

} //namespace compressed_image_transport
