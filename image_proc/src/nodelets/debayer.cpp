#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <image_proc/DebayerConfig.h>

#include <opencv2/imgproc/imgproc.hpp>
// Until merged into OpenCV
#include "edge_aware.h"
#include "yuv422.h"

#include <boost/make_shared.hpp>

namespace image_proc {

namespace enc = sensor_msgs::image_encodings;

class DebayerNodelet : public nodelet::Nodelet
{
  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber sub_raw_;
  
  boost::mutex connect_mutex_;
  image_transport::Publisher pub_mono_;
  image_transport::Publisher pub_color_;

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef image_proc::DebayerConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& raw_msg);

  void configCb(Config &config, uint32_t level);
};

void DebayerNodelet::onInit()
{
  ros::NodeHandle &nh         = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  ReconfigureServer::CallbackType f = boost::bind(&DebayerNodelet::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  typedef image_transport::SubscriberStatusCallback ConnectCB;
  ConnectCB connect_cb = boost::bind(&DebayerNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_XXX
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_mono_  = it_->advertise("image_mono",  1, connect_cb, connect_cb);
  pub_color_ = it_->advertise("image_color", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void DebayerNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_mono_.getNumSubscribers() == 0 && pub_color_.getNumSubscribers() == 0)
    sub_raw_.shutdown();
  else if (!sub_raw_)
    sub_raw_ = it_->subscribe("image_raw", 1, &DebayerNodelet::imageCb, this);
}

void DebayerNodelet::imageCb(const sensor_msgs::ImageConstPtr& raw_msg)
{
  /// @todo Could simplify this whole method by explicitly constructing a map
  /// from raw encoding to OpenCV cvtColor code
  
  if (enc::isMono(raw_msg->encoding))
  {
    // For monochrome, no processing needed!
    pub_mono_.publish(raw_msg);
    pub_color_.publish(raw_msg);
    
    // Warn if the user asked for color
    if (pub_color_.getNumSubscribers() > 0)
    {
      NODELET_WARN_THROTTLE(30, 
                            "Color topic '%s' requested, but raw image data from topic '%s' is grayscale",
                            pub_color_.getTopic().c_str(), sub_raw_.getTopic().c_str());
    }
  }
  else if (enc::isColor(raw_msg->encoding))
  {
    pub_color_.publish(raw_msg);
    
    // Convert to monochrome if needed
    if (pub_mono_.getNumSubscribers() > 0)
    {
      int bit_depth    = enc::bitDepth(raw_msg->encoding);
      int num_channels = enc::numChannels(raw_msg->encoding);
      int code = -1;
      if (raw_msg->encoding == enc::BGR8 ||
          raw_msg->encoding == enc::BGR16)
        code = CV_BGR2GRAY;
      else if (raw_msg->encoding == enc::RGB8 ||
               raw_msg->encoding == enc::RGB16)
        code = CV_RGB2GRAY;
      else if (raw_msg->encoding == enc::BGRA8 ||
               raw_msg->encoding == enc::BGRA16)
        code = CV_BGRA2GRAY;
      else if (raw_msg->encoding == enc::RGBA8 ||
               raw_msg->encoding == enc::RGBA16)
        code = CV_RGBA2GRAY;

      sensor_msgs::ImagePtr gray_msg = boost::make_shared<sensor_msgs::Image>();
      gray_msg->header   = raw_msg->header;
      gray_msg->height   = raw_msg->height;
      gray_msg->width    = raw_msg->width;
      gray_msg->encoding = bit_depth == 8 ? enc::MONO8 : enc::MONO16;
      gray_msg->step     = gray_msg->width * (bit_depth / 8);
      gray_msg->data.resize(gray_msg->height * gray_msg->step);

      int type = bit_depth == 8 ? CV_8U : CV_16U;
      const cv::Mat color(raw_msg->height, raw_msg->width, CV_MAKETYPE(type, num_channels),
                          const_cast<uint8_t*>(&raw_msg->data[0]), raw_msg->step);
      cv::Mat gray(gray_msg->height, gray_msg->width, CV_MAKETYPE(type, 1),
                   &gray_msg->data[0], gray_msg->step);
      cv::cvtColor(color, gray, code);

      pub_mono_.publish(gray_msg);
    }
  }
  else if (enc::isBayer(raw_msg->encoding)) {
    int bit_depth = enc::bitDepth(raw_msg->encoding);
    int type = bit_depth == 8 ? CV_8U : CV_16U;
    const cv::Mat bayer(raw_msg->height, raw_msg->width, CV_MAKETYPE(type, 1),
                        const_cast<uint8_t*>(&raw_msg->data[0]), raw_msg->step);
    
    if (pub_mono_.getNumSubscribers() > 0)
    {
      int code = -1;
      if (raw_msg->encoding == enc::BAYER_RGGB8 ||
          raw_msg->encoding == enc::BAYER_RGGB16)
        code = CV_BayerBG2GRAY;
      else if (raw_msg->encoding == enc::BAYER_BGGR8 ||
               raw_msg->encoding == enc::BAYER_BGGR16)
        code = CV_BayerRG2GRAY;
      else if (raw_msg->encoding == enc::BAYER_GBRG8 ||
               raw_msg->encoding == enc::BAYER_GBRG16)
        code = CV_BayerGR2GRAY;
      else if (raw_msg->encoding == enc::BAYER_GRBG8 ||
               raw_msg->encoding == enc::BAYER_GRBG16)
        code = CV_BayerGB2GRAY;

      sensor_msgs::ImagePtr gray_msg = boost::make_shared<sensor_msgs::Image>();
      gray_msg->header   = raw_msg->header;
      gray_msg->height   = raw_msg->height;
      gray_msg->width    = raw_msg->width;
      gray_msg->encoding = bit_depth == 8 ? enc::MONO8 : enc::MONO16;
      gray_msg->step     = gray_msg->width * (bit_depth / 8);
      gray_msg->data.resize(gray_msg->height * gray_msg->step);

      cv::Mat gray(gray_msg->height, gray_msg->width, CV_MAKETYPE(type, 1),
                   &gray_msg->data[0], gray_msg->step);
      cv::cvtColor(bayer, gray, code);
      
      pub_mono_.publish(gray_msg);
    }

    if (pub_color_.getNumSubscribers() > 0)
    {
      sensor_msgs::ImagePtr color_msg = boost::make_shared<sensor_msgs::Image>();
      color_msg->header   = raw_msg->header;
      color_msg->height   = raw_msg->height;
      color_msg->width    = raw_msg->width;
      color_msg->encoding = bit_depth == 8? enc::BGR8 : enc::BGR16;
      color_msg->step     = color_msg->width * 3 * (bit_depth / 8);
      color_msg->data.resize(color_msg->height * color_msg->step);

      cv::Mat color(color_msg->height, color_msg->width, CV_MAKETYPE(type, 3),
                    &color_msg->data[0], color_msg->step);

      int algorithm;
      {
        boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
        algorithm = config_.debayer;
      }
      
      if (algorithm == Debayer_EdgeAware ||
          algorithm == Debayer_EdgeAwareWeighted)
      {
        // These algorithms are not in OpenCV yet
        if (raw_msg->encoding != enc::BAYER_GRBG8)
        {
          NODELET_WARN_THROTTLE(30, "Edge aware algorithms currently only support GRBG8 Bayer. "
                                "Falling back to bilinear interpolation.");
          algorithm = Debayer_Bilinear;
        }
        else
        {
          if (algorithm == Debayer_EdgeAware)
            debayerEdgeAware(bayer, color);
          else
            debayerEdgeAwareWeighted(bayer, color);
        }
      }
      if (algorithm == Debayer_Bilinear ||
          algorithm == Debayer_VNG)
      {
        int code = -1;
        if (raw_msg->encoding == enc::BAYER_RGGB8 ||
            raw_msg->encoding == enc::BAYER_RGGB16)
          code = CV_BayerBG2BGR;
        else if (raw_msg->encoding == enc::BAYER_BGGR8 ||
                 raw_msg->encoding == enc::BAYER_BGGR16)
          code = CV_BayerRG2BGR;
        else if (raw_msg->encoding == enc::BAYER_GBRG8 ||
                 raw_msg->encoding == enc::BAYER_GBRG16)
          code = CV_BayerGR2BGR;
        else if (raw_msg->encoding == enc::BAYER_GRBG8 ||
                 raw_msg->encoding == enc::BAYER_GRBG16)
          code = CV_BayerGB2BGR;

        if (algorithm == Debayer_VNG)
          code += CV_BayerBG2BGR_VNG - CV_BayerBG2BGR;

        cv::cvtColor(bayer, color, code);
      }
      
      pub_color_.publish(color_msg);
    }
  }
  else if (raw_msg->encoding == enc::YUV422)
  {
    const cv::Mat yuv(raw_msg->height, raw_msg->width, CV_8UC2,
                      const_cast<uint8_t*>(&raw_msg->data[0]), raw_msg->step);
    
    if (pub_mono_.getNumSubscribers() > 0)
    {
      sensor_msgs::ImagePtr gray_msg = boost::make_shared<sensor_msgs::Image>();
      gray_msg->header   = raw_msg->header;
      gray_msg->height   = raw_msg->height;
      gray_msg->width    = raw_msg->width;
      gray_msg->encoding = enc::MONO8;
      gray_msg->step     = gray_msg->width;
      gray_msg->data.resize(gray_msg->height * gray_msg->step);

      cv::Mat gray(gray_msg->height, gray_msg->width, CV_8UC1,
                   &gray_msg->data[0], gray_msg->step);
      yuv422ToGray(yuv, gray);

      pub_mono_.publish(gray_msg);
    }

    if (pub_color_.getNumSubscribers() > 0)
    {
      sensor_msgs::ImagePtr color_msg = boost::make_shared<sensor_msgs::Image>();
      color_msg->header   = raw_msg->header;
      color_msg->height   = raw_msg->height;
      color_msg->width    = raw_msg->width;
      color_msg->encoding = enc::BGR8;
      color_msg->step     = color_msg->width * 3;
      color_msg->data.resize(color_msg->height * color_msg->step);

      cv::Mat color(color_msg->height, color_msg->width, CV_8UC3,
                    &color_msg->data[0], color_msg->step);
      yuv422ToColor(yuv, color);

      pub_color_.publish(color_msg);
    }
  }
  else if (raw_msg->encoding == enc::TYPE_8UC3)
  {
    // 8UC3 does not specify a color encoding. Is it BGR, RGB, HSV, XYZ, LUV...?
    NODELET_ERROR_THROTTLE(10,
                           "Raw image topic '%s' has ambiguous encoding '8UC3'. The "
                           "source should set the encoding to 'bgr8' or 'rgb8'.",
                           sub_raw_.getTopic().c_str());
  }
  else
  {
    NODELET_ERROR_THROTTLE(10, "Raw image topic '%s' has unsupported encoding '%s'",
                           sub_raw_.getTopic().c_str(), raw_msg->encoding.c_str());
  }
}

void DebayerNodelet::configCb(Config &config, uint32_t level)
{
  config_ = config;
}

} // namespace image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(image_proc, debayer, image_proc::DebayerNodelet, nodelet::Nodelet)
