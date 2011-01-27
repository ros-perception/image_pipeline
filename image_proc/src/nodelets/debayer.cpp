#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_proc/advertisement_checker.h>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/make_shared.hpp>

namespace image_proc {

namespace enc = sensor_msgs::image_encodings;

class DebayerNodelet : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber sub_raw_;
  image_transport::Publisher pub_mono_;
  image_transport::Publisher pub_color_;
  boost::shared_ptr<AdvertisementChecker> check_inputs_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& raw_msg);
};

void DebayerNodelet::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Monitor whether anyone is subscribed to the output
  typedef image_transport::SubscriberStatusCallback ConnectCB;
  ConnectCB connect_cb = boost::bind(&DebayerNodelet::connectCb, this);
  pub_mono_  = it_->advertise("image_mono",  1, connect_cb, connect_cb);
  pub_color_ = it_->advertise("image_color", 1, connect_cb, connect_cb);

  // Print a warning every minute until the image topic is advertised
  ros::V_string topics;
  topics.push_back("image_raw");
  check_inputs_.reset( new AdvertisementChecker(nh, getName()) );
  check_inputs_->start(topics, 60.0);
}

// Handles (un)subscribing when clients (un)subscribe
void DebayerNodelet::connectCb()
{
  if (pub_mono_.getNumSubscribers() == 0 && pub_color_.getNumSubscribers() == 0)
    sub_raw_.shutdown();
  else if (!sub_raw_)
    sub_raw_ = it_->subscribe("image_raw", 3, &DebayerNodelet::imageCb, this);
  /// @todo Parameter for queue size
}

void DebayerNodelet::imageCb(const sensor_msgs::ImageConstPtr& raw_msg)
{
  // Special case when raw image is already monochrome, as no processing is needed.
  if (raw_msg->encoding == enc::MONO8)
  {
    // Warn if the user asked for color
    if (pub_color_.getNumSubscribers() > 0)
    {
      NODELET_WARN_THROTTLE(30, 
                            "Color topic '%s' requested, but raw image data from topic '%s' is grayscale",
                            pub_color_.getTopic().c_str(), sub_raw_.getTopic().c_str());
    }
    pub_mono_.publish(raw_msg);
    pub_color_.publish(raw_msg);
    return;
  }

  // 8UC3 does not specify a color encoding. Is it BGR, RGB, HSV, XYZ, LUV...?
  if (raw_msg->encoding == enc::TYPE_8UC3) {
    NODELET_ERROR_THROTTLE(30,
                           "Raw image topic '%s' has ambiguous encoding '8UC3'. The "
                           "source should set the encoding to 'bgr8' or 'rgb8'.",
                           sub_raw_.getTopic().c_str());
    return;
  }

  sensor_msgs::ImageConstPtr color_msg = raw_msg;

  // Bayer case: convert to BGR, then convert that to monochrome if needed
  /// @todo Better to convert directly to monochrome, but OpenCV doesn't support it yet
  if (raw_msg->encoding.find("bayer") != std::string::npos) {
    // Figure out the conversion needed
    int code = 0;
    if (raw_msg->encoding == enc::BAYER_RGGB8)
      code = CV_BayerBG2BGR;
    else if (raw_msg->encoding == enc::BAYER_BGGR8)
      code = CV_BayerRG2BGR;
    else if (raw_msg->encoding == enc::BAYER_GBRG8)
      code = CV_BayerGR2BGR;
    else if (raw_msg->encoding == enc::BAYER_GRBG8)
      code = CV_BayerGB2BGR;
    else {
      NODELET_ERROR_THROTTLE(30, "Raw image topic '%s' has unsupported encoding '%s'",
                             sub_raw_.getTopic().c_str(), raw_msg->encoding.c_str());
      return;
    }

    // Allocate new color image message
    sensor_msgs::ImagePtr out_color = boost::make_shared<sensor_msgs::Image>();
    out_color->header   = raw_msg->header;
    out_color->height   = raw_msg->height;
    out_color->width    = raw_msg->width;
    out_color->encoding = enc::BGR8;
    out_color->step     = out_color->width * 3;
    out_color->data.resize(out_color->height * out_color->step);

    // Construct cv::Mats pointing to raw bayer data and color data
    const cv::Mat bayer(raw_msg->height, raw_msg->width, CV_8UC1,
                        const_cast<uint8_t*>(&raw_msg->data[0]), raw_msg->step);
    cv::Mat color(out_color->height, out_color->width, CV_8UC3,
                  &out_color->data[0], out_color->step);

    // Do conversion directly into out_color data buffer
    cv::cvtColor(bayer, color, code);
    color_msg = out_color;
  }
  // The only remaining encodings we handle are RGB8 and BGR8
  else if (raw_msg->encoding != enc::RGB8 && raw_msg->encoding != enc::BGR8)
  {
    NODELET_ERROR_THROTTLE(30, "Raw image topic '%s' has unsupported encoding '%s'",
                           sub_raw_.getTopic().c_str(), raw_msg->encoding.c_str());
    return;
  }

  pub_color_.publish(color_msg);

  // Create monochrome image if needed
  if (pub_mono_.getNumSubscribers() > 0)
  {
    sensor_msgs::ImagePtr gray_msg = boost::make_shared<sensor_msgs::Image>();
    gray_msg->header   = raw_msg->header;
    gray_msg->height   = raw_msg->height;
    gray_msg->width    = raw_msg->width;
    gray_msg->encoding = enc::MONO8;
    gray_msg->step     = gray_msg->width;
    gray_msg->data.resize(gray_msg->height * gray_msg->step);
      
    const cv::Mat color(color_msg->height, color_msg->width, CV_8UC3,
                        const_cast<uint8_t*>(&color_msg->data[0]), color_msg->step);
    cv::Mat gray(gray_msg->height, gray_msg->width, CV_8UC1,
                 &gray_msg->data[0], gray_msg->step);
    int code = (color_msg->encoding == enc::BGR8) ? CV_BGR2GRAY : CV_RGB2GRAY;
    cv::cvtColor(color, gray, code);

    pub_mono_.publish(gray_msg);
  }
}

} // namespace image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(image_proc, debayer, image_proc::DebayerNodelet, nodelet::Nodelet)
