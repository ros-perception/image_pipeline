#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <image_proc/CropDecimateConfig.h>

/// @todo ROI logic

namespace image_proc {

class CropDecimateNodelet : public nodelet::Nodelet
{
  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_in_, it_out_;
  image_transport::CameraSubscriber sub_;
  image_transport::CameraPublisher pub_;
  int queue_size_;

  // Dynamic reconfigure
  typedef image_proc::CropDecimateConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  void configCb(Config &config, uint32_t level);
};

void CropDecimateNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  ros::NodeHandle nh_in (nh, "camera");
  ros::NodeHandle nh_out(nh, "camera_out");
  it_in_ .reset(new image_transport::ImageTransport(nh_in));
  it_out_.reset(new image_transport::ImageTransport(nh_out));

  // Read parameters
  private_nh.param("queue_size", queue_size_, 5);

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&CropDecimateNodelet::connectCb, this);
  ros::SubscriberStatusCallback connect_cb_info = boost::bind(&CropDecimateNodelet::connectCb, this);
  pub_ = it_out_->advertiseCamera("image_raw",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);

  // Set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(private_nh));
  ReconfigureServer::CallbackType f = boost::bind(&CropDecimateNodelet::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);
}

// Handles (un)subscribing when clients (un)subscribe
void CropDecimateNodelet::connectCb()
{
  if (pub_.getNumSubscribers() == 0)
    sub_.shutdown();
  else if (!sub_)
    sub_ = it_in_->subscribeCamera("image_raw", queue_size_, &CropDecimateNodelet::imageCb, this);
}

void CropDecimateNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if (config_.decimation_x == 1 && config_.decimation_y == 1)
  {
    pub_.publish(image_msg, info_msg);
    return;
  }

  /// @todo Figure out if 2x1, 3x2 etc. binning of bayer images makes sense
  if (config_.decimation_x % 2 != 0 || config_.decimation_y % 2 != 0)
  {
    NODELET_ERROR_THROTTLE(2, "Odd binning not supported yet");
    return;
  }

  /// @todo Support non-bayer images, duh...
  if (!sensor_msgs::image_encodings::isBayer(image_msg->encoding))
  {
    NODELET_ERROR_THROTTLE(2, "Only Bayer encodings supported currenty");
    return;
  }

  /// @todo Check image dimensions match info_msg

  // Create updated CameraInfo message
  sensor_msgs::CameraInfoPtr out_info = boost::make_shared<sensor_msgs::CameraInfo>(*info_msg);
  out_info->binning_x = std::max((int)out_info->binning_x, 1) * config_.decimation_x;
  out_info->binning_y = std::max((int)out_info->binning_y, 1) * config_.decimation_y;

  // Allocate new image message
  sensor_msgs::ImagePtr out_image = boost::make_shared<sensor_msgs::Image>();
  out_image->header   = image_msg->header;
  out_image->height   = image_msg->height / config_.decimation_y;
  out_image->width    = image_msg->width  / config_.decimation_x;
  out_image->encoding = sensor_msgs::image_encodings::BGR8; // for bayer
  out_image->step     = out_image->width * 3;
  out_image->data.resize(out_image->height * out_image->step);

  // Compute offsets to color elements in a 2x2 bayer pixel group
  int R, G1, G2, B;
  if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_RGGB8)
  {
    R  = 0;
    G1 = 1;
    G2 = image_msg->step;
    B  = image_msg->step + 1;
  }
  else if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_BGGR8)
  {
    B  = 0;
    G1 = 1;
    G2 = image_msg->step;
    R  = image_msg->step + 1;
  }
  else if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_GBRG8)
  {
    G1 = 0;
    B  = 1;
    R  = image_msg->step;
    G2 = image_msg->step + 1;
  }
  else if (image_msg->encoding == sensor_msgs::image_encodings::BAYER_GRBG8)
  {
    G1 = 0;
    R  = 1;
    B  = image_msg->step;
    G2 = image_msg->step + 1;
  }
  else
  {
    NODELET_ERROR_THROTTLE(2, "Unrecognized Bayer encoding '%s'", image_msg->encoding.c_str());
    return;
  }

  // Hit only the pixel groups we need
  int bayer_step = config_.decimation_y * image_msg->step;
  int bayer_skip = config_.decimation_x;
  const uint8_t* bayer_row = &image_msg->data[0];
  uint8_t* bgr_buffer = &out_image->data[0];

  // Downsample and debayer at once
  for (int y = 0; y < (int)out_image->height; ++y, bayer_row += bayer_step)
  {
    const uint8_t* bayer_buffer = bayer_row;
    for (int x = 0; x < (int)out_image->width; ++x, bayer_buffer += bayer_skip, bgr_buffer += 3)
    {
      bgr_buffer[0] = bayer_buffer[B];
      bgr_buffer[1] = (bayer_buffer[G1] + bayer_buffer[G2]) / 2;
      bgr_buffer[2] = bayer_buffer[R];
    }
  }

  pub_.publish(out_image, out_info);
}

void CropDecimateNodelet::configCb(Config &config, uint32_t level)
{
  config_ = config;
}

} // namespace image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(image_proc, crop_decimate, image_proc::CropDecimateNodelet, nodelet::Nodelet)
