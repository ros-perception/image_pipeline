#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <image_proc/CropDecimateConfig.h>

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
  /// @todo Check image dimensions match info_msg
  /// @todo Publish tweaks to config_ so they appear in reconfigure_gui
  
  /// @todo Could support odd offsets for Bayer images, but it's a tad complicated
  config_.x_offset = (config_.x_offset / 2) * 2;
  config_.y_offset = (config_.y_offset / 2) * 2;

  int max_width = image_msg->width - config_.x_offset;
  int max_height = image_msg->height - config_.y_offset;
  int width = config_.width;
  int height = config_.height;
  if (width == 0 || width > max_width)
    width = max_width;
  if (height == 0 || height > max_height)
    height = max_height;

  // On no-op, just pass the messages along
  if (config_.decimation_x == 1  &&
      config_.decimation_y == 1  &&
      config_.x_offset == 0      &&
      config_.y_offset == 0      &&
      width  == (int)image_msg->width &&
      height == (int)image_msg->height)
  {
    pub_.publish(image_msg, info_msg);
    return;
  }

  /// @todo Support 16-bit encodings
  if (sensor_msgs::image_encodings::bitDepth(image_msg->encoding) != 8)
  {
    NODELET_ERROR_THROTTLE(2, "Only 8-bit encodings are currently supported");
    return;
  }
  
  // Create updated CameraInfo message
  sensor_msgs::CameraInfoPtr out_info = boost::make_shared<sensor_msgs::CameraInfo>(*info_msg);
  int binning_x = std::max((int)info_msg->binning_x, 1);
  int binning_y = std::max((int)info_msg->binning_y, 1);
  out_info->binning_x = binning_x * config_.decimation_x;
  out_info->binning_y = binning_y * config_.decimation_y;
  out_info->roi.x_offset += config_.x_offset * binning_x;
  out_info->roi.y_offset += config_.y_offset * binning_y;
  out_info->roi.height = height * binning_y;
  out_info->roi.width = width * binning_x;

  // Create new image message
  sensor_msgs::ImagePtr out_image = boost::make_shared<sensor_msgs::Image>();
  out_image->header = image_msg->header;
  out_image->height = height / config_.decimation_y;
  out_image->width  = width  / config_.decimation_x;
  // Don't know encoding, step, or data size yet

  if (config_.decimation_x == 1 && config_.decimation_y == 1)
  {
    // Crop only, preserving original encoding
    int num_channels = sensor_msgs::image_encodings::numChannels(image_msg->encoding);
    out_image->encoding = image_msg->encoding;
    out_image->step = out_image->width * num_channels;
    out_image->data.resize(out_image->height * out_image->step);

    const uint8_t* input_buffer = &image_msg->data[config_.y_offset*image_msg->step + config_.x_offset*num_channels];
    uint8_t* output_buffer = &out_image->data[0];
    for (int y = 0; y < (int)out_image->height; ++y)
    {
      memcpy(output_buffer, input_buffer, out_image->step);
      input_buffer += image_msg->step;
      output_buffer += out_image->step;
    }
  }
  else if (sensor_msgs::image_encodings::isMono(image_msg->encoding))
  {
    // Output is also monochrome
    out_image->encoding = sensor_msgs::image_encodings::MONO8;
    out_image->step = out_image->width;
    out_image->data.resize(out_image->height * out_image->step);

    // Hit only the pixel groups we need
    int input_step = config_.decimation_y * image_msg->step;
    int input_skip = config_.decimation_x;
    const uint8_t* input_row = &image_msg->data[config_.y_offset*image_msg->step + config_.x_offset];
    uint8_t* output_buffer = &out_image->data[0];

    // Downsample
    for (int y = 0; y < (int)out_image->height; ++y, input_row += input_step)
    {
      const uint8_t* input_buffer = input_row;
      for (int x = 0; x < (int)out_image->width; ++x, input_buffer += input_skip, ++output_buffer)
        *output_buffer = *input_buffer;
    }
  }
  else
  {
    // Output is color
    out_image->encoding = sensor_msgs::image_encodings::BGR8;
    out_image->step     = out_image->width * 3;
    out_image->data.resize(out_image->height * out_image->step);

    if (sensor_msgs::image_encodings::isBayer(image_msg->encoding))
    {
      if (config_.decimation_x % 2 != 0 || config_.decimation_y % 2 != 0)
      {
        NODELET_ERROR_THROTTLE(2, "Odd decimation not supported for Bayer images");
        return;
      }
      
      // Compute offsets to color elements
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
      int input_step = config_.decimation_y * image_msg->step;
      int input_skip = config_.decimation_x;
      const uint8_t* input_row = &image_msg->data[config_.y_offset*image_msg->step + config_.x_offset];
      uint8_t* output_buffer = &out_image->data[0];

      // Downsample and debayer at once
      for (int y = 0; y < (int)out_image->height; ++y, input_row += input_step)
      {
        const uint8_t* input_buffer = input_row;
        for (int x = 0; x < (int)out_image->width; ++x, input_buffer += input_skip, output_buffer += 3)
        {
          output_buffer[0] = input_buffer[B];
          output_buffer[1] = (input_buffer[G1] + input_buffer[G2]) / 2;
          output_buffer[2] = input_buffer[R];
        }
      }
    }
    else
    {
      // Support RGB-type encodings
      int R, G, B, channels;
      if (image_msg->encoding == sensor_msgs::image_encodings::BGR8)
      {
        B = 0;
        G = 1;
        R = 2;
        channels = 3;
      }
      else if (image_msg->encoding == sensor_msgs::image_encodings::RGB8)
      {
        R = 0;
        G = 1;
        B = 2;
        channels = 3;
      }
      else if (image_msg->encoding == sensor_msgs::image_encodings::BGRA8)
      {
        B = 0;
        G = 1;
        R = 2;
        channels = 4;
      }
      else if (image_msg->encoding == sensor_msgs::image_encodings::RGBA8)
      {
        R = 0;
        G = 1;
        B = 2;
        channels = 4;
      }
      else
      {
        NODELET_ERROR_THROTTLE(2, "Unsupported encoding '%s'", image_msg->encoding.c_str());
        return;
      }

      // Hit only the pixel groups we need
      int input_step = config_.decimation_y * image_msg->step;
      int input_skip = config_.decimation_x * channels;
      const uint8_t* input_row = &image_msg->data[config_.y_offset*image_msg->step + config_.x_offset*channels];
      uint8_t* output_buffer = &out_image->data[0];

      // Downsample
      for (int y = 0; y < (int)out_image->height; ++y, input_row += input_step)
      {
        const uint8_t* input_buffer = input_row;
        for (int x = 0; x < (int)out_image->width; ++x, input_buffer += input_skip, output_buffer += 3)
        {
          output_buffer[0] = input_buffer[B];
          output_buffer[1] = input_buffer[G];
          output_buffer[2] = input_buffer[R];
        }
      }

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
