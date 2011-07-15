#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

namespace depth_image_proc {

namespace enc = sensor_msgs::image_encodings;

class DisparityNodelet : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_depth_;
  ros::Publisher pub_disparity_;
  int queue_size_;

  virtual void onInit();

  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
};

void DisparityNodelet::onInit()
{
  ros::NodeHandle &nh         = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Read parameters
  private_nh.param("queue_size", queue_size_, 5);

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&DisparityNodelet::connectCb, this);
  pub_disparity_ = nh.advertise<stereo_msgs::DisparityImage>("disparity", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void DisparityNodelet::connectCb()
{
  if (pub_disparity_.getNumSubscribers() == 0)
    sub_depth_.shutdown();
  else if (!sub_depth_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_depth_ = it_->subscribeCamera("image_rect_raw", queue_size_, &DisparityNodelet::depthCb, this, hints);
  }
}

void DisparityNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                               const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  /// @todo Support float (metric) depth images also?
  if (depth_msg->encoding != enc::TYPE_16UC1)
  {
    NODELET_ERROR("Expected data of type [%s], got [%s]", enc::TYPE_16UC1.c_str(),
                  depth_msg->encoding.c_str());
    return;
  }

  // Allocate new DisparityImage message
  stereo_msgs::DisparityImagePtr disp_msg( new stereo_msgs::DisparityImage );
  disp_msg->header         = depth_msg->header;
  disp_msg->image.header   = disp_msg->header;
  disp_msg->image.encoding = enc::TYPE_32FC1;
  disp_msg->image.height   = depth_msg->height;
  disp_msg->image.width    = depth_msg->width;
  disp_msg->image.step     = disp_msg->image.width * sizeof (float);
  disp_msg->image.data.resize( disp_msg->image.height * disp_msg->image.step, 0.0f );
  disp_msg->T = 0.075; /// @todo Get baseline from CameraInfo instead of hardcoding
  disp_msg->f = info_msg->P[0];
  /// @todo Get these values from somewhere... parameters?
  disp_msg->min_disparity = disp_msg->f * disp_msg->T / 4.0;
  disp_msg->max_disparity = disp_msg->f * disp_msg->T / 0.5;
  disp_msg->delta_d = 0.125;

  // For each depth Z, disparity d = fT / Z
  float constant = disp_msg->f * disp_msg->T * 1000.0;

  const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
  float* disp_data = reinterpret_cast<float*>(&disp_msg->image.data[0]);
  for (unsigned index = 0; index < depth_msg->height * depth_msg->width; ++index)
  {
    uint16_t depth = depth_data[index];
    if (depth != 0)
      disp_data[index] = constant / depth;
  }

  pub_disparity_.publish(disp_msg);
}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (depth_image_proc, disparity, depth_image_proc::DisparityNodelet, nodelet::Nodelet);
