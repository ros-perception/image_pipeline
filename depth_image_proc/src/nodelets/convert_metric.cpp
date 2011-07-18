#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

namespace depth_image_proc {

namespace enc = sensor_msgs::image_encodings;

class ConvertMetricNodelet : public nodelet::Nodelet
{
  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber sub_raw_;
  bool subscribed_;

  // Publications
  image_transport::Publisher pub_depth_;

  virtual void onInit();

  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& raw_msg);
};

void ConvertMetricNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Monitor whether anyone is subscribed to the output
  subscribed_ = false;
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&ConvertMetricNodelet::connectCb, this);
  pub_depth_ = it_->advertise("image", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void ConvertMetricNodelet::connectCb()
{
  if (pub_depth_.getNumSubscribers() == 0)
  {
    sub_raw_.shutdown();
    subscribed_ = false;
  }
  else if (!subscribed_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_raw_ = it_->subscribe("image_raw", 1, &ConvertMetricNodelet::depthCb, this, hints);
    subscribed_ = true;
  }
}

void ConvertMetricNodelet::depthCb(const sensor_msgs::ImageConstPtr& raw_msg)
{
  if (raw_msg->encoding != enc::TYPE_16UC1)
  {
    NODELET_ERROR("Expected data of type [%s], got [%s]", enc::TYPE_16UC1.c_str(),
                  raw_msg->encoding.c_str());
    return;
  }

  // Allocate new Image message
  sensor_msgs::ImagePtr depth_msg( new sensor_msgs::Image );
  depth_msg->header   = raw_msg->header;
  depth_msg->encoding = enc::TYPE_32FC1;
  depth_msg->height   = raw_msg->height;
  depth_msg->width    = raw_msg->width;
  depth_msg->step     = raw_msg->width * sizeof (float);
  depth_msg->data.resize( depth_msg->height * depth_msg->step);

  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  // Fill in the depth image data, converting mm to m
  const uint16_t* raw_data = reinterpret_cast<const uint16_t*>(&raw_msg->data[0]);
  float* depth_data = reinterpret_cast<float*>(&depth_msg->data[0]);
  for (unsigned index = 0; index < depth_msg->height * depth_msg->width; ++index)
  {
    uint16_t raw = raw_data[index];
    depth_data[index] = (raw == 0) ? bad_point : (float)raw * 0.001f;
  }

  pub_depth_.publish(depth_msg);
}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (depth_image_proc, convert_metric, depth_image_proc::ConvertMetricNodelet, nodelet::Nodelet);
