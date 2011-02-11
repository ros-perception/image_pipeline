#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_proc/advertisement_checker.h>

#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/CvBridge.h>

namespace image_proc {

class RectifyNodelet : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_camera_;
  image_transport::Publisher pub_rect_;
  boost::shared_ptr<AdvertisementChecker> check_inputs_;
  image_geometry::PinholeCameraModel model_;
  int interpolation_; /// @todo dynamic_reconfigure for interpolation
  int queue_size_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
  
public:
  RectifyNodelet() : interpolation_(CV_INTER_LINEAR) {}
};

void RectifyNodelet::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Read parameters
  private_nh.param("queue_size", queue_size_, 5);

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&RectifyNodelet::connectCb, this);
  pub_rect_  = it_->advertise("image_rect",  1, connect_cb, connect_cb);

  // Print a warning every minute until the input topics are advertised
  ros::V_string topics;
  topics.push_back("image_mono");
  topics.push_back("camera_info");
  check_inputs_.reset( new AdvertisementChecker(nh, getName()) );
  check_inputs_->start(topics, 60.0);
}

// Handles (un)subscribing when clients (un)subscribe
void RectifyNodelet::connectCb()
{
  if (pub_rect_.getNumSubscribers() == 0)
    sub_camera_.shutdown();
  else if (!sub_camera_)
    sub_camera_ = it_->subscribeCamera("image_mono", queue_size_, &RectifyNodelet::imageCb, this);
}

void RectifyNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                             const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // Verify camera is actually calibrated
  if (info_msg->K[0] == 0.0) {
    NODELET_ERROR_THROTTLE(30, "Rectified topic '%s' requested but camera publishing '%s' "
                           "is uncalibrated", pub_rect_.getTopic().c_str(),
                           sub_camera_.getInfoTopic().c_str());
    return;
  }

  // If zero distortion, just pass the message along
  if (info_msg->D.empty() || info_msg->D[0] == 0.0)
  {
    pub_rect_.publish(image_msg);
    return;
  }

  // Update the camera model
  model_.fromCameraInfo(info_msg);
  
  // Allocate new rectified image message
  sensor_msgs::ImagePtr rect_msg = boost::make_shared<sensor_msgs::Image>();
  rect_msg->header   = image_msg->header;
  rect_msg->height   = image_msg->height;
  rect_msg->width    = image_msg->width;
  rect_msg->encoding = image_msg->encoding;
  rect_msg->step     = image_msg->step;
  rect_msg->data.resize(rect_msg->height * rect_msg->step);

  // Create cv::Mat views onto both buffers
  sensor_msgs::CvBridge image_bridge, rect_bridge;
  const cv::Mat image = image_bridge.imgMsgToCv(image_msg);
  cv::Mat rect = rect_bridge.imgMsgToCv(rect_msg);

  // Rectify and publish
  model_.rectifyImage(image, rect, interpolation_);
  pub_rect_.publish(rect_msg);
}

} // namespace image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(image_proc, rectify, image_proc::RectifyNodelet, nodelet::Nodelet)
