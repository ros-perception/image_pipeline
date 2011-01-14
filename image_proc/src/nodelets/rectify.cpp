#include "image_proc/nodelets/rectify.h"
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/CvBridge.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(image_proc, rectify, image_proc::RectifyNodelet, nodelet::Nodelet)

namespace image_proc {

struct RectifyNodelet::Impl
{
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  image_transport::Publisher pub_rect_;
  /// @todo May want to allow sharing camera model with other nodelets (e.g. stereo processing)
  image_geometry::PinholeCameraModel model_;
  int interpolation_;

  Impl(const ros::NodeHandle& nh)
    : it_(nh),
      interpolation_(CV_INTER_LINEAR)
  {
  }
};

void RectifyNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  impl_ = boost::make_shared<Impl>(nh);
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&RectifyNodelet::connectCb, this);
  impl_->pub_rect_  = impl_->it_.advertise("image_rect",  1, connect_cb, connect_cb);

  /// @todo Check input topics
}

// Handles (un)subscribing when clients (un)subscribe
void RectifyNodelet::connectCb()
{
  if (impl_->pub_rect_.getNumSubscribers() == 0)
    impl_->sub_camera_.shutdown();
  else if (!impl_->sub_camera_)
    impl_->sub_camera_ = impl_->it_.subscribeCamera("image_mono", 3, &RectifyNodelet::imageCb, this);
  /// @todo Parameter for queue size
}

void RectifyNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                             const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // Verify camera is actually calibrated
  if (info_msg->K[0] == 0.0) {
    ROS_ERROR_THROTTLE_NAMED(30, getName(),
                             "Rectified topic '%s' requested but camera publishing '%s' "
                             "is uncalibrated", impl_->pub_rect_.getTopic().c_str(),
                             impl_->sub_camera_.getInfoTopic().c_str());
    return;
  }

  // If zero distortion, just pass the message along
  if (info_msg->D.empty() || info_msg->D[0] == 0.0)
  {
    impl_->pub_rect_.publish(image_msg);
    return;
  }

  // Update the camera model
  impl_->model_.fromCameraInfo(info_msg);
  
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
  impl_->model_.rectifyImage(image, rect, impl_->interpolation_);
  impl_->pub_rect_.publish(rect_msg);
}

} // namespace image_proc
