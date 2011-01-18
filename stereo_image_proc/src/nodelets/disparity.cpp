#include "stereo_image_proc/nodelets/disparity.h"
#include <image_proc/advertisement_checker.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/calib3d/calib3d.hpp>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(stereo_image_proc, disparity,
                        stereo_image_proc::DisparityNodelet, nodelet::Nodelet)

namespace stereo_image_proc {

using namespace sensor_msgs;
using namespace stereo_msgs;

struct DisparityNodelet::Impl
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  // Subscriptions
  /// @todo Implement (optional) approx synch of left and right cameras
  image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  message_filters::TimeSynchronizer<Image, CameraInfo, Image, CameraInfo> sync_;
  bool subscribed_;

  // Publications
  ros::Publisher pub_disparity_;

  /// @todo Dynamic reconfigure
  
  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;
  cv::StereoBM block_matcher_; // contains scratch buffers for block matching

  // Error reporting
  image_proc::AdvertisementChecker check_inputs_;

  Impl(const ros::NodeHandle& nh, const std::string& name)
    : nh_(nh),
      it_(nh),
      sync_(3), /// @todo Parameter for sync queue size
      subscribed_(false),
      check_inputs_(nh, name)
  {
  }
};

void DisparityNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  impl_ = boost::make_shared<Impl>(nh, getName());
  ros::SubscriberStatusCallback connect_cb = boost::bind(&DisparityNodelet::connectCb, this);
  impl_->pub_disparity_  = nh.advertise<DisparityImage>("disparity",  1,
                                                        connect_cb, connect_cb);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  impl_->sync_.connectInput(impl_->sub_l_image_, impl_->sub_l_info_,
                            impl_->sub_r_image_, impl_->sub_r_info_);
  impl_->sync_.registerCallback(boost::bind(&DisparityNodelet::imageCb, this,
                                            _1, _2, _3, _4));

  // Print a warning every minute until the input topics are advertised
  ros::V_string topics;
  topics.push_back("left/image_rect");
  topics.push_back("left/camera_info");
  topics.push_back("right/image_rect");
  topics.push_back("right/camera_info");
  impl_->check_inputs_.start(topics, 60.0);
}

// Handles (un)subscribing when clients (un)subscribe
void DisparityNodelet::connectCb()
{
  if (impl_->pub_disparity_.getNumSubscribers() == 0)
  {
    impl_->sub_l_image_.unsubscribe();
    impl_->sub_l_info_ .unsubscribe();
    impl_->sub_r_image_.unsubscribe();
    impl_->sub_r_info_ .unsubscribe();
    impl_->subscribed_ = false;
  }
  else if (!impl_->subscribed_)
  {
    impl_->sub_l_image_.subscribe(impl_->it_, "left/image_rect", 1);
    impl_->sub_l_info_ .subscribe(impl_->nh_, "left/camera_info", 1);
    impl_->sub_r_image_.subscribe(impl_->it_, "right/image_rect", 1);
    impl_->sub_r_info_ .subscribe(impl_->nh_, "right/camera_info", 1);
    impl_->subscribed_ = true;
  }
  /// @todo Parameter for queue size
}

void DisparityNodelet::imageCb(const ImageConstPtr& l_image_msg,
                               const CameraInfoConstPtr& l_info_msg,
                               const ImageConstPtr& r_image_msg,
                               const CameraInfoConstPtr& r_info_msg)
{
  assert(l_image_msg->encoding == sensor_msgs::image_encodings::MONO8);
  assert(r_image_msg->encoding == sensor_msgs::image_encodings::MONO8);

  // Update the camera model
  impl_->model_.fromCameraInfo(l_info_msg, r_info_msg);
  
  // Allocate new disparity image message
  DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
  disp_msg->header = l_image_msg->header;
  disp_msg->image.header = l_image_msg->header;
  disp_msg->image.height = l_image_msg->height;
  disp_msg->image.width  = l_image_msg->width;
  disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  disp_msg->image.step = disp_msg->image.width * sizeof(float);
  disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);

  // Stereo parameters
  disp_msg->f = impl_->model_.right().fx();
  disp_msg->T = impl_->model_.baseline();

  /// @todo Window of (potentially) valid disparities

  /// @todo Disparity search range
  //disparity.min_disparity = getMinDisparity();
  //disparity.max_disparity = getMinDisparity() + getDisparityRange() - 1;
  //disparity.delta_d = 1.0 / 16; // OpenCV uses 16 disparities per pixel

  // Create cv::Mat views onto all buffers
  const cv::Mat_<uint8_t> l_image(l_image_msg->height, l_image_msg->width,
                                  const_cast<uint8_t*>(&l_image_msg->data[0]),
                                  l_image_msg->step);
  const cv::Mat_<uint8_t> r_image(r_image_msg->height, r_image_msg->width,
                                  const_cast<uint8_t*>(&r_image_msg->data[0]),
                                  r_image_msg->step);
  cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
                             reinterpret_cast<float*>(&disp_msg->image.data[0]),
                             disp_msg->image.step);

  // Perform block matching to find the disparities
  impl_->block_matcher_(l_image, r_image, disp_image, CV_32F);

  // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
  double cx_l = impl_->model_.left().cx();
  double cx_r = impl_->model_.right().cx();
  if (cx_l != cx_r)
    cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);

  impl_->pub_disparity_.publish(disp_msg);
}

} // namespace stereo_image_proc
