#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_proc/advertisement_checker.h>

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>

#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <stereo_image_proc/DisparityConfig.h>
#include <dynamic_reconfigure/server.h>

namespace stereo_image_proc {

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class DisparityNodelet : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;
  
  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  bool subscribed_;

  // Publications
  ros::Publisher pub_disparity_;

  // Dynamic reconfigure
  typedef stereo_image_proc::DisparityConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  
  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;
  cv::StereoBM block_matcher_; // contains scratch buffers for block matching

  // Error reporting
  boost::shared_ptr<image_proc::AdvertisementChecker> check_inputs_;

  virtual void onInit();

  void connectCb();

  void imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg,
               const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg);

  void configCallback(Config &config, uint32_t level);
};

void DisparityNodelet::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();

  it_.reset(new image_transport::ImageTransport(nh));

  // Monitor whether anyone is subscribed to the output
  subscribed_ = false;
  ros::SubscriberStatusCallback connect_cb = boost::bind(&DisparityNodelet::connectCb, this);
  pub_disparity_  = nh.advertise<DisparityImage>("disparity",  1,
                                                 connect_cb, connect_cb);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection
  // callback. Optionally do approximate synchronization.
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  bool approx;
  private_nh.param("approximate_sync", approx, false);
  if (approx)
  {
    approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                 sub_l_image_, sub_l_info_,
                                                 sub_r_image_, sub_r_info_) );
    approximate_sync_->registerCallback(boost::bind(&DisparityNodelet::imageCb,
                                                    this, _1, _2, _3, _4));
  }
  else
  {
    exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                                     sub_l_image_, sub_l_info_,
                                     sub_r_image_, sub_r_info_) );
    exact_sync_->registerCallback(boost::bind(&DisparityNodelet::imageCb,
                                              this, _1, _2, _3, _4));
  }

  // Set up dynamic reconfiguration
  ReconfigureServer::CallbackType f = boost::bind(&DisparityNodelet::configCallback,
                                                  this, _1, _2);
  reconfigure_server_.reset(new ReconfigureServer(private_nh));
  reconfigure_server_->setCallback(f);

  // Print a warning every minute until the input topics are advertised
  ros::V_string topics;
  topics.push_back("left/image_rect");
  topics.push_back("left/camera_info");
  topics.push_back("right/image_rect");
  topics.push_back("right/camera_info");
  check_inputs_.reset( new image_proc::AdvertisementChecker(nh, getName()) );
  check_inputs_->start(topics, 60.0);
}

// Handles (un)subscribing when clients (un)subscribe
void DisparityNodelet::connectCb()
{
  if (pub_disparity_.getNumSubscribers() == 0)
  {
    sub_l_image_.unsubscribe();
    sub_l_info_ .unsubscribe();
    sub_r_image_.unsubscribe();
    sub_r_info_ .unsubscribe();
    subscribed_ = false;
  }
  else if (!subscribed_)
  {
    ros::NodeHandle &nh = getNodeHandle();
    // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
    sub_l_image_.subscribe(*it_, "left/image_rect", 1);
    sub_l_info_ .subscribe(nh,   "left/camera_info", 1);
    sub_r_image_.subscribe(*it_, "right/image_rect", 1);
    sub_r_info_ .subscribe(nh,   "right/camera_info", 1);
    subscribed_ = true;
  }
}

void DisparityNodelet::imageCb(const ImageConstPtr& l_image_msg,
                               const CameraInfoConstPtr& l_info_msg,
                               const ImageConstPtr& r_image_msg,
                               const CameraInfoConstPtr& r_info_msg)
{
  /// @todo Convert (share) with new cv_bridge
  assert(l_image_msg->encoding == sensor_msgs::image_encodings::MONO8);
  assert(r_image_msg->encoding == sensor_msgs::image_encodings::MONO8);

  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);
  
  // Allocate new disparity image message
  DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
  disp_msg->header         = l_info_msg->header;
  disp_msg->image.header   = l_info_msg->header;
  disp_msg->image.height   = l_image_msg->height;
  disp_msg->image.width    = l_image_msg->width;
  disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  disp_msg->image.step     = disp_msg->image.width * sizeof(float);
  disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);

  // Stereo parameters
  disp_msg->f = model_.right().fx();
  disp_msg->T = model_.baseline();

  /// @todo Window of (potentially) valid disparities

  // Disparity search range
  cv::Ptr<CvStereoBMState> params = block_matcher_.state;
  disp_msg->min_disparity = params->minDisparity;
  disp_msg->max_disparity = params->minDisparity + params->numberOfDisparities - 1;
  disp_msg->delta_d = 1.0 / 16; // OpenCV uses 16 disparities per pixel

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
  block_matcher_(l_image, r_image, disp_image, CV_32F);

  // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
  double cx_l = model_.left().cx();
  double cx_r = model_.right().cx();
  if (cx_l != cx_r)
    cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);

  pub_disparity_.publish(disp_msg);
}

void DisparityNodelet::configCallback(Config &config, uint32_t level)
{
  // Tweak all settings to be valid
  config.prefilter_size |= 0x1; // must be odd
  config.correlation_window_size |= 0x1; // must be odd
  config.disparity_range = (config.disparity_range / 16) * 16; // must be multiple of 16
  
  block_matcher_.state->preFilterSize       = config.prefilter_size;
  block_matcher_.state->preFilterCap        = config.prefilter_cap;
  block_matcher_.state->SADWindowSize       = config.correlation_window_size;
  block_matcher_.state->minDisparity        = config.min_disparity;
  block_matcher_.state->numberOfDisparities = config.disparity_range;
  block_matcher_.state->uniquenessRatio     = config.uniqueness_ratio;
  block_matcher_.state->textureThreshold    = config.texture_threshold;
  block_matcher_.state->speckleWindowSize   = config.speckle_size;
  block_matcher_.state->speckleRange        = config.speckle_range;
}

} // namespace stereo_image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(stereo_image_proc, disparity,
                        stereo_image_proc::DisparityNodelet, nodelet::Nodelet)
