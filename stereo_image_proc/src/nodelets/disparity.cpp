/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <stereo_image_proc/DisparityConfig.h>
#include <dynamic_reconfigure/server.h>

#include <stereo_image_proc/processor.h>

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
  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_disparity_;

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef stereo_image_proc::DisparityConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  
  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;
  stereo_image_proc::StereoProcessor block_matcher_; // contains scratch buffers for block matching

  virtual void onInit();

  void connectCb();

  void imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg,
               const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg);

  void configCb(Config &config, uint32_t level);
};

void DisparityNodelet::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();

  it_.reset(new image_transport::ImageTransport(nh));

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
  ReconfigureServer::CallbackType f = boost::bind(&DisparityNodelet::configCb,
                                                  this, _1, _2);
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&DisparityNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_disparity_ = nh.advertise<DisparityImage>("disparity", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void DisparityNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_disparity_.getNumSubscribers() == 0)
  {
    sub_l_image_.unsubscribe();
    sub_l_info_ .unsubscribe();
    sub_r_image_.unsubscribe();
    sub_r_info_ .unsubscribe();
  }
  else if (!sub_l_image_.getSubscriber())
  {
    ros::NodeHandle &nh = getNodeHandle();
    // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
    /// @todo Allow remapping left, right?
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_l_image_.subscribe(*it_, "left/image_rect", 1, hints);
    sub_l_info_ .subscribe(nh,   "left/camera_info", 1);
    sub_r_image_.subscribe(*it_, "right/image_rect", 1, hints);
    sub_r_info_ .subscribe(nh,   "right/camera_info", 1);
  }
}

void DisparityNodelet::imageCb(const ImageConstPtr& l_image_msg,
                               const CameraInfoConstPtr& l_info_msg,
                               const ImageConstPtr& r_image_msg,
                               const CameraInfoConstPtr& r_info_msg)
{
  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);

  // Allocate new disparity image message
  DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
  disp_msg->header         = l_info_msg->header;
  disp_msg->image.header   = l_info_msg->header;

  // Compute window of (potentially) valid disparities
  int border   = block_matcher_.getCorrelationWindowSize() / 2;
  int left   = block_matcher_.getDisparityRange() + block_matcher_.getMinDisparity() + border - 1;
  int wtf = (block_matcher_.getMinDisparity() >= 0) ? border + block_matcher_.getMinDisparity() : std::max(border, -block_matcher_.getMinDisparity());
  int right  = disp_msg->image.width - 1 - wtf;
  int top    = border;
  int bottom = disp_msg->image.height - 1 - border;
  disp_msg->valid_window.x_offset = left;
  disp_msg->valid_window.y_offset = top;
  disp_msg->valid_window.width    = right - left;
  disp_msg->valid_window.height   = bottom - top;

  // Create cv::Mat views onto all buffers
  const cv::Mat_<uint8_t> l_image = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)->image;
  const cv::Mat_<uint8_t> r_image = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)->image;

  // Perform block matching to find the disparities
  block_matcher_.processDisparity(l_image, r_image, model_, *disp_msg);

  // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
  double cx_l = model_.left().cx();
  double cx_r = model_.right().cx();
  if (cx_l != cx_r) {
    cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
                              reinterpret_cast<float*>(&disp_msg->image.data[0]),
                              disp_msg->image.step);
    cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);
  }

  pub_disparity_.publish(disp_msg);
}

void DisparityNodelet::configCb(Config &config, uint32_t level)
{
  // Tweak all settings to be valid
  config.prefilter_size |= 0x1; // must be odd
  config.correlation_window_size |= 0x1; // must be odd
  config.disparity_range = (config.disparity_range / 16) * 16; // must be multiple of 16
  
  // check stereo method
  // Note: With single-threaded NodeHandle, configCb and imageCb can't be called
  // concurrently, so this is thread-safe.
  block_matcher_.setPreFilterCap(config.prefilter_cap);
  block_matcher_.setCorrelationWindowSize(config.correlation_window_size);
  block_matcher_.setMinDisparity(config.min_disparity);
  block_matcher_.setDisparityRange(config.disparity_range);
  block_matcher_.setUniquenessRatio(config.uniqueness_ratio);
  block_matcher_.setSpeckleSize(config.speckle_size);
  block_matcher_.setSpeckleRange(config.speckle_range);
  if (config.stereo_algorithm == stereo_image_proc::Disparity_StereoBM) { // StereoBM
    block_matcher_.setStereoType(StereoProcessor::BM);
    block_matcher_.setPreFilterSize(config.prefilter_size);
    block_matcher_.setTextureThreshold(config.texture_threshold);
  }
  else if (config.stereo_algorithm == stereo_image_proc::Disparity_StereoSGBM) { // StereoSGBM
    block_matcher_.setStereoType(StereoProcessor::SGBM);
    block_matcher_.setSgbmMode(config.fullDP);
    block_matcher_.setP1(config.P1);
    block_matcher_.setP2(config.P2);
    block_matcher_.setDisp12MaxDiff(config.disp12MaxDiff);
  }
}

} // namespace stereo_image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stereo_image_proc::DisparityNodelet,nodelet::Nodelet)
