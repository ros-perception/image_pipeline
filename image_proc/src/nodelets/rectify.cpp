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
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_proc/RectifyConfig.h>

namespace image_proc {

class RectifyNodelet : public nodelet::Nodelet
{
  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_camera_;
  int queue_size_;
  
  boost::mutex connect_mutex_;
  image_transport::Publisher pub_rect_;

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef image_proc::RectifyConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  // Processing state (note: only safe because we're using single-threaded NodeHandle!)
  image_geometry::PinholeCameraModel model_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  void configCb(Config &config, uint32_t level);
};

void RectifyNodelet::onInit()
{
  ros::NodeHandle &nh         = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Read parameters
  private_nh.param("queue_size", queue_size_, 5);

  // Set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  ReconfigureServer::CallbackType f = boost::bind(&RectifyNodelet::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&RectifyNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_rect_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_rect_  = it_->advertise("image_rect",  1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void RectifyNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_rect_.getNumSubscribers() == 0)
    sub_camera_.shutdown();
  else if (!sub_camera_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_camera_ = it_->subscribeCamera("image_mono", queue_size_, &RectifyNodelet::imageCb, this, hints);
  }
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
  bool zero_distortion = true;
  for (size_t i = 0; i < info_msg->D.size(); ++i)
  {
    if (info_msg->D[i] != 0.0)
    {
      zero_distortion = false;
      break;
    }
  }
  // This will be true if D is empty/zero sized
  if (zero_distortion)
  {
    pub_rect_.publish(image_msg);
    return;
  }

  // Update the camera model
  model_.fromCameraInfo(info_msg);
  
  // Create cv::Mat views onto both buffers
  const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
  cv::Mat rect;

  // Rectify and publish
  int interpolation;
  {
    boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
    interpolation = config_.interpolation;
  }
  model_.rectifyImage(image, rect, interpolation);

  // Allocate new rectified image message
  sensor_msgs::ImagePtr rect_msg = cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect).toImageMsg();
  pub_rect_.publish(rect_msg);
}

void RectifyNodelet::configCb(Config &config, uint32_t level)
{
  config_ = config;
}

} // namespace image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( image_proc::RectifyNodelet, nodelet::Nodelet)
