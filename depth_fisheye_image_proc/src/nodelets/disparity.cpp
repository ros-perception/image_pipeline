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
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <depth_image_proc/depth_traits.h>

namespace depth_image_proc {

namespace enc = sensor_msgs::image_encodings;

class DisparityNodelet : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> left_it_;
  ros::NodeHandlePtr right_nh_;
  image_transport::SubscriberFilter sub_depth_image_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
  typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> Sync;
  boost::shared_ptr<Sync> sync_;
  
  boost::mutex connect_mutex_;
  ros::Publisher pub_disparity_;
  double min_range_;
  double max_range_;
  double delta_d_;

  virtual void onInit();

  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  template<typename T>
  void convert(const sensor_msgs::ImageConstPtr& depth_msg,
               stereo_msgs::DisparityImagePtr& disp_msg);
};

void DisparityNodelet::onInit()
{
  ros::NodeHandle &nh         = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  ros::NodeHandle left_nh(nh, "left");
  left_it_.reset(new image_transport::ImageTransport(left_nh));
  right_nh_.reset( new ros::NodeHandle(nh, "right") );

  // Read parameters
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  private_nh.param("min_range", min_range_, 0.0);
  private_nh.param("max_range", max_range_, std::numeric_limits<double>::infinity());
  private_nh.param("delta_d", delta_d_, 0.125);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  sync_.reset( new Sync(sub_depth_image_, sub_info_, queue_size) );
  sync_->registerCallback(boost::bind(&DisparityNodelet::depthCb, this, _1, _2));

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&DisparityNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_disparity_ = left_nh.advertise<stereo_msgs::DisparityImage>("disparity", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void DisparityNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_disparity_.getNumSubscribers() == 0)
  {
    sub_depth_image_.unsubscribe();
    sub_info_ .unsubscribe();
  }
  else if (!sub_depth_image_.getSubscriber())
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_depth_image_.subscribe(*left_it_, "image_rect", 1, hints);
    sub_info_.subscribe(*right_nh_, "camera_info", 1);
  }
}

void DisparityNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                               const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // Allocate new DisparityImage message
  stereo_msgs::DisparityImagePtr disp_msg( new stereo_msgs::DisparityImage );
  disp_msg->header         = depth_msg->header;
  disp_msg->image.header   = disp_msg->header;
  disp_msg->image.encoding = enc::TYPE_32FC1;
  disp_msg->image.height   = depth_msg->height;
  disp_msg->image.width    = depth_msg->width;
  disp_msg->image.step     = disp_msg->image.width * sizeof (float);
  disp_msg->image.data.resize( disp_msg->image.height * disp_msg->image.step, 0.0f );
  double fx = info_msg->P[0];
  disp_msg->T = -info_msg->P[3] / fx;
  disp_msg->f = fx;
  // Remaining fields depend on device characteristics, so rely on user input
  disp_msg->min_disparity = disp_msg->f * disp_msg->T / max_range_;
  disp_msg->max_disparity = disp_msg->f * disp_msg->T / min_range_;
  disp_msg->delta_d = delta_d_;

  if (depth_msg->encoding == enc::TYPE_16UC1)
  {
    convert<uint16_t>(depth_msg, disp_msg);
  }
  else if (depth_msg->encoding == enc::TYPE_32FC1)
  {
    convert<float>(depth_msg, disp_msg);
  }
  else
  {
    NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  pub_disparity_.publish(disp_msg);
}

template<typename T>
void DisparityNodelet::convert(const sensor_msgs::ImageConstPtr& depth_msg,
                               stereo_msgs::DisparityImagePtr& disp_msg)
{
  // For each depth Z, disparity d = fT / Z
  float unit_scaling = DepthTraits<T>::toMeters( T(1) );
  float constant = disp_msg->f * disp_msg->T / unit_scaling;

  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  float* disp_data = reinterpret_cast<float*>(&disp_msg->image.data[0]);
  for (int v = 0; v < (int)depth_msg->height; ++v)
  {
    for (int u = 0; u < (int)depth_msg->width; ++u)
    {
      T depth = depth_row[u];
      if (DepthTraits<T>::valid(depth))
        *disp_data = constant / depth;
      ++disp_data;
    }

    depth_row += row_step;
  }
}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_image_proc::DisparityNodelet,nodelet::Nodelet);
