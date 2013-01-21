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
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread.hpp>

namespace depth_image_proc {

namespace enc = sensor_msgs::image_encodings;

class ConvertMetricNodelet : public nodelet::Nodelet
{
  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber sub_raw_;

  // Publications
  boost::mutex connect_mutex_;
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
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&ConvertMetricNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_depth_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_depth_ = it_->advertise("image", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void ConvertMetricNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_depth_.getNumSubscribers() == 0)
  {
    sub_raw_.shutdown();
  }
  else if (!sub_raw_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_raw_ = it_->subscribe("image_raw", 1, &ConvertMetricNodelet::depthCb, this, hints);
  }
}

void ConvertMetricNodelet::depthCb(const sensor_msgs::ImageConstPtr& raw_msg)
{
  if (raw_msg->encoding != enc::TYPE_16UC1)
  {
    NODELET_ERROR_THROTTLE(2, "Expected data of type [%s], got [%s]", enc::TYPE_16UC1.c_str(),
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
PLUGINLIB_EXPORT_CLASS(depth_image_proc::ConvertMetricNodelet,nodelet::Nodelet);
