/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Kentaro Wada.
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
*   * Neither the name of the Kentaro Wada nor the names of its
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
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>
#include <nodelet_topic_tools/nodelet_lazy.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "image_proc/ResizeConfig.h"

namespace image_proc
{

class ResizeNodelet : public nodelet_topic_tools::NodeletLazy
{
protected:
  // ROS communication
  ros::Publisher pub_image_;
  ros::Publisher pub_info_;
  ros::Subscriber sub_info_;
  ros::Subscriber sub_image_;

  // Dynamic reconfigure
  boost::mutex mutex_;
  typedef image_proc::ResizeConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  bool use_scale_;
  int interpolation_;
  int height_;
  int width_;
  double scale_height_;
  double scale_width_;

  virtual void onInit();
  virtual void subscribe();
  virtual void unsubscribe();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);
  void infoCb(const sensor_msgs::CameraInfoConstPtr& info_msg);

  void configCb(Config &config, uint32_t level);
};

void ResizeNodelet::onInit()
{
  nodelet_topic_tools::NodeletLazy::onInit();

  // Set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(*pnh_));
  ReconfigureServer::CallbackType f = boost::bind(&ResizeNodelet::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);

  pub_info_ = advertise<sensor_msgs::CameraInfo>(*pnh_, "camera_info", 1);
  pub_image_ = advertise<sensor_msgs::Image>(*pnh_, "image", 1);

  onInitPostProcess();
}

void ResizeNodelet::configCb(Config &config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  interpolation_ = config.interpolation;
  use_scale_ = config.use_scale;
  height_ = config.height;
  width_ = config.width;
  scale_height_ = config.scale_height;
  scale_width_ = config.scale_width;
}

void ResizeNodelet::subscribe()
{
  sub_info_ = nh_->subscribe("camera_info", 1, &ResizeNodelet::infoCb, this);
  sub_image_ = nh_->subscribe("image", 1, &ResizeNodelet::imageCb, this);
}

void ResizeNodelet::unsubscribe()
{
  sub_info_.shutdown();
  sub_image_.shutdown();
}

void ResizeNodelet::infoCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  boost::mutex::scoped_lock lock(mutex_);

  sensor_msgs::CameraInfo dst_info_msg = *info_msg;

  double scale_y;
  double scale_x;
  if (use_scale_)
  {
    scale_y = scale_height_;
    scale_x = scale_width_;
    dst_info_msg.height = static_cast<int>(info_msg->height * scale_height_);
    dst_info_msg.width = static_cast<int>(info_msg->width * scale_width_);
  }
  else
  {
    scale_y = static_cast<double>(height_) / info_msg->height;
    scale_x = static_cast<double>(width_) / info_msg->width;
    dst_info_msg.height = height_;
    dst_info_msg.width = width_;
  }

  dst_info_msg.K[0] = dst_info_msg.K[0] * scale_x;  // fx
  dst_info_msg.K[2] = dst_info_msg.K[2] * scale_x;  // cx
  dst_info_msg.K[4] = dst_info_msg.K[4] * scale_y;  // fy
  dst_info_msg.K[5] = dst_info_msg.K[5] * scale_y;  // cy

  dst_info_msg.P[0] = dst_info_msg.P[0] * scale_x;  // fx
  dst_info_msg.P[2] = dst_info_msg.P[2] * scale_x;  // cx
  dst_info_msg.P[3] = dst_info_msg.P[3] * scale_x;  // T
  dst_info_msg.P[5] = dst_info_msg.P[5] * scale_y;  // fy
  dst_info_msg.P[6] = dst_info_msg.P[6] * scale_y;  // cy

  pub_info_.publish(dst_info_msg);
}

void ResizeNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg)
{
  boost::mutex::scoped_lock lock(mutex_);

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg);

  if (use_scale_)
  {
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(0, 0), scale_width_, scale_height_, interpolation_);
  }
  else
  {
    int height = height_ == -1 ? image_msg->height : height_;
    int width = width_ == -1 ? image_msg->width : width_;
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(width, height), 0, 0, interpolation_);
  }

  pub_image_.publish(cv_ptr->toImageMsg());
}

}  // namespace image_proc

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_proc::ResizeNodelet, nodelet::Nodelet)
