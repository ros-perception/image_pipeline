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
  boost::recursive_mutex config_mutex_;
  typedef image_proc::ResizeConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

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
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, *pnh_));
  ReconfigureServer::CallbackType f = boost::bind(&ResizeNodelet::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);

  pub_info_ = advertise<sensor_msgs::CameraInfo>(*pnh_, "camera_info", 1);
  pub_image_ = advertise<sensor_msgs::Image>(*pnh_, "image", 1);

  onInitPostProcess();
}

void ResizeNodelet::configCb(Config &config, uint32_t level)
{
  config_ = config;
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
  Config config;
  {
    boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
    config = config_;
  }

  sensor_msgs::CameraInfo dst_info_msg = *info_msg;

  double scale_y;
  double scale_x;
  if (config.use_scale)
  {
    scale_y = config.scale_height;
    scale_x = config.scale_width;
    dst_info_msg.height = static_cast<int>(info_msg->height * config.scale_height);
    dst_info_msg.width = static_cast<int>(info_msg->width * config.scale_width);
  }
  else
  {
    scale_y = static_cast<double>(config.height) / info_msg->height;
    scale_x = static_cast<double>(config.width) / info_msg->width;
    dst_info_msg.height = config.height;
    dst_info_msg.width = config.width;
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
  Config config;
  {
    boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
    config = config_;
  }

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg);

  if (config.use_scale)
  {
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(0, 0),
               config.scale_width, config.scale_height, config.interpolation);
  }
  else
  {
    int height = config.height == -1 ? image_msg->height : config.height;
    int width = config.width == -1 ? image_msg->width : config.width;
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(width, height), 0, 0, config.interpolation);
  }

  pub_image_.publish(cv_ptr->toImageMsg());
}

}  // namespace image_proc

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_proc::ResizeNodelet, nodelet::Nodelet)
