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
#include <mutex>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include "image_proc/ResizeConfig.h"

namespace image_proc
{
class ResizeNodelet : public nodelet::Nodelet
{
protected:
  // ROS communication
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::NodeHandle> pnh_;
  image_transport::Publisher pub_image_;
  ros::Publisher pub_info_;
  image_transport::Subscriber sub_image_;
  ros::Subscriber sub_info_;

  std::shared_ptr<image_transport::ImageTransport> it_, private_it_;
  std::mutex connect_mutex_;

  // Dynamic reconfigure
  std::mutex config_mutex_;
  typedef image_proc::ResizeConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  std::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;
  cv_bridge::CvImage scaled_cv_;

  virtual void onInit();
  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);
  void infoCb(const sensor_msgs::CameraInfoConstPtr& info_msg);

  void configCb(Config& config, uint32_t level);
};

void ResizeNodelet::onInit()
{
  nh_.reset(new ros::NodeHandle(getNodeHandle()));
  pnh_.reset(new ros::NodeHandle(getPrivateNodeHandle()));
  it_.reset(new image_transport::ImageTransport(*nh_));
  private_it_.reset(new image_transport::ImageTransport(*pnh_));

  // Set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(*pnh_));
  ReconfigureServer::CallbackType f =
      std::bind(&ResizeNodelet::configCb, this, std::placeholders::_1, std::placeholders::_2);
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  auto connect_cb = std::bind(&ResizeNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to
  // pub_XXX
  std::lock_guard<std::mutex> lock(connect_mutex_);
  pub_image_ = private_it_->advertise("image", 1, connect_cb, connect_cb);
  pub_info_ = pnh_->advertise<sensor_msgs::CameraInfo>("camera_info", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void ResizeNodelet::connectCb()
{
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (pub_image_.getNumSubscribers() == 0)
  {
    sub_image_.shutdown();
  }
  else if (!sub_image_)
  {
    sub_image_ = it_->subscribe("image", 1, &ResizeNodelet::imageCb, this);
  }
  if (pub_info_.getNumSubscribers() == 0)
  {
    sub_info_.shutdown();
  }
  else if (!sub_info_)
  {
    sub_info_ = nh_->subscribe<sensor_msgs::CameraInfo>("camera_info", 1, &ResizeNodelet::infoCb, this);
  }
}

void ResizeNodelet::configCb(Config& config, uint32_t level)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  config_ = config;
}

void ResizeNodelet::infoCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  Config config;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config = config_;
  }

  sensor_msgs::CameraInfoPtr dst_info_msg(new sensor_msgs::CameraInfo(*info_msg));

  double scale_y;
  double scale_x;
  if (config.use_scale)
  {
    scale_y = config.scale_height;
    scale_x = config.scale_width;
    dst_info_msg->height = static_cast<int>(info_msg->height * config.scale_height);
    dst_info_msg->width = static_cast<int>(info_msg->width * config.scale_width);
  }
  else
  {
    scale_y = static_cast<double>(config.height) / info_msg->height;
    scale_x = static_cast<double>(config.width) / info_msg->width;
    dst_info_msg->height = config.height;
    dst_info_msg->width = config.width;
  }

  dst_info_msg->K[0] = dst_info_msg->K[0] * scale_x;  // fx
  dst_info_msg->K[2] = dst_info_msg->K[2] * scale_x;  // cx
  dst_info_msg->K[4] = dst_info_msg->K[4] * scale_y;  // fy
  dst_info_msg->K[5] = dst_info_msg->K[5] * scale_y;  // cy

  dst_info_msg->P[0] = dst_info_msg->P[0] * scale_x;  // fx
  dst_info_msg->P[2] = dst_info_msg->P[2] * scale_x;  // cx
  dst_info_msg->P[3] = dst_info_msg->P[3] * scale_x;  // T
  dst_info_msg->P[5] = dst_info_msg->P[5] * scale_y;  // fy
  dst_info_msg->P[6] = dst_info_msg->P[6] * scale_y;  // cy

  dst_info_msg->roi.x_offset = static_cast<int>(dst_info_msg->roi.x_offset * scale_x);
  dst_info_msg->roi.y_offset = static_cast<int>(dst_info_msg->roi.y_offset * scale_y);
  dst_info_msg->roi.width = static_cast<int>(dst_info_msg->roi.width * scale_x);
  dst_info_msg->roi.height = static_cast<int>(dst_info_msg->roi.height * scale_y);

  pub_info_.publish(dst_info_msg);
}

void ResizeNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg)
{
  Config config;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config = config_;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(image_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (config.use_scale)
  {
    cv::resize(cv_ptr->image, scaled_cv_.image, cv::Size(0, 0), config.scale_width, config.scale_height,
               config.interpolation);
  }
  else
  {
    int height = config.height == -1 ? image_msg->height : config.height;
    int width = config.width == -1 ? image_msg->width : config.width;
    cv::resize(cv_ptr->image, scaled_cv_.image, cv::Size(width, height), 0, 0, config.interpolation);
  }

  scaled_cv_.header = image_msg->header;
  scaled_cv_.encoding = image_msg->encoding;
  pub_image_.publish(scaled_cv_.toImageMsg());
}

}  // namespace image_proc

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(image_proc::ResizeNodelet, nodelet::Nodelet)
