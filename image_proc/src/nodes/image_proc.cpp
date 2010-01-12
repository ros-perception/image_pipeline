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
#include <ros/names.h>
#include <ros/master.h>
#include <ros/this_node.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/foreach.hpp>

#include "image_proc/processor.h"

class ImageProcNode
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber cam_sub_;
  image_transport::Publisher pub_mono_;
  image_transport::Publisher pub_rect_;
  image_transport::Publisher pub_color_;
  image_transport::Publisher pub_rect_color_;
  ros::Timer check_inputs_timer_;

  // OK for these to be members in single-threaded case.
  image_proc::Processor processor_;
  image_geometry::PinholeCameraModel model_;
  image_proc::ImageSet processed_images_;
  /// @todo Separate color_img_ to avoid some allocations?
  sensor_msgs::Image img_;
  int subscriber_count_;

public:

  ImageProcNode()
    : it_(nh_),
      subscriber_count_(0)
  {
    // Advertise outputs
    image_transport::SubscriberStatusCallback connect_cb    = boost::bind(&ImageProcNode::connectCb, this, _1);
    image_transport::SubscriberStatusCallback disconnect_cb = boost::bind(&ImageProcNode::disconnectCb, this, _1);
    pub_mono_       = it_.advertise("image_mono", 1, connect_cb, disconnect_cb);
    pub_rect_       = it_.advertise("image_rect", 1, connect_cb, disconnect_cb);
    pub_color_      = it_.advertise("image_color", 1, connect_cb, disconnect_cb);
    pub_rect_color_ = it_.advertise("image_rect_color", 1, connect_cb, disconnect_cb);

    // Print a warning every minute until the camera topics are advertised
    check_inputs_timer_ = nh_.createTimer(ros::Duration(60.0), boost::bind(&ImageProcNode::checkInputsAdvertised, this));
    checkInputsAdvertised();
  }

  void connectCb(const image_transport::SingleSubscriberPublisher& ssp)
  {
    if (subscriber_count_++ == 0) {
      ROS_DEBUG("Subscribing to camera topics");
      cam_sub_ = it_.subscribeCamera("image_raw", 3, &ImageProcNode::imageCb, this);
    }
  }

  void disconnectCb(const image_transport::SingleSubscriberPublisher&)
  {
    subscriber_count_--;
    if (subscriber_count_ == 0) {
      ROS_DEBUG("Unsubscribing from camera topics");
      cam_sub_.shutdown();
    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& raw_image, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    // Update the camera model
    model_.fromCameraInfo(cam_info);

    // Compute which outputs are in demand
    int flags = 0;
    typedef image_proc::Processor Proc;
    if (pub_mono_      .getNumSubscribers() > 0) flags |= Proc::MONO;
    if (pub_rect_      .getNumSubscribers() > 0) flags |= Proc::RECT;
    if (pub_color_     .getNumSubscribers() > 0) flags |= Proc::COLOR;
    if (pub_rect_color_.getNumSubscribers() > 0) flags |= Proc::RECT_COLOR;

    // Process raw image into colorized and/or rectified outputs
    if (!processor_.process(raw_image, model_, processed_images_, flags))
      return;

    // Publish images
    img_.header.stamp = raw_image->header.stamp;
    img_.header.frame_id = raw_image->header.frame_id;
    if (flags & Proc::MONO)
      publishImage(pub_mono_, processed_images_.mono, sensor_msgs::image_encodings::MONO8);
    if (flags & Proc::RECT)
      publishImage(pub_rect_, processed_images_.rect, sensor_msgs::image_encodings::MONO8);
    if (flags & Proc::COLOR)
      publishImage(pub_color_, processed_images_.color, processed_images_.color_encoding);
    if (flags & Proc::RECT_COLOR)
      publishImage(pub_rect_color_, processed_images_.rect_color, processed_images_.color_encoding);
  }

  void publishImage(const image_transport::Publisher& pub, const cv::Mat& image, const std::string& encoding)
  {
    fillImage(img_, encoding, image.rows, image.cols, image.step, const_cast<uint8_t*>(image.data));
    pub.publish(img_);
  }

  void checkInputsAdvertised()
  {
    ros::master::V_TopicInfo topics;
    if (!ros::master::getTopics(topics)) return;

    std::string image_topic = nh_.resolveName("image_raw");
    std::string info_topic  = nh_.resolveName("camera_info");
    bool have_image = false, have_info = false;
    BOOST_FOREACH(const ros::master::TopicInfo& info, topics) {
      have_image = have_image || (info.name == image_topic);
      have_info = have_info || (info.name == info_topic);
      if (have_image && have_info) {
        check_inputs_timer_.stop();
        return;
      }
    }
    if (!have_image)
      ROS_WARN("The camera image topic [%s] does not appear to be published yet.", image_topic.c_str());
    if (!have_info)
      ROS_WARN("The camera info topic [%s] does not appear to be published yet.", info_topic.c_str());
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_proc", ros::init_options::AnonymousName);

  // Check for common user errors
  if (ros::names::remap("camera") != "camera") {
    ROS_WARN("[image_proc] Remapping 'camera' no longer has any effect! Start image_proc in the "
             "camera namespace instead.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=%s rosrun image_proc image_proc", ros::names::remap("camera").c_str());
  }
  if (ros::this_node::getNamespace() == "/") {
    ROS_WARN("[image_proc] Started in the global namespace! This is probably wrong. Start image_proc "
             "in the camera namespace.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=my_camera rosrun image_proc image_proc");
  }

  ImageProcNode proc;

  ros::spin();
  return 0;
}
