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
#include <cv_bridge/cv_bridge.h>
#include <cstdint>
#include <opencv2/imgproc/imgproc.hpp>

namespace depth_image_proc {

namespace enc = sensor_msgs::image_encodings;

class CropForemostNodelet : public nodelet::Nodelet
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

  double distance_;
};

void CropForemostNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  private_nh.getParam("distance", distance_);
  it_.reset(new image_transport::ImageTransport(nh));

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&CropForemostNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_depth_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_depth_ = it_->advertise("image", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void CropForemostNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_depth_.getNumSubscribers() == 0)
  {
    sub_raw_.shutdown();
  }
  else if (!sub_raw_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_raw_ = it_->subscribe("image_raw", 1, &CropForemostNodelet::depthCb, this, hints);
  }
}

void CropForemostNodelet::depthCb(const sensor_msgs::ImageConstPtr& raw_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(raw_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  if (raw_msg->encoding == enc::TYPE_8UC1){

    // search the min value without invalid value "0"
    cv::MatIterator_<uint8_t>ind = std::min_element(cv_ptr->image.begin<uint8_t>(), cv_ptr->image.end<uint8_t>(), [](uint8_t a, uint8_t b) {
      return (a == 0) ? false : (b == 0) || a < b;
    });
    cv::threshold(cv_ptr->image, cv_ptr->image, *ind + (uint8_t)distance_, 0, CV_THRESH_TOZERO_INV);

  }else if (raw_msg->encoding == enc::TYPE_8SC1){

    // search the min value without invalid value "0"
    cv::MatIterator_<int8_t>ind = std::min_element(cv_ptr->image.begin<int8_t>(), cv_ptr->image.end<int8_t>(), [](int8_t a, int8_t b) {
      return (a == 0) ? false : (b == 0) || a < b;
    });
    cv::threshold(cv_ptr->image, cv_ptr->image, *ind + (int8_t)distance_, 0, CV_THRESH_TOZERO_INV);

  }else if (raw_msg->encoding == enc::TYPE_16UC1){

    // search the min value without invalid value "0"
    cv::MatIterator_<uint16_t>ind = std::min_element(cv_ptr->image.begin<uint16_t>(), cv_ptr->image.end<uint16_t>(), [](uint16_t a, uint16_t b) {
      return (a == 0) ? false : (b == 0) || a < b;
    });

    // 8 bit or 32 bit floating array is required to use cv::threshold
    cv::Mat1f mf(raw_msg->width, raw_msg->height);
    cv_ptr->image.convertTo(mf, CV_32F);
    cv::threshold(mf, mf, (double)*ind + distance_, 0, CV_THRESH_TOZERO_INV);
    mf.convertTo(cv_ptr->image, CV_16U);

  }else if (raw_msg->encoding == enc::TYPE_16SC1){

    // search the min value without invalid value "0"
    cv::MatIterator_<int16_t>ind = std::min_element(cv_ptr->image.begin<int16_t>(), cv_ptr->image.end<int16_t>(), [](int16_t a, int16_t b) {
      return (a == 0) ? false : (b == 0) || a < b;
    });

    // 8 bit or 32 bit floating array is required to use cv::threshold
    cv::Mat1f mf(raw_msg->width, raw_msg->height);
    cv_ptr->image.convertTo(mf, CV_32F);
    cv::threshold(mf, mf, (double)*ind + distance_, 0, CV_THRESH_TOZERO_INV);
    mf.convertTo(cv_ptr->image, CV_16S);

  //}else if (raw_msg->encoding == enc::TYPE_32SC1){
  }else if (raw_msg->encoding == enc::TYPE_32FC1){
    cv::MatIterator_<float>ind = std::min_element(cv_ptr->image.begin<float>(), cv_ptr->image.end<float>(), [](float a, float b) {
      return (a == 0.) ? false : (b == 0.) || a < b;
    });
    cv::threshold(cv_ptr->image, cv_ptr->image, *ind + distance_, 0, CV_THRESH_TOZERO_INV);
  //}else if (raw_msg->encoding == enc::TYPE_64FC1){
  }else{
    NODELET_ERROR_THROTTLE(2, "Only 8UC1, 8SC1, 16UC1, 16SC1, and 32FC1 image is acceptable, got [%s]", raw_msg->encoding.c_str());
    return;
  }

  pub_depth_.publish(cv_ptr->toImageMsg());
}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_image_proc::CropForemostNodelet,nodelet::Nodelet);
