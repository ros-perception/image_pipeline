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
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <algorithm> // for std::max_element

namespace image_proc {

namespace enc = sensor_msgs::image_encodings;

class CropNonZeroNodelet : public nodelet::Nodelet
{
  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber sub_raw_;

  // Publications
  boost::mutex connect_mutex_;
  image_transport::Publisher pub_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& raw_msg);
};

void CropNonZeroNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&CropNonZeroNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_depth_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_ = it_->advertise("image", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void CropNonZeroNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0)
  {
    sub_raw_.shutdown();
  }
  else if (!sub_raw_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_raw_ = it_->subscribe("image_raw", 1, &CropNonZeroNodelet::imageCb, this, hints);
  }
}

void CropNonZeroNodelet::imageCb(const sensor_msgs::ImageConstPtr& raw_msg)
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

  // Check the number of channels
  if(sensor_msgs::image_encodings::numChannels(raw_msg->encoding) != 1){
    NODELET_ERROR_THROTTLE(2, "Only grayscale image is acceptable, got [%s]", raw_msg->encoding.c_str());
    return;
  }

  std::vector<std::vector<cv::Point> >cnt;
  cv::Mat1b m(raw_msg->width, raw_msg->height);

  if (raw_msg->encoding == enc::TYPE_8UC1){
    m = cv_ptr->image;
  }else{
    double minVal, maxVal;
    cv::minMaxIdx(cv_ptr->image, &minVal, &maxVal, 0, 0, cv_ptr->image != 0.);
    double ra = maxVal - minVal;

    cv_ptr->image.convertTo(m, CV_8U, 255./ra, -minVal*255./ra);
  }

  cv::findContours(m, cnt, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  // search the largest area
  /* // -std=c++11
  std::vector<std::vector<cv::Point> >::iterator it = std::max_element(cnt.begin(), cnt.end(), [](std::vector<cv::Point> a, std::vector<cv::Point> b) {
    return a.size() < b.size();
  });
  */
  std::vector<std::vector<cv::Point> >::iterator it = cnt.begin();
  for(std::vector<std::vector<cv::Point> >::iterator i=cnt.begin();i!=cnt.end();++i){
    it = (*it).size() < (*i).size() ? i : it;
  }
  cv::Rect r = cv::boundingRect(cnt[std::distance(cnt.begin(), it)]);

  cv_bridge::CvImage out_msg;
  out_msg.header   = raw_msg->header;
  out_msg.encoding = raw_msg->encoding;
  out_msg.image    = cv_ptr->image(r);

  pub_.publish(out_msg.toImageMsg());
}

} // namespace image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(image_proc::CropNonZeroNodelet,nodelet::Nodelet);
