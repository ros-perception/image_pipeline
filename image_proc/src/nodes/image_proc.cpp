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

#include <opencv/cv.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv_latest/CvBridge.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>

//
// This is the node creation file
// Subscribes to a single image topic, and performs undistortion and 
//   color processing on the image
//

class ImageProc
{
private:
  ros::NodeHandle node_handle_;
  ros::V_Subscriber subs_;
  
  sensor_msgs::ImageConstPtr last_msg_;
  sensor_msgs::CvBridge img_bridge_;
  
  int count_;

public:
  ImageProc(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle), count_(0)
  {
    // subscribe to the input image
    subs_.push_back( node_handle_.subscribe("image", 1, &ImageProc::image_cb, this) );
  }

  ~ImageProc()
  {
  }

  void image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    sensor_msgs::Image msg2;
    count_++;
    ROS_INFO("[image_proc] Got message %d with encoding %s", 
	     count_, msg->encoding.c_str());

    // processing here, then send out image
#if 0
    // May want to view raw bayer data
    if (msg->encoding.find("bayer") != std::string::npos)
      msg->encoding = "mono";
#endif
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_proc", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  if (n.resolveName("image") == "/image") {
    ROS_WARN("image_view: image has not been remapped! Example command-line usage:\n"
             "\t$ rosrun image_view image_view image:=/forearm/image_color");
  }
  
  ImageProc proc(n);

  ros::spin();
  
  return 0;
}
