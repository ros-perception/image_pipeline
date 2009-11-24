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
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>

#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>

class StereoView
{
private:
  image_transport::SubscriberFilter left_sub_, right_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync_;
  
  sensor_msgs::ImageConstPtr last_left_, last_right_;
  sensor_msgs::CvBridge left_bridge_, right_bridge_;
  boost::mutex image_mutex_;
  
  boost::format filename_format_;
  int count_;

public:
  StereoView(ros::NodeHandle& nh)
    : sync_(3), filename_format_(""), count_(0)
  {
    ros::NodeHandle local_nh("~");
    bool autosize;
    local_nh.param("autosize", autosize, true);
    
    std::string format_string;
    local_nh.param("filename_format", format_string, std::string("%s%04i.pgm"));
    filename_format_.parse(format_string);
    
    cvNamedWindow("left", autosize ? CV_WINDOW_AUTOSIZE : 0);
    cvNamedWindow("right", autosize ? CV_WINDOW_AUTOSIZE : 0);
    cvSetMouseCallback("left",  &StereoView::mouseCB, this);
    cvSetMouseCallback("right", &StereoView::mouseCB, this);
    cvStartWindowThread();

    std::string left_topic = nh.resolveName("stereo") + "/left/" + nh.resolveName("image");
    std::string right_topic = nh.resolveName("stereo") + "/right/" + nh.resolveName("image");
    left_sub_.subscribe(nh, left_topic, 3);
    right_sub_.subscribe(nh, right_topic, 3);
    sync_.connectInput(left_sub_, right_sub_);
    sync_.registerCallback(boost::bind(&StereoView::imageCB, this, _1, _2));
  }

  ~StereoView()
  {
    cvDestroyWindow("left");
    cvDestroyWindow("right");
  }

  void showImage(const char* window, const sensor_msgs::ImageConstPtr& img,
                 sensor_msgs::CvBridge& bridge)
  {
    // May want to view raw bayer data
    if (img->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<sensor_msgs::Image>(img)->encoding = "mono8";

    if (boost::const_pointer_cast<sensor_msgs::Image>(img)->encoding == "mono8" &&
        bridge.fromImage(*img, "mono8"))
      cvShowImage(window, bridge.toIpl());
    else if (bridge.fromImage(*img, "bgr8"))
      cvShowImage(window, bridge.toIpl());
    else
      ROS_ERROR("Unable to convert %s image to bgr8", img->encoding.c_str());
  }

  void imageCB(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right)
  {
    {
      boost::lock_guard<boost::mutex> guard(image_mutex_);
    
      // Hang on to message pointers for sake of mouse_cb
      last_left_ = left;
      last_right_ = right;
    }

    showImage("left", left, left_bridge_);
    showImage("right", right, right_bridge_);
  }

  void saveImage(const char* prefix, const IplImage* image)
  {
    if (image) {
      std::string filename = (filename_format_ % prefix % count_).str();
      cvSaveImage(filename.c_str(), image);
      ROS_INFO("Saved image %s", filename.c_str());
    } else {
      ROS_WARN("Couldn't save image, no data!");
    }
  }
  
  static void mouseCB(int event, int x, int y, int flags, void* param)
  {
    if (event != CV_EVENT_LBUTTONDOWN)
      return;
    
    StereoView *sv = (StereoView*)param;
    boost::lock_guard<boost::mutex> guard(sv->image_mutex_);

    sv->saveImage("left", sv->left_bridge_.toIpl());
    sv->saveImage("right", sv->right_bridge_.toIpl());
    sv->count_++;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_view", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  if (nh.resolveName("stereo") == "/stereo") {
    ROS_WARN("stereo_view: stereo has not been remapped! Example command-line usage:\n"
             "\t$ rosrun image_view stereo_view stereo:=narrow_stereo image:=image_color");
  }
  
  StereoView view(nh);
  
  ros::spin();
  return 0;
}
