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
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <boost/format.hpp>

#include <std_srvs/Empty.h>

boost::format g_format;
bool save_all_image, save_image_service;
std::string encoding;

bool service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  save_image_service = true;
  return true;
}

/** Class to deal with which callback to call whether we have CameraInfo or not
 */
class Callbacks {
public:
  Callbacks() : is_first_image_(true), has_camera_info_(false), count_(0) {
  }

  void callbackWithoutCameraInfo(const sensor_msgs::ImageConstPtr& image_msg)
  {
    if (is_first_image_) {
      is_first_image_ = false;

      // Wait a tiny bit to see whether callbackWithCameraInfo is called
      ros::Duration(0.001).sleep();
    }

    if (has_camera_info_)
      return;

    // save the image
    std::string filename;
    if (!saveImage(image_msg, filename))
      return;

    count_++;
  }

  void callbackWithCameraInfo(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info)
  {
    has_camera_info_ = true;

    // save the image
    std::string filename;
    if (!saveImage(image_msg, filename))
      return;

    // save the CameraInfo
    if (info) {
      filename = filename.replace(filename.rfind("."), filename.length(), ".ini");
      camera_calibration_parsers::writeCalibration(filename, "camera", *info);
    }

    count_++;
  }
private:
  bool saveImage(const sensor_msgs::ImageConstPtr& image_msg, std::string &filename) {
    cv::Mat image;
    try
    {
      image = cv_bridge::toCvShare(image_msg, encoding)->image;
    } catch(cv_bridge::Exception)
    {
      ROS_ERROR("Unable to convert %s image to bgr8", image_msg->encoding.c_str());
      return false;
    }

    if (!image.empty()) {
      try {
        filename = (g_format).str();
      } catch (...) { g_format.clear(); }
      try {
        filename = (g_format % count_).str();
      } catch (...) { g_format.clear(); }
      try { 
        filename = (g_format % count_ % "jpg").str();
      } catch (...) { g_format.clear(); }

      if ( save_all_image || save_image_service ) {
        cv::imwrite(filename, image);
        ROS_INFO("Saved image %s", filename.c_str());

        save_image_service = false;
      }
    } else {
      ROS_WARN("Couldn't save image, no data!");
      return false;
    }
    return true;
  }

private:
  bool is_first_image_;
  bool has_camera_info_;
  size_t count_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_saver", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  std::string topic = nh.resolveName("image");

  Callbacks callbacks;
  // Useful when CameraInfo is being published
  image_transport::CameraSubscriber sub_image_and_camera = it.subscribeCamera(topic, 1,
                                                                              &Callbacks::callbackWithCameraInfo,
                                                                              &callbacks);
  // Useful when CameraInfo is not being published
  image_transport::Subscriber sub_image = it.subscribe(
      topic, 1, boost::bind(&Callbacks::callbackWithoutCameraInfo, &callbacks, _1));

  ros::NodeHandle local_nh("~");
  std::string format_string;
  local_nh.param("filename_format", format_string, std::string("left%04i.%s"));
  local_nh.param("encoding", encoding, std::string("bgr8"));
  local_nh.param("save_all_image", save_all_image, true);
  g_format.parse(format_string);
  ros::ServiceServer save = local_nh.advertiseService ("save", service);

  ros::spin();
}
