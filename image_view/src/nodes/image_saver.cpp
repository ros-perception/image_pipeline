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
#include <boost/thread.hpp>

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>


/** Class to deal with which callback to call whether we have CameraInfo or not
 */
class Callbacks {
public:
  Callbacks(ros::NodeHandle& nh,
            const std::string& topic,
            const std::string& filename_format,
            const std::string& encoding,
            const bool save_all_image,
            const bool wait_for_save)
      : it_(nh), topic_(topic), encoding_(encoding),
        count_(0), start_time_(0),
        subscribing_(false), should_unsubscribe_(false), wait_for_save_(wait_for_save)
  {
    format_.parse(filename_format);
    if (save_all_image)
    {
      subscribe();
    } else {
      srv_save_ = nh.advertiseService("save", &Callbacks::callbackSave, this);
      srv_start_ = nh.advertiseService("start", &Callbacks::callbackStartSave, this);
      srv_end_ = nh.advertiseService("end", &Callbacks::callbackEndSave, this);
    }
  }

  void subscribe()
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (subscribing_) return;
    subscribing_ = true;

    ROS_DEBUG("subscribe");

    is_first_image_ = true;
    has_camera_info_ = false;

    start_time_ = ros::Time::now();
    end_time_ = ros::Time(0);

    // Useful when CameraInfo is being published
    sub_camera_ = it_.subscribeCamera(topic_, 1, &Callbacks::callbackWithCameraInfo, this);
    // Useful when CameraInfo is not being published
    sub_image_ = it_.subscribe(topic_, 1, &Callbacks::callbackWithoutCameraInfo, this);
  }

  void unsubscribe()
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!subscribing_) return;
    subscribing_ = false;

    ROS_DEBUG("unsubcribe");

    start_time_ = ros::Time(0);
    end_time_ = ros::Time::now();
    should_unsubscribe_ = false;

    sub_camera_.shutdown();
    sub_image_.shutdown();
  }

  bool callbackSave(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &res)
  {
    if (start_time_ != ros::Time(0)) {
      ROS_ERROR("The service '~start' was called. Try calling '~end' first to use '~save' service.");
      return false;
    }
    should_unsubscribe_ = true;
    subscribe();

    if (wait_for_save_)
    {
      ros::Rate r(10);
      while (ros::ok() && subscribing_) {
        ros::spinOnce();
        r.sleep();
      }
    }

    return true;
  }

  bool callbackStartSave(std_srvs::Trigger::Request &req,
                         std_srvs::Trigger::Response &res)
  {
    ROS_INFO("Received start saving request");
    subscribe();
    res.success = true;
    return true;
  }

  bool callbackEndSave(std_srvs::Trigger::Request &req,
                       std_srvs::Trigger::Response &res)
  {
    ROS_INFO("Received end saving request");
    unsubscribe();
    res.success = true;
    return true;
  }

  void callbackWithoutCameraInfo(const sensor_msgs::ImageConstPtr& image_msg)
  {
    if (is_first_image_) {
      is_first_image_ = false;

      // Wait a tiny bit to see whether callbackWithCameraInfo is called
      ros::Duration(0.001).sleep();
    }

    if (has_camera_info_) // Images are saved in callbackWithCameraInfo instead
      return;

    // saving flag priority:
    //  1. request by service.
    //  2. request by topic about start and end.
    //  3. flag 'g_save_all_image'.
    const ros::Time image_stamp = image_msg->header.stamp;
    if (start_time_ <= image_stamp &&
        (end_time_ == ros::Time(0) || image_stamp < end_time_))
    {
      // save the image
      std::string filename;
      if (saveImage(image_msg, filename))
      {
        count_++;
      }
    }

    if (should_unsubscribe_)
    {
      unsubscribe();
    }
  }

  void callbackWithCameraInfo(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info)
  {
    has_camera_info_ = true;

    // saving flag priority:
    //  1. request by service.
    //  2. request by topic about start and end.
    //  3. flag 'g_save_all_image'.
    const ros::Time image_stamp = image_msg->header.stamp;
    if (start_time_ <= image_stamp &&
        (end_time_ == ros::Time(0) || image_stamp < end_time_))
    {
      // save the image
      std::string filename;
      if (saveImage(image_msg, filename))
      {
        count_++;
        // save the CameraInfo
        if (info) {
          filename = filename.replace(filename.rfind("."), filename.length(), ".ini");
          camera_calibration_parsers::writeCalibration(filename, "camera", *info);
        }
      }
    }

    if (should_unsubscribe_)
    {
      unsubscribe();
    }
  }
private:
  bool saveImage(const sensor_msgs::ImageConstPtr& image_msg, std::string &filename) {
    cv::Mat image;
    try
    {
      image = cv_bridge::toCvShare(image_msg, encoding_)->image;
    } catch(cv_bridge::Exception)
    {
      ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding_.c_str());
      return false;
    }

    if (image.empty()) {
      ROS_WARN("Couldn't save image, no data!");
      return false;
    }

    try {
      filename = (format_).str();
    } catch (...) { format_.clear(); }
    try {
      filename = (format_ % count_).str();
    } catch (...) { format_.clear(); }
    try {
      filename = (format_ % count_ % "jpg").str();
    } catch (...) { format_.clear(); }

    cv::imwrite(filename, image);
    ROS_INFO("Saved image %s", filename.c_str());

    return true;
  }

private:
  boost::format format_;
  boost::mutex mutex_;
  ros::ServiceServer srv_save_, srv_start_, srv_end_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  image_transport::Subscriber sub_image_;
  std::string topic_;
  std::string encoding_;
  size_t count_;
  ros::Time start_time_;
  ros::Time end_time_;
  bool is_first_image_;
  bool has_camera_info_;
  bool subscribing_;
  bool should_unsubscribe_;
  bool wait_for_save_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_saver", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  std::string topic = nh.resolveName("image");

  ros::NodeHandle local_nh("~");

  std::string format_string;
  local_nh.param("filename_format", format_string, std::string("left%04i.%s"));

  std::string encoding;
  local_nh.param("encoding", encoding, std::string("bgr8"));

  bool save_all_image;
  local_nh.param("save_all_image", save_all_image, true);

  bool wait_for_save;
  local_nh.param("wait_for_save", wait_for_save, false);

  Callbacks callbacks(local_nh, topic, format_string, encoding, save_all_image, wait_for_save);

  ros::spin();

  return 0;
}
