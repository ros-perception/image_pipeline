#ifndef STEREO_IMAGE_PROC_POINT_CLOUD2_H
#define STEREO_IMAGE_PROC_POINT_CLOUD2_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>

namespace stereo_image_proc {

class PointCloud2Nodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  void connectCb();
  
  void imageCb(const sensor_msgs::ImageConstPtr& l_image_msg,
               const sensor_msgs::CameraInfoConstPtr& l_info_msg,
               const sensor_msgs::CameraInfoConstPtr& r_info_msg,
               const stereo_msgs::DisparityImageConstPtr& disp_msg);
  
  struct Impl;
  boost::shared_ptr<Impl> impl_;
};

} // namespace stereo_image_proc

#endif
