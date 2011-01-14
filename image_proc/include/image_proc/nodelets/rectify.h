#ifndef IMAGE_PROC_NODELETS_RECTIFY_H
#define IMAGE_PROC_NODELETS_RECTIFY_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace image_proc {

class RectifyNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  void connectCb();
  
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
  
  struct Impl;
  boost::shared_ptr<Impl> impl_;
};

} // namespace image_proc

#endif
