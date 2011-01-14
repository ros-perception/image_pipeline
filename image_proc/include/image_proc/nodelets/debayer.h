#ifndef IMAGE_PROC_NODELETS_DEBAYER_H
#define IMAGE_PROC_NODELETS_DEBAYER_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>

namespace image_proc {

class DebayerNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& raw_msg);
  
  struct Impl;
  boost::shared_ptr<Impl> impl_;
};

} // namespace image_proc

#endif
