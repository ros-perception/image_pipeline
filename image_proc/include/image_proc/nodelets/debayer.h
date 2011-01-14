#ifndef IMAGE_PROC_NODELETS_DEBAYER_H
#define IMAGE_PROC_NODELETS_DEBAYER_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>

namespace image_proc {

class DebayerNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  struct Impl;
  boost::shared_ptr<Impl> impl_;
};

} // namespace image_proc

#endif
