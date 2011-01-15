#ifndef IMAGE_PROC_ADVERTISEMENT_CHECKER_H
#define IMAGE_PROC_ADVERTISEMENT_CHECKER_H

#include <ros/ros.h>

namespace image_proc {

class AdvertisementChecker
{
  ros::NodeHandle nh_;
  std::string name_;
  ros::WallTimer timer_;
  ros::V_string topics_;

  void timerCb();

public:
  AdvertisementChecker(const ros::NodeHandle& nh = ros::NodeHandle(),
                       const std::string& name = std::string());
  
  void start(const ros::V_string& topics, double duration);

  void stop();
};

} // namespace image_proc

#endif
