#include "image_proc/advertisement_checker.h"
#include <boost/foreach.hpp>

namespace image_proc {

AdvertisementChecker::AdvertisementChecker(const ros::NodeHandle& nh,
                                           const std::string& name)
  : nh_(nh),
    name_(name)
{
}

void AdvertisementChecker::timerCb()
{
  ros::master::V_TopicInfo topic_info;
  if (!ros::master::getTopics(topic_info)) return;

  ros::V_string::iterator topic_it = topics_.begin();
  while (topic_it != topics_.end())
  {
    // Should use std::find_if
    bool found = false;
    ros::master::V_TopicInfo::iterator info_it = topic_info.begin();
    while (!found && info_it != topic_info.end())
    {
      found = (*topic_it == info_it->name);
      ++info_it;
    }
    if (found)
      topic_it = topics_.erase(topic_it);
    else
    {
      ROS_WARN_NAMED(name_, "The input topic '%s' is not yet advertised", topic_it->c_str());
      ++topic_it;
    }
  }

  if (topics_.empty())
    stop();
}

void AdvertisementChecker::start(const ros::V_string& topics, double duration)
{
  topics_.clear();
  BOOST_FOREACH(const std::string& topic, topics)
    topics_.push_back(ros::names::resolve(topic));

  ros::NodeHandle nh;
  timer_ = nh.createWallTimer(ros::WallDuration(duration),
                              boost::bind(&AdvertisementChecker::timerCb, this));
  timerCb();
}

void AdvertisementChecker::stop()
{
  timer_.stop();
}

} // namespace image_proc
