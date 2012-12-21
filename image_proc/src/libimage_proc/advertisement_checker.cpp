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
    topics_.push_back(nh_.resolveName(topic));

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
