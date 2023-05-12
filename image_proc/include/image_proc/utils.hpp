// Copyright 2023 Willow Garage, Inc., Michal Wojcik
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef IMAGE_PROC__UTILS_HPP_
#define IMAGE_PROC__UTILS_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace image_proc
{

rmw_qos_profile_t getTopicQosProfile(rclcpp::Node * node, const std::string & topic)
{
  /**
   * Given a topic name, get the QoS profile with which it is being published.
￼  * Replaces history and depth settings with default sensor values since they cannot be retrieved.
   * @param node pointer to the ROS node
   * @param topic name of the topic
   * @returns QoS profile of the publisher to the topic. If there are several publishers, it returns
   *     returns the profile of the first one on the list. If no publishers exist, it returns
   *     the sensor data profile.
   */
  std::string topic_resolved = node->get_node_base_interface()->resolve_topic_or_service_name(
    topic, false);
  auto topics_info = node->get_publishers_info_by_topic(topic_resolved);
  if (topics_info.size()) {
    auto profile = topics_info[0].qos_profile().get_rmw_qos_profile();
    profile.history = rmw_qos_profile_sensor_data.history;
    profile.depth = rmw_qos_profile_sensor_data.depth;
    return profile;
  } else {
    return rmw_qos_profile_sensor_data;
  }
}

}  // namespace image_proc

#endif  // IMAGE_PROC__UTILS_HPP_
