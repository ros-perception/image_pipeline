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

#include <ros/ros.h>
#include <nodelet/loader.h>
#include <image_proc/advertisement_checker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_proc");

  // Check for common user errors
  if (ros::names::remap("camera") != "camera")
  {
    ROS_WARN("Remapping 'camera' has no effect! Start image_proc in the "
             "camera namespace instead.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=%s rosrun image_proc image_proc",
             ros::names::remap("camera").c_str());
  }
  if (ros::this_node::getNamespace() == "/")
  {
    ROS_WARN("Started in the global namespace! This is probably wrong. Start image_proc "
             "in the camera namespace.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=my_camera rosrun image_proc image_proc");
  }

  // Shared parameters to be propagated to nodelet private namespaces
  ros::NodeHandle private_nh("~");
  XmlRpc::XmlRpcValue shared_params;
  int queue_size;
  if (private_nh.getParam("queue_size", queue_size))
    shared_params["queue_size"] = queue_size;

  nodelet::Loader manager(false); // Don't bring up the manager ROS API
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  // Debayer nodelet, image_raw -> image_mono, image_color
  std::string debayer_name = ros::this_node::getName() + "_debayer";
  manager.load(debayer_name, "image_proc/debayer", remappings, my_argv);

  // Rectify nodelet, image_mono -> image_rect
  std::string rectify_mono_name = ros::this_node::getName() + "_rectify_mono";
  if (shared_params.valid())
    ros::param::set(rectify_mono_name, shared_params);
  manager.load(rectify_mono_name, "image_proc/rectify", remappings, my_argv);

  // Rectify nodelet, image_color -> image_rect_color
  // NOTE: Explicitly resolve any global remappings here, so they don't get hidden.
  remappings["image_mono"] = ros::names::resolve("image_color");
  remappings["image_rect"] = ros::names::resolve("image_rect_color");
  std::string rectify_color_name = ros::this_node::getName() + "_rectify_color";
  if (shared_params.valid())
    ros::param::set(rectify_color_name, shared_params);
  manager.load(rectify_color_name, "image_proc/rectify", remappings, my_argv);

  // Check for only the original camera topics
  ros::V_string topics;
  topics.push_back(ros::names::resolve("image_raw"));
  topics.push_back(ros::names::resolve("camera_info"));
  image_proc::AdvertisementChecker check_inputs(ros::NodeHandle(), ros::this_node::getName());
  check_inputs.start(topics, 60.0);
  
  ros::spin();
  return 0;
}
