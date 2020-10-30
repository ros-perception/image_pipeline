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

void loadMonocularNodelets(nodelet::Loader& manager, const std::string& side,
                           const XmlRpc::XmlRpcValue& rectify_params,
                           const nodelet::V_string& my_argv)
{
  nodelet::M_string remappings;

  // Explicitly resolve global remappings (wg-ros-pkg #5055).
  // Otherwise the internal remapping 'image_raw' -> 'left/image_raw' can hide a
  // global remapping from the launch file or command line.
  std::string image_raw_topic        = ros::names::resolve(side + "/image_raw");
  std::string image_mono_topic       = ros::names::resolve(side + "/image_mono");
  std::string image_color_topic      = ros::names::resolve(side + "/image_color");
  std::string image_rect_topic       = ros::names::resolve(side + "/image_rect");
  std::string image_rect_color_topic = ros::names::resolve(side + "/image_rect_color");
  std::string camera_info_topic      = ros::names::resolve(side + "/camera_info");
  
  // Debayer nodelet: image_raw -> image_mono, image_color
  remappings["image_raw"]   = image_raw_topic;
  remappings["image_mono"]  = image_mono_topic;
  remappings["image_color"] = image_color_topic;
  std::string debayer_name = ros::this_node::getName() + "_debayer_" + side;
  manager.load(debayer_name, "image_proc/debayer", remappings, my_argv);

  // Rectify nodelet: image_mono -> image_rect
  remappings.clear();
  remappings["image_mono"]  = image_mono_topic;
  remappings["camera_info"] = camera_info_topic;
  remappings["image_rect"]  = image_rect_topic;
  std::string rectify_mono_name = ros::this_node::getName() + "_rectify_mono_" + side;
  if (rectify_params.valid())
    ros::param::set(rectify_mono_name, rectify_params);
  manager.load(rectify_mono_name, "image_proc/rectify", remappings, my_argv);

  // Rectify nodelet: image_color -> image_rect_color
  remappings.clear();
  remappings["image_mono"]  = image_color_topic;
  remappings["camera_info"] = camera_info_topic;
  remappings["image_rect"]  = image_rect_color_topic;
  std::string rectify_color_name = ros::this_node::getName() + "_rectify_color_" + side;
  if (rectify_params.valid())
    ros::param::set(rectify_color_name, rectify_params);
  manager.load(rectify_color_name, "image_proc/rectify", remappings, my_argv);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_image_proc");

  // Check for common user errors
  if (ros::names::remap("camera") != "camera")
  {
    ROS_WARN("Remapping 'camera' has no effect! Start stereo_image_proc in the "
             "stereo namespace instead.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=%s rosrun stereo_image_proc stereo_image_proc",
             ros::names::remap("camera").c_str());
  }
  if (ros::this_node::getNamespace() == "/")
  {
    ROS_WARN("Started in the global namespace! This is probably wrong. Start "
             "stereo_image_proc in the stereo namespace.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=my_stereo rosrun stereo_image_proc stereo_image_proc");
  }

  // Shared parameters to be propagated to nodelet private namespaces
  ros::NodeHandle private_nh("~");
  XmlRpc::XmlRpcValue shared_params;
  int queue_size;
  if (private_nh.getParam("queue_size", queue_size))
    shared_params["queue_size"] = queue_size;

  int downsampling_factor;
  if (private_nh.getParam("downsampling_factor", downsampling_factor))
    shared_params["downsampling_factor"] = downsampling_factor;

  nodelet::Loader manager(false); // Don't bring up the manager ROS API
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  // Load equivalents of image_proc for left and right cameras
  loadMonocularNodelets(manager, "left",  shared_params, my_argv);
  loadMonocularNodelets(manager, "right", shared_params, my_argv);

  // Stereo nodelets also need to know the synchronization policy
  bool approx_sync;
  if (private_nh.getParam("approximate_sync", approx_sync))
    shared_params["approximate_sync"] = XmlRpc::XmlRpcValue(approx_sync);

  // Disparity nodelet
  // Inputs: left/image_rect, left/camera_info, right/image_rect, right/camera_info
  // Outputs: disparity
  // NOTE: Using node name for the disparity nodelet because it is the only one using
  // dynamic_reconfigure so far, and this makes us backwards-compatible with cturtle.
  std::string disparity_name = ros::this_node::getName();
  manager.load(disparity_name, "stereo_image_proc/disparity", remappings, my_argv);

  // PointCloud2 nodelet
  // Inputs: left/image_rect_color, left/camera_info, right/camera_info, disparity
  // Outputs: points2
  std::string point_cloud2_name = ros::this_node::getName() + "_point_cloud2";
  if (shared_params.valid())
    ros::param::set(point_cloud2_name, shared_params);
  manager.load(point_cloud2_name, "stereo_image_proc/point_cloud2", remappings, my_argv);

  // Check for only the original camera topics
  ros::V_string topics;
  topics.push_back(ros::names::resolve("left/image_raw"));
  topics.push_back(ros::names::resolve("left/camera_info"));
  topics.push_back(ros::names::resolve("right/image_raw"));
  topics.push_back(ros::names::resolve("right/camera_info"));
  image_proc::AdvertisementChecker check_inputs(ros::NodeHandle(), ros::this_node::getName());
  check_inputs.start(topics, 60.0);

  ros::spin();
  return 0;
}
