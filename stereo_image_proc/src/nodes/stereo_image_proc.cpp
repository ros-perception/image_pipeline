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

void loadMonocularNodelets(nodelet::Loader& manager, const std::string& side)
{
  /// @todo Is there no easier way to push a nodelet into a namespace?
  nodelet::M_string remappings;
  nodelet::V_string my_argv;
  
  // Debayer nodelet: image_raw -> image_mono, image_color
  remappings["image_raw"]   = side + "/image_raw";
  remappings["image_mono"]  = side + "/image_mono";
  remappings["image_color"] = side + "/image_color";
  manager.load("debayer_" + side, "image_proc/debayer", remappings, my_argv);

  // Rectify nodelet: image_mono -> image_rect
  remappings.clear();
  remappings["image_mono"]  = side + "/image_mono";
  remappings["camera_info"] = side + "/camera_info";
  remappings["image_rect"]  = side + "/image_rect";
  manager.load("rectify_mono_" + side, "image_proc/rectify", remappings, my_argv);

  // Rectify nodelet: image_color -> image_rect_color
  remappings.clear();
  remappings["image_mono"]  = side + "/image_color";
  remappings["camera_info"] = side + "/camera_info";
  remappings["image_rect"]  = side + "/image_rect_color";
  manager.load("rectify_color_" + side, "image_proc/rectify", remappings, my_argv);
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

  nodelet::Loader manager(false);
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  // Load equivalents of image_proc for left and right cameras
  loadMonocularNodelets(manager, "left");
  loadMonocularNodelets(manager, "right");

  // Disparity nodelet
  // Inputs: left/image_rect, left/camera_info, right/image_rect, right/camera_info
  // Outputs: disparity
  /// @todo Maybe use ros::this_node::getName() so dynamic_reconfigure looks same
  manager.load("disparity", "stereo_image_proc/disparity", remappings, my_argv);

  // PointCloud2 nodelet
  // Inputs: left/image_rect_color, left/camera_info, right/camera_info, disparity
  // Outputs: points2
  manager.load("point_cloud2", "stereo_image_proc/point_cloud2", remappings, my_argv);

  // PointCloud (deprecated) nodelet
  // Inputs: left/image_rect_color, left/camera_info, right/camera_info, disparity
  // Outputs: points
  manager.load("point_cloud", "stereo_image_proc/point_cloud", remappings, my_argv);

  /// @todo Would be nice to disable nodelet input checking and consolidate it here

  ros::spin();
  return 0;
}
