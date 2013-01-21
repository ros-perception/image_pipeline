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
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <Eigen/Core>
#include "depth_traits.h"

namespace depth_image_proc {

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

class RegisterNodelet : public nodelet::Nodelet
{
  ros::NodeHandlePtr nh_depth_, nh_rgb_;
  boost::shared_ptr<image_transport::ImageTransport> it_depth_;
  
  // Subscriptions
  image_transport::SubscriberFilter sub_depth_image_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_depth_info_, sub_rgb_info_;
  boost::shared_ptr<tf::TransformListener> tf_;
  typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  boost::shared_ptr<Synchronizer> sync_;

  // Publications
  boost::mutex connect_mutex_;
  image_transport::CameraPublisher pub_registered_;

  image_geometry::PinholeCameraModel depth_model_, rgb_model_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& depth_image_msg,
               const sensor_msgs::CameraInfoConstPtr& depth_info_msg,
               const sensor_msgs::CameraInfoConstPtr& rgb_info_msg);

  template<typename T>
  void convert(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::ImagePtr& registered_msg,
               const Eigen::Matrix4d& depth_to_rgb);
};

void RegisterNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  nh_depth_.reset( new ros::NodeHandle(nh, "depth") );
  nh_rgb_.reset( new ros::NodeHandle(nh, "rgb") );
  it_depth_.reset( new image_transport::ImageTransport(*nh_depth_) );
  tf_.reset( new tf::TransformListener );

  // Read parameters
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_image_, sub_depth_info_, sub_rgb_info_) );
  sync_->registerCallback(boost::bind(&RegisterNodelet::imageCb, this, _1, _2, _3));

  // Monitor whether anyone is subscribed to the output
  image_transport::ImageTransport it_depth_reg(ros::NodeHandle(nh, "depth_registered"));
  image_transport::SubscriberStatusCallback image_connect_cb = boost::bind(&RegisterNodelet::connectCb, this);
  ros::SubscriberStatusCallback info_connect_cb = boost::bind(&RegisterNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_registered_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_registered_ = it_depth_reg.advertiseCamera("image_rect", 1,
                                                 image_connect_cb, image_connect_cb,
                                                 info_connect_cb, info_connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void RegisterNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_registered_.getNumSubscribers() == 0)
  {
    sub_depth_image_.unsubscribe();
    sub_depth_info_ .unsubscribe();
    sub_rgb_info_   .unsubscribe();
  }
  else if (!sub_depth_image_.getSubscriber())
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_depth_image_.subscribe(*it_depth_, "image_rect",  1, hints);
    sub_depth_info_ .subscribe(*nh_depth_, "camera_info", 1);
    sub_rgb_info_   .subscribe(*nh_rgb_,   "camera_info", 1);
  }
}

void RegisterNodelet::imageCb(const sensor_msgs::ImageConstPtr& depth_image_msg,
                              const sensor_msgs::CameraInfoConstPtr& depth_info_msg,
                              const sensor_msgs::CameraInfoConstPtr& rgb_info_msg)
{
  // Update camera models - these take binning & ROI into account
  depth_model_.fromCameraInfo(depth_info_msg);
  rgb_model_  .fromCameraInfo(rgb_info_msg);

  // Query tf for transform from (X,Y,Z) in depth camera frame to RGB camera frame
  Eigen::Matrix4d depth_to_rgb;
  try
  {
    ros::Duration timeout(1.0 / 30); /// @todo Parameter for expected frame rate
    tf_->waitForTransform(rgb_info_msg->header.frame_id, depth_info_msg->header.frame_id,
                          depth_info_msg->header.stamp, timeout);
    tf::StampedTransform transform;
    tf_->lookupTransform (rgb_info_msg->header.frame_id, depth_info_msg->header.frame_id,
                          depth_info_msg->header.stamp, transform);

    tf::Matrix3x3& R = transform.getBasis();
    tf::Vector3&   T = transform.getOrigin();
    depth_to_rgb << R[0][0], R[0][1], R[0][2], T[0],
                    R[1][0], R[1][1], R[1][2], T[1],
                    R[2][0], R[2][1], R[2][2], T[2],
                          0,       0,       0,    1;
  }
  catch (tf::TransformException& ex)
  {
    NODELET_WARN_THROTTLE(2, "TF exception:\n%s", ex.what());
    return;
    /// @todo Can take on order of a minute to register a disconnect callback when we
    /// don't call publish() in this cb. What's going on roscpp?
  }

  // Allocate registered depth image
  sensor_msgs::ImagePtr registered_msg( new sensor_msgs::Image );
  registered_msg->header.stamp    = depth_image_msg->header.stamp;
  registered_msg->header.frame_id = rgb_info_msg->header.frame_id;
  registered_msg->encoding        = depth_image_msg->encoding;
  
  cv::Size resolution = rgb_model_.reducedResolution();
  registered_msg->height = resolution.height;
  registered_msg->width  = resolution.width;
  // step and data set in convert(), depend on depth data type

  if (depth_image_msg->encoding == enc::TYPE_16UC1)
  {
    convert<uint16_t>(depth_image_msg, registered_msg, depth_to_rgb);
  }
  else if (depth_image_msg->encoding == enc::TYPE_32FC1)
  {
    convert<float>(depth_image_msg, registered_msg, depth_to_rgb);
  }
  else
  {
    NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_image_msg->encoding.c_str());
    return;
  }

  // Registered camera info is the same as the RGB info, but uses the depth timestamp
  sensor_msgs::CameraInfoPtr registered_info_msg( new sensor_msgs::CameraInfo(*rgb_info_msg) );
  registered_info_msg->header.stamp = registered_msg->header.stamp;

  pub_registered_.publish(registered_msg, registered_info_msg);
}

template<typename T>
void RegisterNodelet::convert(const sensor_msgs::ImageConstPtr& depth_msg,
                              const sensor_msgs::ImagePtr& registered_msg,
                              const Eigen::Matrix4d& depth_to_rgb)
{
  // Allocate memory for registered depth image
  registered_msg->step = registered_msg->width * sizeof(T);
  registered_msg->data.resize( registered_msg->height * registered_msg->step );
  // data is already zero-filled in the uint16 case, but for floats we want to initialize everything to NaN.
  DepthTraits<T>::initializeBuffer(registered_msg->data);

  // Extract all the parameters we need
  double inv_depth_fx = 1.0 / depth_model_.fx();
  double inv_depth_fy = 1.0 / depth_model_.fy();
  double depth_cx = depth_model_.cx(), depth_cy = depth_model_.cy();
  double depth_Tx = depth_model_.Tx(), depth_Ty = depth_model_.Ty();
  double rgb_fx = rgb_model_.fx(), rgb_fy = rgb_model_.fy();
  double rgb_cx = rgb_model_.cx(), rgb_cy = rgb_model_.cy();
  double rgb_Tx = rgb_model_.Tx(), rgb_Ty = rgb_model_.Ty();
  
  // Transform the depth values into the RGB frame
  /// @todo When RGB is higher res, interpolate by rasterizing depth triangles onto the registered image  
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  T* registered_data = reinterpret_cast<T*>(&registered_msg->data[0]);
  int raw_index = 0;
  for (unsigned v = 0; v < depth_msg->height; ++v, depth_row += row_step)
  {
    for (unsigned u = 0; u < depth_msg->width; ++u, ++raw_index)
    {
      T raw_depth = depth_row[u];
      if (!DepthTraits<T>::valid(raw_depth))
        continue;
      
      double depth = DepthTraits<T>::toMeters(raw_depth);

      /// @todo Combine all operations into one matrix multiply on (u,v,d)
      // Reproject (u,v,Z) to (X,Y,Z,1) in depth camera frame
      Eigen::Vector4d xyz_depth;
      xyz_depth << ((u - depth_cx)*depth - depth_Tx) * inv_depth_fx,
                   ((v - depth_cy)*depth - depth_Ty) * inv_depth_fy,
                   depth,
                   1;

      // Transform to RGB camera frame
      Eigen::Vector4d xyz_rgb = depth_to_rgb * xyz_depth;

      // Project to (u,v) in RGB image
      double inv_Z = 1.0 / xyz_rgb.z();
      int u_rgb = (rgb_fx*xyz_rgb.x() + rgb_Tx)*inv_Z + rgb_cx + 0.5;
      int v_rgb = (rgb_fy*xyz_rgb.y() + rgb_Ty)*inv_Z + rgb_cy + 0.5;
      
      if (u_rgb < 0 || u_rgb >= (int)registered_msg->width ||
          v_rgb < 0 || v_rgb >= (int)registered_msg->height)
        continue;
      
      T& reg_depth = registered_data[v_rgb*registered_msg->width + u_rgb];
      T  new_depth = DepthTraits<T>::fromMeters(xyz_rgb.z());
      // Validity and Z-buffer checks
      if (!DepthTraits<T>::valid(reg_depth) || reg_depth > new_depth)
        reg_depth = new_depth;
    }
  }
}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_image_proc::RegisterNodelet,nodelet::Nodelet);
