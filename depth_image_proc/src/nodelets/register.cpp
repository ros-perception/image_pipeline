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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <depth_image_proc/depth_traits.h>

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
  boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  boost::shared_ptr<tf2_ros::TransformListener> tf_;
  typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  boost::shared_ptr<Synchronizer> sync_;

  // Publications
  boost::mutex connect_mutex_;
  image_transport::CameraPublisher pub_registered_;

  image_geometry::PinholeCameraModel depth_model_, rgb_model_;

  // Parameters
  bool fill_upsampling_holes_;	// fills holes which occur due to upsampling by scaling each pixel to the target image scale (only takes effect on upsampling)

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& depth_image_msg,
               const sensor_msgs::CameraInfoConstPtr& depth_info_msg,
               const sensor_msgs::CameraInfoConstPtr& rgb_info_msg);

  template<typename T>
  void convert(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::ImagePtr& registered_msg,
               const Eigen::Affine3d& depth_to_rgb);
};

void RegisterNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  nh_depth_.reset( new ros::NodeHandle(nh, "depth") );
  nh_rgb_.reset( new ros::NodeHandle(nh, "rgb") );
  it_depth_.reset( new image_transport::ImageTransport(*nh_depth_) );
  tf_buffer_.reset( new tf2_ros::Buffer );
  tf_.reset( new tf2_ros::TransformListener(*tf_buffer_) );

  // Read parameters
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  private_nh.param("fill_upsampling_holes", fill_upsampling_holes_, false);

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

  // Query tf2 for transform from (X,Y,Z) in depth camera frame to RGB camera frame
  Eigen::Affine3d depth_to_rgb;
  try
  {
    geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform (
                          rgb_info_msg->header.frame_id, depth_info_msg->header.frame_id,
                          depth_info_msg->header.stamp);

    tf::transformMsgToEigen(transform.transform, depth_to_rgb);
  }
  catch (tf2::TransformException& ex)
  {
    NODELET_WARN_THROTTLE(2, "TF2 exception:\n%s", ex.what());
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
                              const Eigen::Affine3d& depth_to_rgb)
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

      if (fill_upsampling_holes_ == false)
      {
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
      else
      {
        // Reproject (u,v,Z) to (X,Y,Z,1) in depth camera frame
        Eigen::Vector4d xyz_depth_1, xyz_depth_2;
        xyz_depth_1 << ((u-0.5f - depth_cx)*depth - depth_Tx) * inv_depth_fx,
                       ((v-0.5f - depth_cy)*depth - depth_Ty) * inv_depth_fy,
                       depth,
                       1;
        xyz_depth_2 << ((u+0.5f - depth_cx)*depth - depth_Tx) * inv_depth_fx,
                       ((v+0.5f - depth_cy)*depth - depth_Ty) * inv_depth_fy,
                       depth,
                       1;

        // Transform to RGB camera frame
        Eigen::Vector4d xyz_rgb_1 = depth_to_rgb * xyz_depth_1;
        Eigen::Vector4d xyz_rgb_2 = depth_to_rgb * xyz_depth_2;

        // Project to (u,v) in RGB image
        double inv_Z = 1.0 / xyz_rgb_1.z();
        int u_rgb_1 = (rgb_fx*xyz_rgb_1.x() + rgb_Tx)*inv_Z + rgb_cx + 0.5;
        int v_rgb_1 = (rgb_fy*xyz_rgb_1.y() + rgb_Ty)*inv_Z + rgb_cy + 0.5;
        inv_Z = 1.0 / xyz_rgb_2.z();
        int u_rgb_2 = (rgb_fx*xyz_rgb_2.x() + rgb_Tx)*inv_Z + rgb_cx + 0.5;
        int v_rgb_2 = (rgb_fy*xyz_rgb_2.y() + rgb_Ty)*inv_Z + rgb_cy + 0.5;

        if (u_rgb_1 < 0 || u_rgb_2 >= (int)registered_msg->width ||
            v_rgb_1 < 0 || v_rgb_2 >= (int)registered_msg->height)
          continue;

        for (int nv=v_rgb_1; nv<=v_rgb_2; ++nv)
        {
          for (int nu=u_rgb_1; nu<=u_rgb_2; ++nu)
          {
            T& reg_depth = registered_data[nv*registered_msg->width + nu];
            T  new_depth = DepthTraits<T>::fromMeters(0.5*(xyz_rgb_1.z()+xyz_rgb_2.z()));
            // Validity and Z-buffer checks
            if (!DepthTraits<T>::valid(reg_depth) || reg_depth > new_depth)
              reg_depth = new_depth;
          }
        }
      }
    }
  }
}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_image_proc::RegisterNodelet,nodelet::Nodelet);
