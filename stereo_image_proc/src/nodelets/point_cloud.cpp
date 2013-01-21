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
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/stereo_camera_model.h>

#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>

namespace stereo_image_proc {

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class PointCloudNodelet : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;

  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  message_filters::Subscriber<DisparityImage> sub_disparity_;
  typedef ExactTime<Image, CameraInfo, CameraInfo, DisparityImage> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, CameraInfo, DisparityImage> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_points_;

  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;
  cv::Mat_<cv::Vec3f> points_mat_; // scratch buffer

  virtual void onInit();

  void connectCb();

  void imageCb(const ImageConstPtr& l_image_msg,
               const CameraInfoConstPtr& l_info_msg,
               const CameraInfoConstPtr& r_info_msg,
               const DisparityImageConstPtr& disp_msg);
};

void PointCloudNodelet::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Synchronize inputs. Topic subscriptions happen on demand in the connection
  // callback. Optionally do approximate synchronization.
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  bool approx;
  private_nh.param("approximate_sync", approx, false);
  if (approx)
  {
    approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                 sub_l_image_, sub_l_info_,
                                                 sub_r_info_, sub_disparity_) );
    approximate_sync_->registerCallback(boost::bind(&PointCloudNodelet::imageCb,
                                                    this, _1, _2, _3, _4));
  }
  else
  {
    exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                                     sub_l_image_, sub_l_info_,
                                     sub_r_info_, sub_disparity_) );
    exact_sync_->registerCallback(boost::bind(&PointCloudNodelet::imageCb,
                                              this, _1, _2, _3, _4));
  }

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloudNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_points_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_points_ = nh.advertise<PointCloud>("points", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void PointCloudNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_points_.getNumSubscribers() == 0)
  {
    sub_l_image_  .unsubscribe();
    sub_l_info_   .unsubscribe();
    sub_r_info_   .unsubscribe();
    sub_disparity_.unsubscribe();
  }
  else if (!sub_l_image_.getSubscriber())
  {
    ros::NodeHandle &nh = getNodeHandle();
    // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_l_image_  .subscribe(*it_, "left/image_rect_color", 1, hints);
    sub_l_info_   .subscribe(nh,   "left/camera_info", 1);
    sub_r_info_   .subscribe(nh,   "right/camera_info", 1);
    sub_disparity_.subscribe(nh,   "disparity", 1);
  }
}

inline bool isValidPoint(const cv::Vec3f& pt)
{
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
}

void PointCloudNodelet::imageCb(const ImageConstPtr& l_image_msg,
                                const CameraInfoConstPtr& l_info_msg,
                                const CameraInfoConstPtr& r_info_msg,
                                const DisparityImageConstPtr& disp_msg)
{
  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);

  // Calculate point cloud
  const Image& dimage = disp_msg->image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  model_.projectDisparityImageTo3d(dmat, points_mat_, true);
  cv::Mat_<cv::Vec3f> mat = points_mat_;

  // Fill in new PointCloud message (1D dense layout - no invalid points)
  PointCloudPtr points_msg = boost::make_shared<PointCloud>();
  points_msg->header = disp_msg->header;

  points_msg->channels.resize(3);
  points_msg->channels[0].name = "rgb";
  points_msg->channels[0].values.resize(0);
  points_msg->channels[1].name = "u";
  points_msg->channels[1].values.resize(0);
  points_msg->channels[2].name = "v";
  points_msg->channels[2].values.resize(0);

  // NOTE: For historical reasons, (u,v) = (row, column), the opposite of the
  // usual convention. Good thing we're moving to sensor_msgs/PointCloud2.
  for (int32_t u = 0; u < mat.rows; ++u) {
    for (int32_t v = 0; v < mat.cols; ++v) {
      if (isValidPoint(mat(u,v))) {
        // x,y,z
        geometry_msgs::Point32 pt;
        pt.x = mat(u,v)[0];
        pt.y = mat(u,v)[1];
        pt.z = mat(u,v)[2];
        points_msg->points.push_back(pt);
        // u,v
        points_msg->channels[1].values.push_back(u);
        points_msg->channels[2].values.push_back(v);
      }
    }
  }

  // Fill in color
  namespace enc = sensor_msgs::image_encodings;
  const std::string& encoding = l_image_msg->encoding;
  points_msg->channels[0].values.reserve(points_msg->points.size());
  if (encoding == enc::MONO8) {
    const cv::Mat_<uint8_t> color(l_image_msg->height, l_image_msg->width,
                                  (uint8_t*)&l_image_msg->data[0],
                                  l_image_msg->step);
    for (int32_t u = 0; u < mat.rows; ++u) {
      for (int32_t v = 0; v < mat.cols; ++v) {
        if (isValidPoint(mat(u,v))) {
          uint8_t g = color(u,v);
          int32_t rgb = (g << 16) | (g << 8) | g;
          points_msg->channels[0].values.push_back(*(float*)(&rgb));
        }
      }
    }
  }
  else if (encoding == enc::RGB8) {
    const cv::Mat_<cv::Vec3b> color(l_image_msg->height, l_image_msg->width,
                                    (cv::Vec3b*)&l_image_msg->data[0],
                                    l_image_msg->step);
    for (int32_t u = 0; u < mat.rows; ++u) {
      for (int32_t v = 0; v < mat.cols; ++v) {
        if (isValidPoint(mat(u,v))) {
          const cv::Vec3b& rgb = color(u,v);
          int32_t rgb_packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
          points_msg->channels[0].values.push_back(*(float*)(&rgb_packed));
        }
      }
    }
  }
  else if (encoding == enc::BGR8) {
    const cv::Mat_<cv::Vec3b> color(l_image_msg->height, l_image_msg->width,
                                    (cv::Vec3b*)&l_image_msg->data[0],
                                    l_image_msg->step);
    for (int32_t u = 0; u < mat.rows; ++u) {
      for (int32_t v = 0; v < mat.cols; ++v) {
        if (isValidPoint(mat(u,v))) {
          const cv::Vec3b& bgr = color(u,v);
          int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
          points_msg->channels[0].values.push_back(*(float*)(&rgb_packed));
        }
      }
    }
  }
  else {
    NODELET_WARN_THROTTLE(30, "Could not fill color channel of the point cloud, "
                          "unsupported encoding '%s'", encoding.c_str());
  }

  pub_points_.publish(points_msg);
}

} // namespace stereo_image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stereo_image_proc::PointCloudNodelet,nodelet::Nodelet)
