/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, JSK Lab.
*                2008, Willow Garage, Inc.
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

/********************************************************************
* image_rotate_nodelet.cpp
* this is a forked version of image_rotate.
* this image_rotate_nodelet supports:
*  1) nodelet
*  2) tf and tf2
*********************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/buffer_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <image_rotate/ImageRotateConfig.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <dynamic_reconfigure/server.h>
#include <math.h>

namespace image_rotate {
class ImageRotateNodelet : public nodelet::Nodelet
{
  bool use_tf2_;
  boost::shared_ptr<tf::TransformListener> tf_sub_;
  tf::TransformBroadcaster tf_pub_;
  boost::shared_ptr<tf2::BufferClient> tf2_client_;
  image_rotate::ImageRotateConfig config_;
  dynamic_reconfigure::Server<image_rotate::ImageRotateConfig> srv;

  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  tf::Stamped<tf::Vector3> target_vector_;
  tf::Stamped<tf::Vector3> source_vector_;
    
  boost::shared_ptr<image_transport::ImageTransport> it_;
  ros::NodeHandle nh_;

  int subscriber_count_;
  double angle_;
  ros::Time prev_stamp_;

  void setupTFListener()
  {
    if (use_tf2_) {
      // shutdown tf_sub_
      if (tf_sub_) {
        tf_sub_.reset();
      }
    }
    else {
      if(!tf_sub_) {
        tf_sub_.reset(new tf::TransformListener());
      }
    }
  }
  
  void reconfigureCallback(image_rotate::ImageRotateConfig &new_config, uint32_t level)
  {
    config_ = new_config;
    target_vector_.setValue(config_.target_x, config_.target_y, config_.target_z);
    source_vector_.setValue(config_.source_x, config_.source_y, config_.source_z);
    if (subscriber_count_)
    { // @todo Could do this without an interruption at some point.
      unsubscribe();
      subscribe();
    }
    if (use_tf2_ != config_.use_tf2) {
      use_tf2_ = config_.use_tf2;
      setupTFListener();
    }
  }

  const std::string &frameWithDefault(const std::string &frame, const std::string &image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
  }

  void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    do_work(msg, cam_info->header.frame_id);
  }
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    do_work(msg, msg->header.frame_id);
  }

  void transformVector(const std::string& input_frame_id, const ros::Time& target_time,
                       const std::string& source_frame_id, const ros::Time& time,
                       const std::string& fixed_frame_id,
                       const tf::Stamped<tf::Vector3>& input_vector,
                       tf::Stamped<tf::Vector3>& target_vector,
                       const ros::Duration& duration)
  {
    if (use_tf2_) {
      geometry_msgs::TransformStamped trans
        = tf2_client_->lookupTransform(input_frame_id, source_frame_id,
                                       target_time, duration);
      // geometry_msgs -> eigen -> tf
      Eigen::Affine3d transform_eigen;
      tf::transformMsgToEigen(trans.transform, transform_eigen);
      tf::StampedTransform transform_tf; // convert trans to tfStampedTransform
      tf::transformEigenToTF(transform_eigen, transform_tf);
      tf::Vector3 origin = tf::Vector3(0, 0, 0);
      tf::Vector3 end = input_vector;
      tf::Vector3 output = (transform_tf * end) - (transform_tf * origin);
      target_vector.setData(output);
      target_vector.stamp_ = input_vector.stamp_;
      target_vector.frame_id_ = input_frame_id;
    }
    else {
      tf_sub_->waitForTransform(input_frame_id, target_time,
                                source_frame_id, time,
                                fixed_frame_id, duration);
      tf_sub_->transformVector(input_frame_id, target_time, input_vector,
                               fixed_frame_id, target_vector);
    }
  }
  
  void do_work(const sensor_msgs::ImageConstPtr& msg, const std::string input_frame_from_msg)
  {
    try
    {
      std::string input_frame_id = frameWithDefault(config_.input_frame_id, input_frame_from_msg);

      // Transform the target vector into the image frame.
      target_vector_.stamp_ = msg->header.stamp;
      target_vector_.frame_id_ = frameWithDefault(config_.target_frame_id, input_frame_id);
      tf::Stamped<tf::Vector3> target_vector_transformed;
      transformVector(input_frame_id, msg->header.stamp,
                      target_vector_.frame_id_, target_vector_.stamp_,
                      input_frame_id, target_vector_, target_vector_transformed,
                      ros::Duration(0.2));

      // Transform the source vector into the image frame.
      source_vector_.stamp_ = msg->header.stamp;
      source_vector_.frame_id_ = frameWithDefault(config_.source_frame_id, input_frame_id);
      tf::Stamped<tf::Vector3> source_vector_transformed;
      transformVector(input_frame_id, msg->header.stamp,
                      source_vector_.frame_id_, source_vector_.stamp_,
                      input_frame_id, source_vector_, source_vector_transformed,
                      ros::Duration(0.01));

      // NODELET_INFO("target: %f %f %f", target_vector_.x(), target_vector_.y(), target_vector_.z());
      // NODELET_INFO("target_transformed: %f %f %f", target_vector_transformed.x(), target_vector_transformed.y(), target_vector_transformed.z());
      // NODELET_INFO("source: %f %f %f", source_vector_.x(), source_vector_.y(), source_vector_.z());
      // NODELET_INFO("source_transformed: %f %f %f", source_vector_transformed.x(), source_vector_transformed.y(), source_vector_transformed.z());

      // Calculate the angle of the rotation.
      double angle = angle_;
      if ((target_vector_transformed.x()    != 0 || target_vector_transformed.y()    != 0) &&
          (source_vector_transformed.x() != 0 || source_vector_transformed.y() != 0))
      {
        angle = atan2(target_vector_transformed.y(), target_vector_transformed.x());
        angle -= atan2(source_vector_transformed.y(), source_vector_transformed.x());
      }

      // Rate limit the rotation.
      if (config_.max_angular_rate == 0)
        angle_ = angle;
      else
      {
        double delta = fmod(angle - angle_, 2.0 * M_PI);
        if (delta > M_PI)
          delta -= 2.0 * M_PI;
        else if (delta < - M_PI)
          delta += 2.0 * M_PI;

        double max_delta = config_.max_angular_rate * (msg->header.stamp - prev_stamp_).toSec();
        if (delta > max_delta)
          delta = max_delta;
        else if (delta < -max_delta)
          delta = - max_delta;

        angle_ += delta;
      }
      angle_ = fmod(angle_, 2.0 * M_PI);
    }
    catch (tf::TransformException &e)
    {
      NODELET_ERROR("Transform error: %s", e.what());
    }

    //NODELET_INFO("angle: %f", 180 * angle_ / M_PI);

    // Publish the transform.
    tf::StampedTransform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform.setRotation(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), angle_));
    transform.frame_id_ = msg->header.frame_id;
    transform.child_frame_id_ = frameWithDefault(config_.output_frame_id, msg->header.frame_id + "_rotated");
    transform.stamp_ = msg->header.stamp;
    tf_pub_.sendTransform(transform);

    // Transform the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat in_image = cv_bridge::toCvShare(msg, msg->encoding)->image;

      // Compute the output image size.
      int max_dim = in_image.cols > in_image.rows ? in_image.cols : in_image.rows;
      int min_dim = in_image.cols < in_image.rows ? in_image.cols : in_image.rows;
      int noblack_dim = min_dim / sqrt(2);
      int diag_dim = sqrt(in_image.cols*in_image.cols + in_image.rows*in_image.rows);
      int out_size;
      int candidates[] = { noblack_dim, min_dim, max_dim, diag_dim, diag_dim }; // diag_dim repeated to simplify limit case.
      int step = config_.output_image_size;
      out_size = candidates[step] + (candidates[step + 1] - candidates[step]) * (config_.output_image_size - step);
      //NODELET_INFO("out_size: %d", out_size);

      // Compute the rotation matrix.
      cv::Mat rot_matrix = cv::getRotationMatrix2D(cv::Point2f(in_image.cols / 2.0, in_image.rows / 2.0), 180 * angle_ / M_PI, 1);
      cv::Mat translation = rot_matrix.col(2);
      rot_matrix.at<double>(0, 2) += (out_size - in_image.cols) / 2.0;
      rot_matrix.at<double>(1, 2) += (out_size - in_image.rows) / 2.0;

      // Do the rotation
      cv::Mat out_image;
      cv::warpAffine(in_image, out_image, rot_matrix, cv::Size(out_size, out_size));

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding, out_image).toImageMsg();
      out_img->header.frame_id = transform.child_frame_id_;
      img_pub_.publish(out_img);
    }
    catch (cv::Exception &e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe()
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info && config_.input_frame_id.empty())
      cam_sub_ = it_->subscribeCamera("image", 3, &ImageRotateNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", 3, &ImageRotateNodelet::imageCallback, this);
  }

  void unsubscribe()
  {
      NODELET_DEBUG("Unsubscribing from image topic.");
      img_sub_.shutdown();
      cam_sub_.shutdown();
  }

  void connectCb(const image_transport::SingleSubscriberPublisher& ssp)
  {
    if (subscriber_count_++ == 0) {
      subscribe();
    }
  }

  void disconnectCb(const image_transport::SingleSubscriberPublisher&)
  {
    subscriber_count_--;
    if (subscriber_count_ == 0) {
      unsubscribe();
    }
  }

public:
  virtual void onInit()
  {
    nh_ = getNodeHandle();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh_));
    tf2_client_.reset(new tf2::BufferClient("tf2_buffer_server", 100, ros::Duration(0.2)));
    subscriber_count_ = 0;
    angle_ = 0;
    prev_stamp_ = ros::Time(0, 0);
    image_transport::SubscriberStatusCallback connect_cb    = boost::bind(&ImageRotateNodelet::connectCb, this, _1);
    image_transport::SubscriberStatusCallback disconnect_cb = boost::bind(&ImageRotateNodelet::disconnectCb, this, _1);
    img_pub_ = image_transport::ImageTransport(ros::NodeHandle(nh_, "rotated")).advertise("image", 1, connect_cb, disconnect_cb);

    dynamic_reconfigure::Server<image_rotate::ImageRotateConfig>::CallbackType f =
      boost::bind(&ImageRotateNodelet::reconfigureCallback, this, _1, _2);
    srv.setCallback(f);
  }
};
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_rotate::ImageRotateNodelet, nodelet::Nodelet);
