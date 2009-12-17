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
#include <ros/names.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>

#include <image_transport/image_transport.h>

#include "image_proc/image.h"
#include "image_proc/cam_bridge.h"

//
// This is the node creation file
// Subscribes to a single image topic, and performs rectification and 
//   color processing on the image
//

// NB: We currently rely on single-threaded spinning.
class ImageProc
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber cam_sub_;
  image_transport::Publisher pub_mono_;
  image_transport::Publisher pub_rect_;
  image_transport::Publisher pub_color_;
  image_transport::Publisher pub_rect_color_;

  // OK for these to be members in single-threaded case.
  cam::ImageData img_data_;
  sensor_msgs::Image img_;
  int subscriber_count_;

public:

  ImageProc(const ros::NodeHandle& nh)
    : nh_(nh), it_(nh_),
      subscriber_count_(0)
  {
    // Advertise outputs
    image_transport::SubscriberStatusCallback connect_cb    = boost::bind(&ImageProc::connectCb, this, _1);
    image_transport::SubscriberStatusCallback disconnect_cb = boost::bind(&ImageProc::disconnectCb, this, _1);
    pub_mono_       = it_.advertise("image_mono", 1, connect_cb, disconnect_cb);
    pub_rect_       = it_.advertise("image_rect", 1, connect_cb, disconnect_cb);
    pub_color_      = it_.advertise("image_color", 1, connect_cb, disconnect_cb);
    pub_rect_color_ = it_.advertise("image_rect_color", 1, connect_cb, disconnect_cb);
  }

  void connectCb(const image_transport::SingleSubscriberPublisher& ssp)
  {
    if (subscriber_count_++ == 0) {
      ROS_DEBUG("Subscribing to camera topics");
      cam_sub_ = it_.subscribeCamera("image_raw", 3, &ImageProc::imageCb, this);
    }
  }

  void disconnectCb(const image_transport::SingleSubscriberPublisher&)
  {
    subscriber_count_--;
    if (subscriber_count_ == 0) {
      ROS_DEBUG("Unsubscribing from camera topics");
      cam_sub_.shutdown();
    }
  }

  bool doColorize()
  {
    //return pub_color_.getNumSubscribers() > 0 || pub_rect_color_.getNumSubscribers() > 0;
    /// @todo Operations required really depend on subscribed topics and image encoding...
    return true;
  }

  bool doRectify()
  {
    return pub_rect_.getNumSubscribers() > 0 || pub_rect_color_.getNumSubscribers() > 0;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& raw_image, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    cam_bridge::RawToCamData(*raw_image, *cam_info, cam::IMAGE_RAW, &img_data_);

    if (doColorize())
    {
      /// @todo Parameter for which bayer interpolation to use
      /// @todo Might need mono only, call doBayerMono() in that case
      //img_data_.colorConvertType = COLOR_CONVERSION_EDGE;
      img_data_.doBayerColorRGB();
      //img_data_.doBayerMono();
    }

    if (doRectify())
      img_data_.doRectify();

    // Publish images
    img_.header.stamp = raw_image->header.stamp;
    img_.header.frame_id = raw_image->header.frame_id;
    publishImage(img_data_.imType, img_data_.im, img_data_.imSize, pub_mono_);
    publishImage(img_data_.imColorType, img_data_.imColor, img_data_.imColorSize, pub_color_);
    publishImage(img_data_.imRectType, img_data_.imRect, img_data_.imRectSize, pub_rect_);
    publishImage(img_data_.imRectColorType, img_data_.imRectColor, img_data_.imRectColorSize, pub_rect_color_);
  }

  void publishImage(color_coding_t coding, void* data, size_t dataSize, const image_transport::Publisher& pub)
  {
    if (coding == COLOR_CODING_NONE)
      return;

    uint32_t step = dataSize / img_data_.imHeight;
    fillImage(img_, cam_bridge::ColorCodingToImageEncoding(coding),
              img_data_.imHeight, img_data_.imWidth, step, data);
    pub.publish(img_);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_proc", ros::init_options::AnonymousName);
  if (ros::names::remap("camera") != "camera") {
    ROS_WARN("[image_proc] Remapping 'camera' no longer has any effect! Start image_proc in the "
             "camera namespace instead.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=%s rosrun image_proc image_proc", ros::names::remap("camera").c_str());
  }

  ros::NodeHandle nh;
  ImageProc proc(nh);

  ros::spin();
  return 0;
}
