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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/CameraInfo.h>

#include "image_proc/image.h"
#include "image_proc/cam_bridge.h"

#include <boost/thread.hpp>

using namespace std;

// @todo: also support when camera is not calibrated!
class ImageProc
{
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
  ros::Publisher mono_pub_;
  ros::Publisher color_pub_;
  ros::Publisher rect_pub_;
  ros::Publisher rect_color_pub_;
  
  bool do_colorize_;
  bool do_rectify_;

  // @todo: maybe these should not be members?
  cam::ImageData img_data_;
  sensor_msgs::Image img_;

public:

  ImageProc(const ros::NodeHandle& nh) : nh_(nh), sync_(3)
  {
    std::string cam_name = nh_.resolveName("camera") + "/";
    nh_.param(cam_name + "do_colorize", do_colorize_, false);
    nh_.param(cam_name + "do_rectify", do_rectify_, false);

    // Advertise outputs
    mono_pub_       = nh_.advertise<sensor_msgs::Image>(cam_name + "image", 1);
    color_pub_      = nh_.advertise<sensor_msgs::Image>(cam_name + "image_color", 1);
    rect_pub_       = nh_.advertise<sensor_msgs::Image>(cam_name + "image_rect", 1);
    rect_color_pub_ = nh_.advertise<sensor_msgs::Image>(cam_name + "image_rect_color", 1);

    // Subscribe to synchronized Image & CameraInfo topics
    image_sub_.subscribe(nh_, cam_name + "image_raw", 1);
    info_sub_.subscribe(nh_, cam_name + "cam_info", 1);
    sync_.connectInput(image_sub_, info_sub_);
    sync_.registerCallback(boost::bind(&ImageProc::imageCb, this, _1, _2));
  }

  void imageCb(const sensor_msgs::ImageConstPtr& raw_image, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    cam_bridge::RawStereoToCamData(*raw_image, *cam_info, stereo_msgs::RawStereo::IMAGE_RAW, &img_data_);

    // @todo: only do processing if topics have subscribers
    // @todo: parameter for bayer interpolation to use
    if (do_colorize_) {
      //img_data_.colorConvertType = COLOR_CONVERSION_EDGE;
      img_data_.doBayerColorRGB();
      //img_data_.doBayerMono();
    }

    if (do_rectify_)
      img_data_.doRectify();

    // Publish images
    img_.header.stamp = raw_image->header.stamp;
    img_.header.frame_id = raw_image->header.frame_id;
    publishImage(img_data_.imType, img_data_.im, img_data_.imSize, mono_pub_);
    publishImage(img_data_.imColorType, img_data_.imColor, img_data_.imColorSize, color_pub_);
    publishImage(img_data_.imRectType, img_data_.imRect, img_data_.imRectSize, rect_pub_);
    publishImage(img_data_.imRectColorType, img_data_.imRectColor, img_data_.imRectColorSize, rect_color_pub_);
  }

  void publishImage(color_coding_t coding, void* data, size_t dataSize, const ros::Publisher& pub)
  {
    if (coding == COLOR_CODING_NONE)
      return;

    // @todo: step calculation is a little hacky
    fillImage(img_, cam_bridge::ColorCodingToImageEncoding(coding),
              img_data_.imHeight, img_data_.imWidth,
              dataSize / img_data_.imHeight /*step*/, data);
    pub.publish(img_);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imageproc");
  ros::NodeHandle nh;
  ImageProc ip(nh);

  ros::spin();

  return 0;
}
