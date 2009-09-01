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

#include "ros/node.h"

#include "image_proc/image.h"
#include "image_proc/cam_bridge.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/fill_image.h"
#include "sensor_msgs/CameraInfo.h"

#include <boost/thread.hpp>

using namespace std;

class ImageProc
{
  ros::Node &node_;
  
  bool do_colorize_;
  bool do_rectify_;

  sensor_msgs::Image raw_img_;
  sensor_msgs::CameraInfo cam_info_;
  bool have_cam_info_;
  boost::mutex cam_info_mutex_;

  sensor_msgs::Image img_;

  std::string cam_name_;

public:

  cam::ImageData img_data_;

  ImageProc(ros::Node &node) : node_(node), have_cam_info_(false)
  {
    cam_name_ = node_.mapName("camera") + "/";
    node_.param(cam_name_ + "do_colorize", do_colorize_, false);
    node_.param(cam_name_ + "do_rectify", do_rectify_, false);

    advertiseImages();
    node_.subscribe(cam_name_ + "cam_info", cam_info_, &ImageProc::camInfoCb, this, 1);
    node_.subscribe(cam_name_ + "image_raw", raw_img_, &ImageProc::rawCb, this, 1);
  }

  // @todo: synchronize callbacks
  void camInfoCb()
  {
    boost::lock_guard<boost::mutex> guard(cam_info_mutex_);
    have_cam_info_ = true;
  }
  
  void rawCb()
  {
    // @todo: move publishing into here, only do processing if topics have subscribers
    boost::lock_guard<boost::mutex> guard(cam_info_mutex_);

    cam_bridge::RawStereoToCamData(raw_img_, cam_info_, stereo_msgs::RawStereo::IMAGE_RAW, &img_data_);

    if (do_colorize_) {
      //img_data_.colorConvertType = COLOR_CONVERSION_EDGE;
      img_data_.doBayerColorRGB();
      //img_data_.doBayerMono();
    }

    if (do_rectify_ && have_cam_info_)
      img_data_.doRectify();

    publishImages();
  }

  void publishImage(color_coding_t coding, void* data, size_t dataSize, const std::string& topic)
  {
    if (coding == COLOR_CODING_NONE)
      return;

    // @todo: step calculation is a little hacky
    fillImage(img_, cam_bridge::ColorCodingToImageEncoding(coding),
              img_data_.imHeight, img_data_.imWidth,
              dataSize / img_data_.imHeight /*step*/, data);
    node_.publish(cam_name_ + topic, img_);
  }

  void publishImages()
  {
    img_.header.stamp = raw_img_.header.stamp;
    img_.header.frame_id = raw_img_.header.frame_id;

    publishImage(img_data_.imType, img_data_.im, img_data_.imSize, "image");
    publishImage(img_data_.imColorType, img_data_.imColor, img_data_.imColorSize, "image_color");
    publishImage(img_data_.imRectType, img_data_.imRect, img_data_.imRectSize, "image_rect");
    publishImage(img_data_.imRectColorType, img_data_.imRectColor, img_data_.imRectColorSize, "image_rect_color");
  }

  void advertiseImages()
  {
    node_.advertise<sensor_msgs::Image>(cam_name_ + "image", 1);
    node_.advertise<sensor_msgs::Image>(cam_name_ + "image_color", 1);
    node_.advertise<sensor_msgs::Image>(cam_name_ + "image_rect", 1);
    node_.advertise<sensor_msgs::Image>(cam_name_ + "image_rect_color", 1);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("imageproc");
  ImageProc ip(n);

  n.spin();

  return 0;
}
