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
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/fill_image.h>

#include <opencv_latest/CvBridge.h>
#include <image_transport/image_publisher.h>

#include "stereoimage.h"
#include "cam_bridge.h"

#include <boost/thread.hpp>

using namespace std;

//
// This is the node creation file
// Subscribes to two Image/CameraInfo topics, and performs rectification,
//   color processing, and stereo disparity on the images
//

// @TODO: add in diagnostic frequency messages

class StereoProc
{
private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_l, image_sub_r;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_l, info_sub_r;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, 
				    sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
  
  bool do_colorize_;
  bool do_rectify_;
  bool do_stereo_;
  bool do_calc_points_;
  bool do_keep_coords_;

  image_transport::ImagePublisher pub_mono_l_;
  image_transport::ImagePublisher pub_rect_l_;
  image_transport::ImagePublisher pub_color_l_;
  image_transport::ImagePublisher pub_rect_color_l_;
  image_transport::ImagePublisher pub_mono_r_;
  image_transport::ImagePublisher pub_rect_r_;
  image_transport::ImagePublisher pub_color_r_;
  image_transport::ImagePublisher pub_rect_color_r_;
  image_transport::ImagePublisher pub_disparity_;

  // @todo: maybe these should not be members?
  sensor_msgs::Image img_;
  stereo_msgs::DisparityImage dimg_;


public:

  cam::StereoData* stdata_;

  StereoProc(const ros::NodeHandle& nh) : nh_(nh), sync_(3)
  {
    // get standard parameters
    // @TODO use nodehandle with "~" as namespace
    nh_.param("~do_colorize", do_colorize_, false);
    nh_.param("~do_rectify", do_rectify_, false);
    nh_.param("~do_stereo", do_stereo_, true);
    nh_.param("~do_calc_points", do_calc_points_, false);
    nh_.param("~do_keep_coords", do_keep_coords_, false);

    // set up stereo structures
    if (do_keep_coords_) {
    	ROS_INFO("I'm keeping the image coordinate in the point cloud\n");
    }
    // Must do stereo if calculating points
    do_stereo_ = do_stereo_ || do_calc_points_;
    // Must rectify if doing stereo
    do_rectify_ = do_rectify_ || do_stereo_;

    stdata_ = new cam::StereoData;

    int unique_thresh;
    nh_.param("~unique_thresh", unique_thresh, 36);
    stdata_->setTextureThresh(unique_thresh);
    int texture_thresh;
    nh_.param("~texture_thresh", texture_thresh, 30);
    stdata_->setUniqueThresh(texture_thresh);
    int speckle_size;
    nh_.param("~speckle_size", speckle_size, 100);
    stdata_->setSpeckleRegionSize(speckle_size);
    int speckle_diff;
    nh_.param("~speckle_diff", speckle_diff, 10);
    stdata_->setSpeckleDiff(speckle_diff);
    int smoothness_thresh;
    if (nh_.getParam("~smoothness_thresh", smoothness_thresh))
      stdata_->setSmoothnessThresh(smoothness_thresh);
    int horopter;
    if (nh_.getParam("~horopter", horopter))
      stdata_->setHoropter(horopter);
    int corr_size;
    if (nh_.getParam("~corr_size", corr_size))
      stdata_->setCorrSize(corr_size);
    int num_disp;
    if (nh_.getParam("~num_disp", num_disp))
      stdata_->setNumDisp(num_disp);


    // resolve names, advertise and subscribe

    std::string cam_name_l = nh_.resolveName("camera_left") + "/";
    std::string cam_name_r = nh_.resolveName("camera_right") + "/";
    std::string cam_name_s = nh_.resolveName("stereo") + "/";

    // Advertise outputs
    // @TODO parameters can change, we don't check
    pub_mono_l_.advertise(nh_, cam_name_l+"image_mono", 1);
    pub_mono_r_.advertise(nh_, cam_name_r+"image_mono", 1);
    if (do_rectify_)
      {
	pub_rect_l_.advertise(nh_, cam_name_l+"image_rect", 1);
	pub_rect_r_.advertise(nh_, cam_name_r+"image_rect", 1);
      }
    if (do_colorize_)
      {
	pub_color_l_.advertise(nh_, cam_name_l+"image_color", 1);
	pub_color_r_.advertise(nh_, cam_name_r+"image_color", 1);
	if (do_rectify_)
	  {
	    pub_rect_color_l_.advertise(nh_, cam_name_l+"image_rect_color", 1);
	    pub_rect_color_r_.advertise(nh_, cam_name_r+"image_rect_color", 1);
	  }
      }
    if (do_stereo_)
      pub_disparity_.advertise(nh_, cam_name_s+"image_disparity", 1);

    // Subscribe to synchronized Image & CameraInfo topics
    image_sub_l.subscribe(nh_, cam_name_l + "image_raw", 1);
    info_sub_l.subscribe(nh_, cam_name_l + "camera_info", 1);
    image_sub_r.subscribe(nh_, cam_name_r + "image_raw", 1);
    info_sub_r.subscribe(nh_, cam_name_r + "camera_info", 1);
    sync_.connectInput(image_sub_l, info_sub_l, image_sub_r, info_sub_r);
    sync_.registerCallback(boost::bind(&StereoProc::imageCb, this, _1, _2, _3, _4));
  }

  // teardown
  ~StereoProc()
  {
    if (stdata_)
      delete stdata_;
  }

  // callback
  // gets 2 images and 2 parameter sets, computes disparity
  void imageCb(const sensor_msgs::ImageConstPtr& raw_image_l, const sensor_msgs::CameraInfoConstPtr& cam_info_l,
	       const sensor_msgs::ImageConstPtr& raw_image_r, const sensor_msgs::CameraInfoConstPtr& cam_info_r)
  {
    // @TODO: check image sizes for comatibility
    cam::ImageData *img_data_l, *img_data_r;
    img_data_l = stdata_->imLeft;
    img_data_r = stdata_->imRight;

    // copy timestamps
    stdata_->imLeft->im_time = raw_image_l->header.stamp.toNSec() / 1000;
    stdata_->imRight->im_time = raw_image_l->header.stamp.toNSec() / 1000;
    // set size
    stdata_->setSize(raw_image_l->width, raw_image_l->height);
    stdata_->hasDisparity = false;

    // @TODO get T, Om, RP
    // RP can be computed from CameraInfo's
    // T and Om aren't needed

    // copy data
    cam_bridge::RawToCamData(*raw_image_l, *cam_info_l, cam::IMAGE_RAW, img_data_l);
    cam_bridge::RawToCamData(*raw_image_r, *cam_info_r, cam::IMAGE_RAW, img_data_r);

    // @todo: only do processing if topics have subscribers
    // @todo: parameter for bayer interpolation to use
    if (do_colorize_ && 
	(pub_color_l_.getNumSubscribers() > 0 ||
	 pub_rect_color_l_.getNumSubscribers() > 0))
      img_data_l->doBayerColorRGB();

    if (do_colorize_ && 
	(pub_color_r_.getNumSubscribers() > 0 ||
	 pub_rect_color_r_.getNumSubscribers() > 0))
      img_data_r->doBayerColorRGB();

    // @TODO check subscribers
    if (do_rectify_)
      {
	//	ROS_INFO("[stereo_image_proc] Rectifying");
	stdata_->doRectify();
      }

    if (do_stereo_)
      {
	//	ROS_INFO("[stereo_image_proc] Disparity calc");
	stdata_->doDisparity();
      }

    if (do_calc_points_)
      stdata_->doCalcPts();


    // Publish images
    img_.header.stamp = raw_image_l->header.stamp;
    img_.header.frame_id = raw_image_l->header.frame_id;

    // left
    publishImage(img_data_l->imType, img_data_l->im, img_data_l->imSize, pub_mono_l_);
    publishImage(img_data_l->imColorType, img_data_l->imColor, img_data_l->imColorSize, pub_color_l_);
    publishImage(img_data_l->imRectType, img_data_l->imRect, img_data_l->imRectSize, pub_rect_l_);
    publishImage(img_data_l->imRectColorType, img_data_l->imRectColor, img_data_l->imRectColorSize, pub_rect_color_l_);
    publishImage(img_data_l->imType, img_data_l->im, img_data_l->imSize, pub_mono_r_);
    publishImage(img_data_l->imColorType, img_data_l->imColor, img_data_l->imColorSize, pub_color_r_);
    publishImage(img_data_l->imRectType, img_data_l->imRect, img_data_l->imRectSize, pub_rect_r_);
    publishImage(img_data_l->imRectColorType, img_data_l->imRectColor, img_data_l->imRectColorSize, pub_rect_color_r_);
    // right
    publishImage(img_data_r->imType, img_data_r->im, img_data_r->imSize, pub_mono_r_);
    publishImage(img_data_r->imColorType, img_data_r->imColor, img_data_r->imColorSize, pub_color_r_);
    publishImage(img_data_r->imRectType, img_data_r->imRect, img_data_r->imRectSize, pub_rect_r_);
    publishImage(img_data_r->imRectColorType, img_data_r->imRectColor, img_data_r->imRectColorSize, pub_rect_color_r_);
    publishImage(img_data_r->imType, img_data_r->im, img_data_r->imSize, pub_mono_r_);
    publishImage(img_data_r->imColorType, img_data_r->imColor, img_data_r->imColorSize, pub_color_r_);
    publishImage(img_data_r->imRectType, img_data_r->imRect, img_data_r->imRectSize, pub_rect_r_);
    publishImage(img_data_r->imRectColorType, img_data_r->imRectColor, img_data_r->imRectColorSize, pub_rect_color_r_);

    // disparity
    if (stdata_->hasDisparity)
      {
	dimg_.header.stamp = raw_image_l->header.stamp;
	dimg_.header.frame_id = raw_image_l->header.frame_id;
	dimg_.dpp = stdata_->dpp;
	dimg_.num_disp = stdata_->numDisp;
	dimg_.im_Dtop = stdata_->imDtop;
	dimg_.im_Dleft = stdata_->imDleft;
	dimg_.im_Dwidth = stdata_->imDwidth;
	dimg_.im_Dheight = stdata_->imDheight;
	publishImageDisparity(stdata_->imDisp, stdata_->imDispSize, pub_disparity_);
      }

  }

  void publishImage(color_coding_t coding, void* data, size_t dataSize, const image_transport::ImagePublisher& pub)
  {
    if (coding == COLOR_CODING_NONE)
      return;

    // @todo: step calculation is a little hacky
    fillImage(img_, cam_bridge::ColorCodingToImageEncoding(coding),
              stdata_->imHeight, stdata_->imWidth,
              dataSize / stdata_->imHeight /*step*/, data);
    pub.publish(img_);
  }

  bool fillImageDisparity(stereo_msgs::DisparityImage& image,
                 std::string encoding_arg,
                 uint32_t rows_arg,
                 uint32_t cols_arg,
                 uint32_t step_arg,
                 void* data_arg)
  {
    image.encoding = encoding_arg;
    image.height   = rows_arg;
    image.width    = cols_arg;
    image.step     = step_arg;
    size_t st0 = (step_arg * rows_arg);
    image.data.resize(st0);
    memcpy((char*)(&image.data[0]), (char*)(data_arg), st0);

    image.is_bigendian = 0;
    return true;
  }

  void publishImageDisparity(void* data, size_t dataSize, const image_transport::ImagePublisher& pub)
  {
    // @todo: step calculation is a little hacky
    fillImageDisparity(dimg_, sensor_msgs::image_encodings::MONO16,
              stdata_->imHeight, stdata_->imWidth,
              dataSize / stdata_->imHeight /*step*/, data);
    pub.publish(img_);
  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_proc", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  if (nh.resolveName("camera_left") == "/camera_left" ||
      nh.resolveName("camera_right") == "/camera_right")
    {
      ROS_WARN("image_view: source image has not been remapped! Example command-line usage:\n"
	       "\t$ rosrun stere_image_proc stereo_image_proc camera_left:=/forearm-l camera_right:=/forearm-r");
    }
  
  StereoProc proc(nh);

  ros::spin();
  
  return 0;
}
