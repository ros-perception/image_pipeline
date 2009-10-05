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
#include <sensor_msgs/PointCloud.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/fill_image.h>

#include <opencv_latest/CvBridge.h>
#include <image_transport/image_publisher.h>

#include "stereo_image_proc/stereoimage.h"
#include "image_proc/cam_bridge.h"

#include <boost/thread.hpp>

using namespace std;

// colormap for disparities
static unsigned char dmap[768] = 
  { 150, 150, 150,
    107, 0, 12,
    106, 0, 18,
    105, 0, 24,
    103, 0, 30,
    102, 0, 36,
    101, 0, 42,
    99, 0, 48,
    98, 0, 54,
    97, 0, 60,
    96, 0, 66,
    94, 0, 72,
    93, 0, 78,
    92, 0, 84,
    91, 0, 90,
    89, 0, 96,
    88, 0, 102,
    87, 0, 108,
    85, 0, 114,
    84, 0, 120,
    83, 0, 126,
    82, 0, 131,
    80, 0, 137,
    79, 0, 143,
    78, 0, 149,
    77, 0, 155,
    75, 0, 161,
    74, 0, 167,
    73, 0, 173,
    71, 0, 179,
    70, 0, 185,
    69, 0, 191,
    68, 0, 197,
    66, 0, 203,
    65, 0, 209,
    64, 0, 215,
    62, 0, 221,
    61, 0, 227,
    60, 0, 233,
    59, 0, 239,
    57, 0, 245,
    56, 0, 251,
    55, 0, 255,
    54, 0, 255,
    52, 0, 255,
    51, 0, 255,
    50, 0, 255,
    48, 0, 255,
    47, 0, 255,
    46, 0, 255,
    45, 0, 255,
    43, 0, 255,
    42, 0, 255,
    41, 0, 255,
    40, 0, 255,
    38, 0, 255,
    37, 0, 255,
    36, 0, 255,
    34, 0, 255,
    33, 0, 255,
    32, 0, 255,
    31, 0, 255,
    29, 0, 255,
    28, 0, 255,
    27, 0, 255,
    26, 0, 255,
    24, 0, 255,
    23, 0, 255,
    22, 0, 255,
    20, 0, 255,
    19, 0, 255,
    18, 0, 255,
    17, 0, 255,
    15, 0, 255,
    14, 0, 255,
    13, 0, 255,
    11, 0, 255,
    10, 0, 255,
    9, 0, 255,
    8, 0, 255,
    6, 0, 255,
    5, 0, 255,
    4, 0, 255,
    3, 0, 255,
    1, 0, 255,
    0, 4, 255,
    0, 10, 255,
    0, 16, 255,
    0, 22, 255,
    0, 28, 255,
    0, 34, 255,
    0, 40, 255,
    0, 46, 255,
    0, 52, 255,
    0, 58, 255,
    0, 64, 255,
    0, 70, 255,
    0, 76, 255,
    0, 82, 255,
    0, 88, 255,
    0, 94, 255,
    0, 100, 255,
    0, 106, 255,
    0, 112, 255,
    0, 118, 255,
    0, 124, 255,
    0, 129, 255,
    0, 135, 255,
    0, 141, 255,
    0, 147, 255,
    0, 153, 255,
    0, 159, 255,
    0, 165, 255,
    0, 171, 255,
    0, 177, 255,
    0, 183, 255,
    0, 189, 255,
    0, 195, 255,
    0, 201, 255,
    0, 207, 255,
    0, 213, 255,
    0, 219, 255,
    0, 225, 255,
    0, 231, 255,
    0, 237, 255,
    0, 243, 255,
    0, 249, 255,
    0, 255, 255,
    0, 255, 249,
    0, 255, 243,
    0, 255, 237,
    0, 255, 231,
    0, 255, 225,
    0, 255, 219,
    0, 255, 213,
    0, 255, 207,
    0, 255, 201,
    0, 255, 195,
    0, 255, 189,
    0, 255, 183,
    0, 255, 177,
    0, 255, 171,
    0, 255, 165,
    0, 255, 159,
    0, 255, 153,
    0, 255, 147,
    0, 255, 141,
    0, 255, 135,
    0, 255, 129,
    0, 255, 124,
    0, 255, 118,
    0, 255, 112,
    0, 255, 106,
    0, 255, 100,
    0, 255, 94,
    0, 255, 88,
    0, 255, 82,
    0, 255, 76,
    0, 255, 70,
    0, 255, 64,
    0, 255, 58,
    0, 255, 52,
    0, 255, 46,
    0, 255, 40,
    0, 255, 34,
    0, 255, 28,
    0, 255, 22,
    0, 255, 16,
    0, 255, 10,
    0, 255, 4,
    2, 255, 0,
    8, 255, 0,
    14, 255, 0,
    20, 255, 0,
    26, 255, 0,
    32, 255, 0,
    38, 255, 0,
    44, 255, 0,
    50, 255, 0,
    56, 255, 0,
    62, 255, 0,
    68, 255, 0,
    74, 255, 0,
    80, 255, 0,
    86, 255, 0,
    92, 255, 0,
    98, 255, 0,
    104, 255, 0,
    110, 255, 0,
    116, 255, 0,
    122, 255, 0,
    128, 255, 0,
    133, 255, 0,
    139, 255, 0,
    145, 255, 0,
    151, 255, 0,
    157, 255, 0,
    163, 255, 0,
    169, 255, 0,
    175, 255, 0,
    181, 255, 0,
    187, 255, 0,
    193, 255, 0,
    199, 255, 0,
    205, 255, 0,
    211, 255, 0,
    217, 255, 0,
    223, 255, 0,
    229, 255, 0,
    235, 255, 0,
    241, 255, 0,
    247, 255, 0,
    253, 255, 0,
    255, 251, 0,
    255, 245, 0,
    255, 239, 0,
    255, 233, 0,
    255, 227, 0,
    255, 221, 0,
    255, 215, 0,
    255, 209, 0,
    255, 203, 0,
    255, 197, 0,
    255, 191, 0,
    255, 185, 0,
    255, 179, 0,
    255, 173, 0,
    255, 167, 0,
    255, 161, 0,
    255, 155, 0,
    255, 149, 0,
    255, 143, 0,
    255, 137, 0,
    255, 131, 0,
    255, 126, 0,
    255, 120, 0,
    255, 114, 0,
    255, 108, 0,
    255, 102, 0,
    255, 96, 0,
    255, 90, 0,
    255, 84, 0,
    255, 78, 0,
    255, 72, 0,
    255, 66, 0,
    255, 60, 0,
    255, 54, 0,
    255, 48, 0,
    255, 42, 0,
    255, 36, 0,
    255, 30, 0,
    255, 24, 0,
    255, 18, 0,
    255, 12, 0,
    255,  6, 0,
    255,  0, 0,
  };

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
  image_transport::ImagePublisher pub_disparity_image_;
  ros::Publisher pub_disparity_;
  ros::Publisher pub_pts_;


  // @todo: maybe these should not be members?
  sensor_msgs::Image img_;
  stereo_msgs::DisparityImage dimg_;


public:

  cam::StereoData* stdata_;

  StereoProc(const ros::NodeHandle& nh) : nh_(nh), sync_(3)
  {
    // get standard parameters
    // @TODO use nodehandle with "~" as namespace
    ros::NodeHandle lnh("~");
    lnh.param("do_colorize", do_colorize_, false);
    lnh.param("do_rectify", do_rectify_, false);
    lnh.param("do_stereo", do_stereo_, true);
    lnh.param("do_calc_points", do_calc_points_, false);
    lnh.param("do_keep_coords", do_keep_coords_, false);

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
    lnh.param("unique_thresh", unique_thresh, 36);
    stdata_->setTextureThresh(unique_thresh);
    int texture_thresh;
    lnh.param("texture_thresh", texture_thresh, 30);
    stdata_->setUniqueThresh(texture_thresh);
    int speckle_size;
    lnh.param("speckle_size", speckle_size, 100);
    stdata_->setSpeckleRegionSize(speckle_size);
    int speckle_diff;
    lnh.param("speckle_diff", speckle_diff, 10);
    stdata_->setSpeckleDiff(speckle_diff);
    int smoothness_thresh;
    if (lnh.getParam("smoothness_thresh", smoothness_thresh))
      stdata_->setSmoothnessThresh(smoothness_thresh);
    int horopter;
    if (lnh.getParam("horopter", horopter))
      stdata_->setHoropter(horopter);
    int corr_size;
    if (lnh.getParam("corr_size", corr_size))
      stdata_->setCorrSize(corr_size);
    int num_disp;
    if (lnh.getParam("num_disp", num_disp))
      stdata_->setNumDisp(num_disp);


    // resolve names, advertise and subscribe

    std::string cam_name_l;
    std::string cam_name_r;
    std::string cam_name_s;
    if (nh.resolveName("image") != "/image") // we're looking at a stereo cam
      {
	cam_name_l = nh_.resolveName("image") + "/left/";
	cam_name_r = nh_.resolveName("image") + "/right/";
	cam_name_s = nh_.resolveName("image") + "/";
      }
    else
      {
	cam_name_l = nh_.resolveName("image_left") + "/";
	cam_name_r = nh_.resolveName("image_right") + "/";
	cam_name_s = nh_.resolveName("image_left") + "/";
      }

    if (nh.resolveName("output") != "/output") // remap of output
      {
	cam_name_s = nh_.resolveName("output") + "/";	
      }

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
      {
	pub_disparity_image_.advertise(nh_, cam_name_s+"image_disparity", 1);
	pub_disparity_ = nh_.advertise<stereo_msgs::DisparityImage>(cam_name_s+"disparity", 1);
      }

    if (do_calc_points_)
      {
	pub_pts_ = nh_.advertise<sensor_msgs::PointCloud>(cam_name_s+"points", 1);
      }


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
  void imageCb(const sensor_msgs::ImageConstPtr& raw_image_l, 
	       const sensor_msgs::CameraInfoConstPtr& cam_info_l,
	       const sensor_msgs::ImageConstPtr& raw_image_r, 
	       const sensor_msgs::CameraInfoConstPtr& cam_info_r)
  {
    // @TODO: check image sizes for compatibility
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
    stdata_->setReprojection();	// set up the reprojection matrix

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
    if (do_rectify_ &&
	(pub_rect_color_r_.getNumSubscribers() > 0 ||
	 pub_rect_r_.getNumSubscribers() > 0 ||
	 pub_rect_color_l_.getNumSubscribers() > 0 ||
	 pub_rect_l_.getNumSubscribers() > 0 ||
	 pub_rect_r_.getNumSubscribers() > 0 ||
	 pub_disparity_.getNumSubscribers() > 0 ||
	 pub_disparity_image_.getNumSubscribers() > 0 ||
	 pub_pts_.getNumSubscribers() > 0))
      {
	//	ROS_INFO("[stereo_image_proc] Rectifying");
	stdata_->doRectify();
      }

    if (do_stereo_ &&
	(pub_disparity_.getNumSubscribers() > 0 ||
	 pub_disparity_image_.getNumSubscribers() > 0 ||
	 pub_pts_.getNumSubscribers() > 0))
      {
	//	ROS_INFO("[stereo_image_proc] Disparity calc");
	stdata_->doDisparity();
      }

    if (do_calc_points_ &&
	pub_pts_.getNumSubscribers() > 0)
      {
	//	ROS_INFO("[stereo_image_proc] 3D Points calc");
	stdata_->doCalcPts();
      }

    // Publish images
    img_.header.stamp = raw_image_l->header.stamp;
    img_.header.frame_id = raw_image_l->header.frame_id;

    // left
    publishImage(img_data_l->imType, img_data_l->im, img_data_l->imSize, pub_mono_l_);
    publishImage(img_data_l->imColorType, img_data_l->imColor, img_data_l->imColorSize, pub_color_l_);
    publishImage(img_data_l->imRectType, img_data_l->imRect, img_data_l->imRectSize, pub_rect_l_);
    publishImage(img_data_l->imRectColorType, img_data_l->imRectColor, img_data_l->imRectColorSize, pub_rect_color_l_);
    // right
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

	dimg_.f = (*cam_info_l).K[0];
	dimg_.cx = (*cam_info_l).K[2];
	dimg_.cy = (*cam_info_l).K[5];
	if ((*cam_info_r).P[3] > 1e-10)
	  dimg_.Tx = (*cam_info_r).P[0] / (*cam_info_r).P[3];
	else
	  dimg_.Tx =0.0;

	publishImageDisparity(stdata_->imDisp, stdata_->imDispSize);
      }


    // point cloud
    if (stdata_->numPts > 0)
      {
	sensor_msgs::PointCloud cloud_;
	cloud_.header.stamp = raw_image_l->header.stamp;
	cloud_.header.frame_id = raw_image_l->header.frame_id;
	cloud_.points.resize(stdata_->numPts);
	if (do_keep_coords_) 
    	  cloud_.channels.resize(3);
	else
    	  cloud_.channels.resize(1);
	cloud_.channels[0].name = "rgb";
	cloud_.channels[0].values.resize(stdata_->numPts);
	if (do_keep_coords_) 
	  {
	    cloud_.channels[1].name = "x";
	    cloud_.channels[1].values.resize(stdata_->numPts);
	    cloud_.channels[2].name = "y";
	    cloud_.channels[2].values.resize(stdata_->numPts);
	  }

	for (int i = 0; i < stdata_->numPts; i++)
	  {
	    cloud_.points[i].x = stdata_->imPts[3*i + 0];
	    cloud_.points[i].y = stdata_->imPts[3*i + 1];
	    cloud_.points[i].z = stdata_->imPts[3*i + 2];
	  }

	for (int i = 0; i < stdata_->numPts; i++)
	  {
	    int rgb = (stdata_->imPtsColor[3*i] << 16) | (stdata_->imPtsColor[3*i + 1] << 8) | stdata_->imPtsColor[3*i + 2];
	    cloud_.channels[0].values[i] = *(float*)(&rgb);
	    if (do_keep_coords_) 
	      {
		cloud_.channels[1].values[i] = stdata_->imCoords[2*i+0];
		cloud_.channels[2].values[i] = stdata_->imCoords[2*i+1];
	      }
	  }
	pub_pts_.publish(cloud_);
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

  bool fillImageDisparity(stereo_msgs::DisparityImage& dimage,
			  sensor_msgs::Image& image,
			  std::string encoding_arg,
			  uint32_t rows_arg,
			  uint32_t cols_arg,
			  uint32_t step_arg, // row size in bytes
			  void* data_arg)
  {
    // first do disparity image
    dimage.encoding = encoding_arg;
    dimage.height   = rows_arg;
    dimage.width    = cols_arg;
    dimage.step     = step_arg;
    size_t st0 = (step_arg * rows_arg);
    dimage.data.resize(st0);
    memcpy((char*)(&dimage.data[0]), (char*)(data_arg), st0);
    dimage.is_bigendian = 0;

    // now do regular image
    if (1)			// should check if we're subscribed
      {
	int nd = dimg_.num_disp;
	int dpp = dimg_.dpp;
	nd = nd*dpp;		// total number of sub-disparities
	int sh = 0;
	while (nd > 256)
	  { sh++; nd = nd>>1; }
	step_arg = cols_arg*3;
	image.encoding = sensor_msgs::image_encodings::RGB8,
	image.height   = rows_arg;
	image.width    = cols_arg;
	image.step     = step_arg;
	size_t st0 = (step_arg * rows_arg);
	image.data.resize(st0);
	uint16_t *ddata = (uint16_t *)data_arg;

	for (size_t i=0; i<st0; i+=3, ddata++)
	  {
	    int k = 3 * (0xff & (*ddata)>>sh);
	    image.data[i] = dmap[k];
	    image.data[i+1] = dmap[k+1];
	    image.data[i+2] = dmap[k+2];
	  }
	image.is_bigendian = 0;
      }

    return true;
  }

  void publishImageDisparity(void* data, size_t dataSize)
  {
    // @todo: step calculation is a little hacky
    fillImageDisparity(dimg_, img_, sensor_msgs::image_encodings::MONO16,
              stdata_->imHeight, stdata_->imWidth,
              dataSize / stdata_->imHeight /*step*/, data);
    pub_disparity_image_.publish(img_);
    pub_disparity_.publish(dimg_);
  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_proc", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  // resolve either <image> for both cams, or individual stereo cams
  if (nh.resolveName("image") == "/image" && 
      (nh.resolveName("image_left") == "/image_left" ||
       nh.resolveName("image_right") == "/image_right"))
    {
      ROS_WARN("[stereo_image_proc] Remap either <image> or <image_left> and <image_right>");
    }
  
  // resolve either <image> for both cams, or individual stereo cams
  if (nh.resolveName("image") != "/image" && 
      nh.resolveName("output") == "/output")
    {
      std::string name = nh.resolveName("image_left");
      ROS_WARN("[stereo_image_proc] Output will be %s, remap <output> to change", 
	       name.c_str());
    }

  StereoProc proc(nh);

  ros::spin();
  
  return 0;
}
