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

#include <cstdio>

#include "ros/ros.h"

#include "stereo_msgs/RawStereo.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/fill_image.h"
#include "sensor_msgs/CameraInfo.h"
#include "stereo_msgs/StereoInfo.h"
#include "sensor_msgs/PointCloud.h"

#include "image_proc/image.h"
#include "stereo_image_proc/cam_bridge_old.h"

#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"

using namespace std;

class StereoProc
{
  class ImagePublishers
  {
  public:
    ros::Publisher cam_info_;
    ros::Publisher image_raw_; 
    ros::Publisher image_; 
    ros::Publisher image_color_; 
    ros::Publisher image_rect_; 
    ros::Publisher image_rect_color_; 
  };

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::NodeHandle left_nh_;
  ros::NodeHandle right_nh_;

  ros::Subscriber raw_stereo_sub_;

  ImagePublishers left_pubs_;
  ImagePublishers right_pubs_;

  ros::Publisher stereo_info_pub_;
  ros::Publisher disparity_pub_;
  ros::Publisher disparity_info_pub_;

  ros::Publisher cloud_pub_;

  bool do_colorize_;
  bool do_rectify_;
  bool do_stereo_;
  bool do_calc_points_;
  bool do_keep_coords_;

  sensor_msgs::Image         img_;
  sensor_msgs::PointCloud      cloud_;
  sensor_msgs::CameraInfo       cam_info_;
  stereo_msgs::DisparityInfo disparity_info_;
  stereo_msgs::StereoInfo    stereo_info_;


  diagnostic_updater::Updater diagnostic_;
  ros::Timer diagnostic_timer_;
  diagnostic_updater::FrequencyStatus freq_status_;
  int count_;
  double desired_freq_;

  string frame_id_;



public:

  cam::StereoData* stdata_;

  StereoProc() : private_nh_("~"), left_nh_("left"), right_nh_("right"), diagnostic_(nh_),
                 freq_status_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_)), 
                 count_(0), stdata_(NULL)
  {
    diagnostic_.add( freq_status_ );

    private_nh_.param("do_colorize", do_colorize_, false);

    private_nh_.param("do_rectify", do_rectify_, false);

    private_nh_.param("do_stereo", do_stereo_, false);

    private_nh_.param("do_calc_points", do_calc_points_, false);

    private_nh_.param("do_keep_coords", do_keep_coords_, false);

    if (do_keep_coords_) {
    	ROS_INFO("I'm keeping the image coordinate in the point cloud\n");
    }
    // Must do stereo if calculating points
    do_stereo_ = do_stereo_ || do_calc_points_;

    // Must rectify if doing stereo
    do_rectify_ = do_rectify_ || do_stereo_;

    stdata_ = new cam::StereoData;

    //    stdata_->setUniqueThresh(36);
    //    stdata_->setTextureThresh(30);
    //    stdata_->setSpeckleRegionSize(100);
    //    stdata_->setSpeckleDiff(10);

    int unique_thresh;
    private_nh_.param("unique_thresh", unique_thresh, 36);
    stdata_->setTextureThresh(unique_thresh);

    int texture_thresh;
    private_nh_.param("texture_thresh", texture_thresh, 30);
    stdata_->setUniqueThresh(texture_thresh);

    int speckle_size;
    private_nh_.param("speckle_size", speckle_size, 100);
    stdata_->setSpeckleRegionSize(speckle_size);

    int speckle_diff;
    private_nh_.param("speckle_diff", speckle_diff, 10);
    stdata_->setSpeckleDiff(speckle_diff);
    
    int smoothness_thresh;
    if (private_nh_.getParam("smoothness_thresh", smoothness_thresh))
      stdata_->setSmoothnessThresh(smoothness_thresh);

    int horopter;
    if (private_nh_.getParam("horopter", horopter))
      stdata_->setHoropter(horopter);

    int corr_size;
    if (private_nh_.getParam("corr_size", corr_size))
      stdata_->setCorrSize(corr_size);

    int num_disp;
    if (private_nh_.getParam("num_disp", num_disp))
      stdata_->setNumDisp(num_disp);

    diagnostic_timer_ = nh_.createTimer(ros::Duration(0.1), &StereoProc::diagnosticUpdate, this);

    raw_stereo_sub_ = nh_.subscribe("raw_stereo", 1, &StereoProc::rawCb, this);
  }

  ~StereoProc()
  {
    if (stdata_)
      delete stdata_;
  }

  void rawCb(const stereo_msgs::RawStereo::ConstPtr& raw_stereo)
  {
    cam_bridge::RawStereoToStereoData(*raw_stereo, stdata_);

    if (do_colorize_)
    {
      stdata_->doBayerColorRGB();
    }

    if (do_rectify_)
    {
      stdata_->doRectify();
    }

    if (do_stereo_)
    {
      stdata_->doDisparity();
    }

    if (do_calc_points_)
    {
      stdata_->doCalcPts();
    }

    advertiseCam();
    publishCam(raw_stereo);

    count_++;
  }


  void publishCam(const stereo_msgs::RawStereo::ConstPtr& raw_stereo)
  {
    publishImages(left_pubs_, raw_stereo, stdata_->imLeft);
    publishImages(right_pubs_, raw_stereo, stdata_->imRight);

    if (stdata_->hasDisparity)
    {
      disparity_info_.header.stamp = raw_stereo->header.stamp;
      disparity_info_.header.frame_id = raw_stereo->header.frame_id;

      disparity_info_.height = stdata_->imHeight;
      disparity_info_.width = stdata_->imWidth;

      disparity_info_.dpp = stdata_->dpp;
      disparity_info_.num_disp = stdata_->numDisp;
      disparity_info_.im_Dtop = stdata_->imDtop;
      disparity_info_.im_Dleft = stdata_->imDleft;
      disparity_info_.im_Dwidth = stdata_->imDwidth;
      disparity_info_.im_Dheight = stdata_->imDheight;
      disparity_info_.corr_size = stdata_->corrSize;
      disparity_info_.filter_size = stdata_->filterSize;
      disparity_info_.hor_offset = stdata_->horOffset;
      disparity_info_.texture_thresh = stdata_->textureThresh;
      disparity_info_.unique_thresh = stdata_->uniqueThresh;
      disparity_info_.smooth_thresh = stdata_->smoothThresh;
      disparity_info_.speckle_diff = stdata_->speckleDiff;
      disparity_info_.speckle_region_size = stdata_->speckleRegionSize;
      disparity_info_.unique_check = stdata_->unique_check;

      disparity_info_pub_.publish(disparity_info_);

      fillImage(img_, sensor_msgs::image_encodings::TYPE_16SC1, stdata_->imHeight, stdata_->imWidth, 2 * stdata_->imWidth, stdata_->imDisp);

      img_.header.stamp = raw_stereo->header.stamp;
      img_.header.frame_id = raw_stereo->header.frame_id;
      disparity_pub_.publish(img_);
    }

    if (do_calc_points_)
    {
      cloud_.header.stamp = raw_stereo->header.stamp;
      cloud_.header.frame_id = raw_stereo->header.frame_id;
      cloud_.points.resize(stdata_->numPts);
      if (do_keep_coords_) {
    	  cloud_.channels.resize(3);
      }
      else {
    	  cloud_.channels.resize(1);
      }
      cloud_.channels[0].name = "rgb";
      cloud_.channels[0].values.resize(stdata_->numPts);
      if (do_keep_coords_) {
          cloud_.channels[1].name = "u";
          cloud_.channels[1].values.resize(stdata_->numPts);
          cloud_.channels[2].name = "v";
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
        if (do_keep_coords_) {
        	cloud_.channels[1].values[i] = stdata_->imCoords[2*i+0];
        	cloud_.channels[2].values[i] = stdata_->imCoords[2*i+1];
        }
      }

      cloud_pub_.publish(cloud_);
    }

    stereo_info_.header.stamp = raw_stereo->header.stamp;

    stereo_info_.height = stdata_->imHeight;
    stereo_info_.width = stdata_->imWidth;

    memcpy((char*)(&stereo_info_.T[0]),  (char*)(stdata_->T),   3*sizeof(double));
    memcpy((char*)(&stereo_info_.Om[0]), (char*)(stdata_->Om),  3*sizeof(double));
    memcpy((char*)(&stereo_info_.RP[0]), (char*)(stdata_->RP), 16*sizeof(double));

    stereo_info_pub_.publish(stereo_info_);
  }

  void publishImage(const ros::Publisher& pub, color_coding_t coding, int height,
                    int width, void* data, size_t dataSize)
  {
    if (coding == COLOR_CODING_NONE)
      return;

    // @todo: step calculation is a little hacky
    fillImage(img_, cam_bridge::ColorCodingToImageEncoding(coding),
              height, width, dataSize / height /*step*/, data);
    pub.publish(img_);
  }

  void publishImages(ImagePublishers& pubs, const stereo_msgs::RawStereo::ConstPtr& raw_stereo, cam::ImageData* img_data)
  {
    img_.header.stamp = raw_stereo->header.stamp;
    img_.header.frame_id = raw_stereo->header.frame_id;
    
    publishImage(pubs.image_raw_, img_data->imRawType, img_data->imHeight,
                 img_data->imWidth, img_data->imRaw, img_data->imRawSize);
    publishImage(pubs.image_, img_data->imType, img_data->imHeight,
                 img_data->imWidth, img_data->im, img_data->imSize);
    publishImage(pubs.image_color_, img_data->imColorType, img_data->imHeight,
                 img_data->imWidth, img_data->imColor, img_data->imColorSize);
    publishImage(pubs.image_rect_, img_data->imRectType, img_data->imHeight,
                 img_data->imWidth, img_data->imRect, img_data->imRectSize);
    publishImage(pubs.image_rect_color_, img_data->imRectColorType, img_data->imHeight,
                 img_data->imWidth, img_data->imRectColor, img_data->imRectColorSize);

    cam_info_.header.stamp = raw_stereo->header.stamp;
    cam_info_.header.frame_id = raw_stereo->header.frame_id;
    cam_info_.height = img_data->imHeight;
    cam_info_.width  = img_data->imWidth;

    memcpy((char*)(&cam_info_.D[0]), (char*)(img_data->D),  5*sizeof(double));
    memcpy((char*)(&cam_info_.K[0]), (char*)(img_data->K),  9*sizeof(double));
    memcpy((char*)(&cam_info_.R[0]), (char*)(img_data->R),  9*sizeof(double));
    memcpy((char*)(&cam_info_.P[0]), (char*)(img_data->P), 12*sizeof(double));

    pubs.cam_info_.publish(cam_info_);
  }


  void advertiseImages(ros::NodeHandle& nh, ImagePublishers& pubs, cam::ImageData* img_data)
  {
    pubs.cam_info_ = nh.advertise<sensor_msgs::CameraInfo>("cam_info", 1);

    if (img_data->imRawType != COLOR_CODING_NONE)
      pubs.image_raw_ = nh.advertise<sensor_msgs::Image>("image_raw", 1);

    if (img_data->imType != COLOR_CODING_NONE)
      pubs.image_ = nh.advertise<sensor_msgs::Image>("image", 1);

    if (img_data->imColorType != COLOR_CODING_NONE)
      pubs.image_color_ = nh.advertise<sensor_msgs::Image>("image_color", 1);

    if (img_data->imRectType != COLOR_CODING_NONE)
      pubs.image_rect_ = nh.advertise<sensor_msgs::Image>("image_rect", 1);

    if (img_data->imRectColorType != COLOR_CODING_NONE)
      pubs.image_rect_color_ = nh.advertise<sensor_msgs::Image>("image_rect_color", 1);

  }

  void advertiseCam()
  {
    stereo_info_pub_ = nh_.advertise<stereo_msgs::StereoInfo>("stereo_info", 1);

    advertiseImages(left_nh_, left_pubs_, stdata_->imLeft);
    advertiseImages(right_nh_, right_pubs_, stdata_->imRight);

    if (stdata_->hasDisparity)
    {
      disparity_info_pub_ = nh_.advertise<stereo_msgs::DisparityInfo>("disparity_info", 1);
      disparity_pub_ = nh_.advertise<sensor_msgs::Image>("disparity", 1);
    }

    if (do_calc_points_)
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("cloud",1);
  }

  void diagnosticUpdate(const ros::TimerEvent&)
  {
    diagnostic_.update();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereoproc");

  StereoProc sp;

  ros::spin();

  return 0;
}

