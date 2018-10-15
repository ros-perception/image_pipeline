#include "ros/ros.h"
#include "rectify.h"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_proc_fisheye::RectifyNodelet, nodelet::Nodelet)


namespace image_proc_fisheye {
  void RectifyNodelet::onInit(){
    ros::NodeHandle &nh = getNodeHandle();
    camera_set_=false;
    sub_ = nh.subscribe("image_raw", 5, &RectifyNodelet::process_image, this);
    sub_info_ = nh.subscribe("camera_info", 5, &RectifyNodelet::camera_info, this);
    pub_ = nh.advertise<sensor_msgs::Image>("image_rect", 10);
  };

  void RectifyNodelet::process_image(const sensor_msgs::ImageConstPtr& frame){
    if(!camera_set_){
      return;
    }
    cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(frame);
    cv::Mat image_gpu(image->image);
    cv::Mat image_gpu_rect(cv::Size(image->image.rows, image->image.cols), image->image.type());
    cv::remap(image_gpu, image_gpu_rect, mapx_, mapy_, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT);
    cv::Mat image_rect = cv::Mat(image_gpu_rect);
	
    cv_bridge::CvImage out_msg;
    out_msg.header   = frame->header;
    out_msg.encoding = frame->encoding;
    out_msg.image  = image_rect;
    pub_.publish(out_msg.toImageMsg());
  }

  void RectifyNodelet::camera_info(const sensor_msgs::CameraInfoConstPtr& info_msg){
    image_geometry::PinholeCameraModel camera;
    camera.fromCameraInfo(info_msg);
    cv::Mat m1;
    cv::Mat m2;
    cv::fisheye::initUndistortRectifyMap(camera.intrinsicMatrix(), camera.distortionCoeffs(), cv::Mat(), camera.intrinsicMatrix(), camera.fullResolution(), CV_32FC1, m1, m2);
    mapx_ = cv::Mat(m1);
    mapy_ = cv::Mat(m2);
    sub_info_.shutdown();
    camera_set_ = true;
  }
} // namespace

