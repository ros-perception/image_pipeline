#include "ros/ros.h"
#include "rectify.h"

#include <cuda_runtime_api.h>
#include <cuda.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_proc_tegra::RectifyNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(image_proc_tegra_fisheye::RectifyNodelet, nodelet::Nodelet)


namespace image_proc_tegra {
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
    cv::cuda::GpuMat image_gpu(image->image);
    cv::cuda::GpuMat image_gpu_rect(cv::Size(image->image.rows, image->image.cols), image->image.type());
    cv::cuda::remap(image_gpu, image_gpu_rect, mapx_, mapy_, cv::INTER_CUBIC, cv::BORDER_CONSTANT);
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
    cv::initUndistortRectifyMap(camera.intrinsicMatrix(), camera.distortionCoeffs(), cv::Mat(), camera.intrinsicMatrix(), camera.fullResolution(), CV_32FC1, m1, m2);
    mapx_ = cv::cuda::GpuMat(m1);
    mapy_ = cv::cuda::GpuMat(m2);
    sub_info_.shutdown();
    camera_set_ = true;
  }
} // namespace

namespace image_proc_tegra_fisheye {
  void RectifyNodelet::onInit(){
    ros::NodeHandle &nh = getNodeHandle();
    camera_set_ = false;
    
    // ROS params
    ros::param::get(ros::this_node::getName()+"/img_downsample", img_downsample);
    ros::param::get(ros::this_node::getName()+"/img_downsample_width", img_downsample_width);
    ros::param::get(ros::this_node::getName()+"/img_downsample_height", img_downsample_height);
    
    sub_ = nh.subscribe("image_raw", 5, &RectifyNodelet::process_image, this);
    sub_info_ = nh.subscribe("camera_info", 5, &RectifyNodelet::camera_info, this);
    pub_ = nh.advertise<sensor_msgs::Image>("image_rect", 10);
  };
  
  void RectifyNodelet::process_image(const sensor_msgs::ImageConstPtr& frame){
    if(!camera_set_){
      return;
    }
    cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(frame);
    cv::cuda::GpuMat image_gpu, image_gpu_rect ;
    
    // upload to GPU
    image_gpu.upload( image->image );
    
    
    //cv::cuda::remap(image_gpu, image_gpu_rect, mapx_, mapy_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::cuda::remap(image_gpu, image_gpu_rect, mapx_, mapy_, cv::INTER_CUBIC, cv::BORDER_CONSTANT);
    
    // Resize? maybe to increase resolution
    if(img_downsample){
        cv::cuda::resize(image_gpu_rect , image_gpu_rect, cv::Size(img_downsample_width, img_downsample_height) , 0.0, 0.0, cv::INTER_CUBIC);
    }
    cv::Mat image_rect;
    image_gpu_rect.download( image_rect );
    
    // CPU only
    //cv::Mat image_rect( image->image );
    //cv::remap(image_rect, image_rect, mx, my, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT);
    //cv::resize(image_rect , image_rect, cv::Size(800, 503) , 0.0, 0.0, cv::INTER_LANCZOS4);
    
    cv_bridge::CvImage out_msg;
    out_msg.header   = frame->header;
    out_msg.encoding = frame->encoding;
    out_msg.image  = image_rect;
    pub_.publish(out_msg.toImageMsg());
	
	// Deallocate GPU memory:
	//cudaFree(image_gpu.data);
	//cudaFree(image_gpu_rect.data);
  }

  void RectifyNodelet::camera_info(const sensor_msgs::CameraInfoConstPtr& info_msg){
    image_geometry::PinholeCameraModel camera;
    
    camera.fromCameraInfo(info_msg);
    cv::Mat m1; 
    cv::Mat m2;
    
    //cv::fisheye::initUndistortRectifyMap(camera.intrinsicMatrix(), camera.distortionCoeffs(), cv::Mat(), camera.intrinsicMatrix(), camera.fullResolution(), CV_32FC1, m1, m2); 

    // Try  with rotation matrix for proper alignment of Stereo pairs
    cv::fisheye::initUndistortRectifyMap(camera.intrinsicMatrix(), camera.distortionCoeffs(), camera.rotationMatrix(), camera.intrinsicMatrix() , camera.fullResolution(), CV_32FC1, m1, m2);
    
    mapx_ = cv::cuda::GpuMat(m1);
    mapy_ = cv::cuda::GpuMat(m2);
    
    sub_info_.shutdown();
    camera_set_ = true;
  }
} // namespace
