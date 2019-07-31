#include <nodelet/nodelet.h>
#include "ros/ros.h"
#include <image_transport/image_transport.h>


#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/cudacodec.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

namespace image_proc_tegra
{

    class RectifyNodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
            void process_image(const sensor_msgs::ImageConstPtr& frame);
            void camera_info(const sensor_msgs::CameraInfoConstPtr& info_msg);
        private:
            ros::Subscriber sub_;
            ros::Subscriber sub_info_;
            ros::Publisher pub_;
            cv::cuda::GpuMat mapx_;
            cv::cuda::GpuMat mapy_;
            bool camera_set_;
    };

}

namespace image_proc_tegra_fisheye
{

    class RectifyNodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
            void process_image(const sensor_msgs::ImageConstPtr& frame);
            void camera_info(const sensor_msgs::CameraInfoConstPtr& info_msg);
        private:
            ros::Subscriber sub_;
            ros::Subscriber sub_info_;
            ros::Publisher pub_;
            cv::cuda::GpuMat mapx_;
            cv::cuda::GpuMat mapy_;
            
            cv::Mat mx;
            cv::Mat my;
            
     
            bool camera_set_;
            
            bool img_downsample = false;
            int img_downsample_width = 800;
            int img_downsample_height = 503;
            
            
    };

}

