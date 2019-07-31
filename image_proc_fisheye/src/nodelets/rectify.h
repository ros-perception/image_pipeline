#include <nodelet/nodelet.h>
#include "ros/ros.h"
#include <image_transport/image_transport.h>


#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

namespace image_proc_fisheye
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
            cv::Mat mapx_;
            cv::Mat mapy_;
            bool camera_set_;
    };

}
