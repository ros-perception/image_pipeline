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

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/CvBridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>

namespace enc = sensor_msgs::image_encodings;

// colormap for disparities, RGB
static unsigned char colormap[768] = 
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

class StereoView
{
private:
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter left_sub_, right_sub_;
  message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image,
                                    stereo_msgs::DisparityImage> sync_;
  
  sensor_msgs::ImageConstPtr last_left_, last_right_;
  sensor_msgs::CvBridge left_bridge_, right_bridge_;
  cv::Mat_<cv::Vec3b> disparity_color_;
  boost::mutex image_mutex_;
  
  boost::format filename_format_;
  int count_;

public:
  StereoView(ros::NodeHandle& nh)
    : it_(nh), sync_(3), filename_format_(""), count_(0)
  {
    ros::NodeHandle local_nh("~");
    bool autosize;
    local_nh.param("autosize", autosize, true);
    
    std::string format_string;
    local_nh.param("filename_format", format_string, std::string("%s%04i.pgm"));
    filename_format_.parse(format_string);
    
    cvNamedWindow("left", autosize ? CV_WINDOW_AUTOSIZE : 0);
    cvNamedWindow("right", autosize ? CV_WINDOW_AUTOSIZE : 0);
    cv::namedWindow("disparity", autosize ? CV_WINDOW_AUTOSIZE : 0);
    cvSetMouseCallback("left",  &StereoView::mouseCB, this);
    cvSetMouseCallback("right", &StereoView::mouseCB, this);
    cvStartWindowThread();

    std::string stereo_ns = nh.resolveName("stereo");
    std::string left_topic = stereo_ns + "/left/" + nh.resolveName("image");
    std::string right_topic = stereo_ns + "/right/" + nh.resolveName("image");
    std::string disparity_topic = stereo_ns + "/disparity";
    left_sub_.subscribe(it_, left_topic, 3);
    right_sub_.subscribe(it_, right_topic, 3);
    disparity_sub_.subscribe(nh, disparity_topic, 3);
    sync_.connectInput(left_sub_, right_sub_, disparity_sub_);
    sync_.registerCallback(boost::bind(&StereoView::imageCB, this, _1, _2, _3));
  }

  ~StereoView()
  {
    cvDestroyWindow("left");
    cvDestroyWindow("right");
  }

  void showImage(const char* window, const sensor_msgs::Image& img,
                 sensor_msgs::CvBridge& bridge)
  {
    // May want to view raw bayer data
    if (img.encoding.find("bayer") != std::string::npos)
      const_cast<sensor_msgs::Image&>(img).encoding = enc::MONO8;

    if (img.encoding == enc::MONO8 && bridge.fromImage(img, enc::MONO8))
      cvShowImage(window, bridge.toIpl());
    else if (bridge.fromImage(img, enc::BGR8))
      cvShowImage(window, bridge.toIpl());
    else
      ROS_ERROR("Unable to convert %s image to bgr8", img.encoding.c_str());
  }

  void imageCB(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right,
               const stereo_msgs::DisparityImageConstPtr& disparity_msg)
  {
    {
      boost::lock_guard<boost::mutex> guard(image_mutex_);
    
      // Hang on to message pointers for sake of mouse_cb
      last_left_ = left;
      last_right_ = right;
    }

    showImage("left", *left, left_bridge_);
    showImage("right", *right, right_bridge_);

    // Colormap and display the disparity image
    float min_disparity = disparity_msg->min_disparity;
    float max_disparity = disparity_msg->max_disparity;
    float multiplier = 255.0f / (max_disparity - min_disparity);

    assert(disparity_msg->image.encoding == enc::TYPE_32FC1);
    const cv::Mat_<float> dmat(disparity_msg->image.height, disparity_msg->image.width,
                               (float*)&disparity_msg->image.data[0], disparity_msg->image.step);
    disparity_color_.create(disparity_msg->image.height, disparity_msg->image.width);
    
    for (int row = 0; row < disparity_color_.rows; ++row) {
      const float* d = dmat[row];
      for (int col = 0; col < disparity_color_.cols; ++col) {
        int index = (d[col] - min_disparity) * multiplier + 0.5;
        index = std::min(255, std::max(0, index));
        // Fill as BGR
        disparity_color_(row, col)[2] = colormap[3*index + 0];
        disparity_color_(row, col)[1] = colormap[3*index + 1];
        disparity_color_(row, col)[0] = colormap[3*index + 2];
      }
    }

    cv::imshow("disparity", disparity_color_);
  }

  void saveImage(const char* prefix, const IplImage* image)
  {
    if (image) {
      std::string filename = (filename_format_ % prefix % count_).str();
      cvSaveImage(filename.c_str(), image);
      ROS_INFO("Saved image %s", filename.c_str());
    } else {
      ROS_WARN("Couldn't save image, no data!");
    }
  }
  
  static void mouseCB(int event, int x, int y, int flags, void* param)
  {
    if (event != CV_EVENT_LBUTTONDOWN)
      return;
    
    StereoView *sv = (StereoView*)param;
    boost::lock_guard<boost::mutex> guard(sv->image_mutex_);

    sv->saveImage("left", sv->left_bridge_.toIpl());
    sv->saveImage("right", sv->right_bridge_.toIpl());
    sv->count_++;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_view", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  if (nh.resolveName("stereo") == "/stereo") {
    ROS_WARN("stereo_view: stereo has not been remapped! Example command-line usage:\n"
             "\t$ rosrun image_view stereo_view stereo:=narrow_stereo image:=image_color");
  }
  
  StereoView view(nh);
  
  ros::spin();
  return 0;
}
