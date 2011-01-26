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

#include <opencv2/highgui/highgui.hpp>

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

#ifdef HAVE_GTK
#include <gtk/gtk.h>

// Platform-specific workaround for #3026: image_view doesn't close when
// closing image window. On platforms using GTK+ we connect this to the
// window's "destroy" event so that image_view exits.
static void destroy(GtkWidget *widget, gpointer data)
{
  ros::shutdown();
}
#endif

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

void increment(int* value)
{
  ++(*value);
}

// Note: StereoView is NOT nodelet-based, as it synchronizes the three streams.
class StereoView
{
private:
  image_transport::SubscriberFilter left_sub_, right_sub_;
  message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image,
                                    stereo_msgs::DisparityImage> sync_;
  
  sensor_msgs::ImageConstPtr last_left_msg_, last_right_msg_;
  cv::Mat last_left_image_, last_right_image_;
  sensor_msgs::CvBridge left_bridge_, right_bridge_;
  cv::Mat_<cv::Vec3b> disparity_color_;
  boost::mutex image_mutex_;
  
  boost::format filename_format_;
  int save_count_;

  ros::WallTimer check_synced_timer_;
  int left_received_, right_received_, disp_received_, all_received_;

public:
  StereoView()
    : sync_(3), filename_format_(""), save_count_(0),
      left_received_(0), right_received_(0), disp_received_(0), all_received_(0)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    bool autosize;
    local_nh.param("autosize", autosize, true);
    
    std::string format_string;
    local_nh.param("filename_format", format_string, std::string("%s%04i.pgm"));
    filename_format_.parse(format_string);

    // Do GUI window setup
    cv::namedWindow("left", autosize ? CV_WINDOW_AUTOSIZE : 0);
    cv::namedWindow("right", autosize ? CV_WINDOW_AUTOSIZE : 0);
    cv::namedWindow("disparity", autosize ? CV_WINDOW_AUTOSIZE : 0);
    cvSetMouseCallback("left",  &StereoView::mouseCb, this);
    cvSetMouseCallback("right", &StereoView::mouseCb, this);
#ifdef HAVE_GTK
    g_signal_connect(GTK_WIDGET( cvGetWindowHandle("left") ),
                     "destroy", G_CALLBACK(destroy), NULL);
    g_signal_connect(GTK_WIDGET( cvGetWindowHandle("right") ),
                     "destroy", G_CALLBACK(destroy), NULL);
    g_signal_connect(GTK_WIDGET( cvGetWindowHandle("disparity") ),
                     "destroy", G_CALLBACK(destroy), NULL);
#endif
    cvStartWindowThread();

    // Resolve topic names
    ros::NodeHandle nh;
    std::string stereo_ns = nh.resolveName("stereo");
    std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh.resolveName("image"));
    std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh.resolveName("image"));
    std::string disparity_topic = ros::names::clean(stereo_ns + "/disparity");
    ROS_INFO("Subscribing to:\n\t* %s\n\t* %s\n\t* %s", left_topic.c_str(), right_topic.c_str(),
             disparity_topic.c_str());

    // Subscribe to synchronized topics
    image_transport::ImageTransport it(nh);
    left_sub_.subscribe(it, left_topic, 3);
    right_sub_.subscribe(it, right_topic, 3);
    disparity_sub_.subscribe(nh, disparity_topic, 3);
    sync_.connectInput(left_sub_, right_sub_, disparity_sub_);
    sync_.registerCallback(boost::bind(&StereoView::imageCb, this, _1, _2, _3));

    // Complain every 30s if the topics appear unsynchronized
    left_sub_.registerCallback(boost::bind(increment, &left_received_));
    right_sub_.registerCallback(boost::bind(increment, &right_received_));
    disparity_sub_.registerCallback(boost::bind(increment, &disp_received_));
    sync_.registerCallback(boost::bind(increment, &all_received_));
    check_synced_timer_ = nh.createWallTimer(ros::WallDuration(30.0),
                                             boost::bind(&StereoView::checkInputsSynchronized, this));
  }

  ~StereoView()
  {
    cvDestroyAllWindows();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right,
               const stereo_msgs::DisparityImageConstPtr& disparity_msg)
  {
    image_mutex_.lock();

    // May want to view raw bayer data
    if (left->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<sensor_msgs::Image>(left)->encoding = "mono8";
    if (right->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<sensor_msgs::Image>(right)->encoding = "mono8";

    // Hang on to image data for sake of mouseCb
    last_left_msg_ = left;
    last_right_msg_ = right;
    try {
      last_left_image_ = left_bridge_.imgMsgToCv(left, "bgr8");
      last_right_image_ = right_bridge_.imgMsgToCv(right, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException& e) {
      ROS_ERROR("Unable to convert one of '%s' or '%s' to 'bgr8'",
                left->encoding.c_str(), right->encoding.c_str());
    }

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

    // Must release the mutex before calling cv::imshow, or can deadlock against
    // OpenCV's window mutex.
    image_mutex_.unlock();
    if (!last_left_image_.empty())
      cv::imshow("left", last_left_image_);
    if (!last_right_image_.empty())
      cv::imshow("right", last_right_image_);
    cv::imshow("disparity", disparity_color_);
  }

  void saveImage(const char* prefix, const cv::Mat& image)
  {
    if (!image.empty()) {
      std::string filename = (filename_format_ % prefix % save_count_).str();
      cv::imwrite(filename, image);
      ROS_INFO("Saved image %s", filename.c_str());
    } else {
      ROS_WARN("Couldn't save %s image, no data!", prefix);
    }
  }
  
  static void mouseCb(int event, int x, int y, int flags, void* param)
  {
    if (event != CV_EVENT_LBUTTONDOWN)
      return;
    
    StereoView *sv = (StereoView*)param;
    boost::lock_guard<boost::mutex> guard(sv->image_mutex_);

    sv->saveImage("left",  sv->last_left_image_);
    sv->saveImage("right", sv->last_right_image_);
    sv->save_count_++;
  }

  void checkInputsSynchronized()
  {
    int threshold = 3 * all_received_;
    if (left_received_ > threshold || right_received_ > threshold || disp_received_ > threshold) {
      ROS_WARN("[stereo_view] Low number of synchronized left/right/disparity triplets received.\n"
               "Left images received: %d\n"
               "Right images received: %d\n"
               "Disparity images received: %d\n"
               "Synchronized triplets: %d\n"
               "Possible issues:\n"
               "\t* stereo_image_proc is not running.\n"
               "\t* The cameras are not synchronized.\n"
               "\t* The network is too slow. One or more images are dropped from each triplet.",
               left_received_, right_received_, disp_received_, all_received_);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_view", ros::init_options::AnonymousName);
  if (ros::names::remap("stereo") == "stereo") {
    ROS_WARN("'stereo' has not been remapped! Example command-line usage:\n"
             "\t$ rosrun image_view stereo_view stereo:=narrow_stereo image:=image_color");
  }
  if (ros::names::remap("image") == "/image_raw") {
    ROS_WARN("There is a delay between when the camera drivers publish the raw images and "
             "when stereo_image_proc publishes the computed point cloud. stereo_view "
             "may fail to synchronize these topics.");
  }
  
  StereoView view;
  
  ros::spin();
  return 0;
}
