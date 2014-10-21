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
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread.hpp>
#include <boost/format.hpp>

namespace image_view {

class ImageNodelet : public nodelet::Nodelet
{
  image_transport::Subscriber sub_;

  boost::mutex image_mutex_;
  sensor_msgs::ImageConstPtr last_msg_;
  cv::Mat last_image_;
  
  std::string window_name_;
  boost::format filename_format_;
  int count_;

  virtual void onInit();
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg);

  static void mouseCb(int event, int x, int y, int flags, void* param);

public:
  ImageNodelet();

  ~ImageNodelet();
};

ImageNodelet::ImageNodelet()
  : filename_format_(""), count_(0)
{
}

ImageNodelet::~ImageNodelet()
{
  cv::destroyWindow(window_name_);
}

void ImageNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle local_nh = getPrivateNodeHandle();

  // Command line argument parsing
  const std::vector<std::string>& argv = getMyArgv();
  // First positional argument is the transport type
  std::string transport = "raw";
  for (int i = 0; i < (int)argv.size(); ++i)
  {
    if (argv[i][0] != '-')
    {
      transport = argv[i];
      break;
    }
  }
  // Internal option, should be used only by the image_view node
  bool shutdown_on_close = std::find(argv.begin(), argv.end(),
                                     "--shutdown-on-close") != argv.end();

  // Default window name is the resolved topic name
  std::string topic = nh.resolveName("image");
  local_nh.param("window_name", window_name_, topic);

  bool autosize;
  local_nh.param("autosize", autosize, false);
  
  std::string format_string;
  local_nh.param("filename_format", format_string, std::string("frame%04i.jpg"));
  filename_format_.parse(format_string);

#if OPENCV3
  cv::namedWindow(window_name_, autosize ? cv::WND_PROP_AUTOSIZE : 0);
#else
  cv::namedWindow(window_name_, autosize ? CV_WINDOW_AUTOSIZE : 0);
#endif
  cv::setMouseCallback(window_name_, &ImageNodelet::mouseCb, this);

  image_transport::ImageTransport it(nh);
  image_transport::TransportHints hints(transport, ros::TransportHints(), getPrivateNodeHandle());
  sub_ = it.subscribe(topic, 1, &ImageNodelet::imageCb, this, hints);
}

void ImageNodelet::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  image_mutex_.lock();

  // May want to view raw bayer data, which CvBridge doesn't know about
  if (msg->encoding.find("bayer") != std::string::npos)
  {
    last_image_ = cv::Mat(msg->height, msg->width, CV_8UC1,
                          const_cast<uint8_t*>(&msg->data[0]), msg->step);
  }
  // We want to scale floating point images so that they display nicely
  else if(msg->encoding.find("F") != std::string::npos)
  {
    cv::Mat float_image_bridge = cv_bridge::toCvShare(msg, msg->encoding)->image;
    cv::Mat_<float> float_image = float_image_bridge;
    double max_val;
    cv::minMaxIdx(float_image, 0, &max_val);

    if(max_val > 0)
    {
      float_image /= max_val;
    }
    last_image_ = float_image;
  }
  else
  {
    // Convert to OpenCV native BGR color
    try {
      last_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e) {
      NODELET_ERROR_THROTTLE(30, "Unable to convert '%s' image to bgr8: '%s'",
                             msg->encoding.c_str(), e.what());
    }
  }

  // last_image_ may point to data owned by last_msg_, so we hang onto it for
  // the sake of mouseCb.
  last_msg_ = msg;

  // Must release the mutex before calling cv::imshow, or can deadlock against
  // OpenCV's window mutex.
  image_mutex_.unlock();
  if (!last_image_.empty()) {
    cv::imshow(window_name_, last_image_);
    cv::waitKey(1);
  }
}

void ImageNodelet::mouseCb(int event, int x, int y, int flags, void* param)
{
  ImageNodelet *this_ = reinterpret_cast<ImageNodelet*>(param);
  // Trick to use NODELET_* logging macros in static function
  boost::function<const std::string&()> getName =
    boost::bind(&ImageNodelet::getName, this_);

#if OPENCV3
  if (event == cv::EVENT_LBUTTONDOWN)
#else
  if (event == CV_EVENT_LBUTTONDOWN)
#endif
  {
    NODELET_WARN_ONCE("Left-clicking no longer saves images. Right-click instead.");
    return;
  }
#if OPENCV3
  if (event != cv::EVENT_RBUTTONDOWN)
#else
  if (event != CV_EVENT_RBUTTONDOWN)
#endif
    return;
  
  boost::lock_guard<boost::mutex> guard(this_->image_mutex_);

  const cv::Mat image = this_->last_image_;
  if (image.empty())
  {
    NODELET_WARN("Couldn't save image, no data!");
    return;
  }

  std::string filename = (this_->filename_format_ % this_->count_).str();
  if (cv::imwrite(filename, image))
  {
    NODELET_INFO("Saved image %s", filename.c_str());
    this_->count_++;
  }
  else
  {
    /// @todo Show full path, ask if user has permission to write there
    NODELET_ERROR("Failed to save image.");
  }
}

} // namespace image_view

// Register the nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( image_view::ImageNodelet, nodelet::Nodelet)
