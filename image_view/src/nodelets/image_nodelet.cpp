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
#include "window_thread.h"

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>


namespace image_view {

class ThreadSafeImage
{
  boost::mutex mutex_;
  boost::condition_variable condition_;
  cv::Mat image_;

public:
  void set(const cv::Mat& image);

  cv::Mat get();

  cv::Mat pop();
};

void ThreadSafeImage::set(const cv::Mat& image)
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  image_ = image;
  condition_.notify_one();
}

cv::Mat ThreadSafeImage::get()
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  return image_;
}

cv::Mat ThreadSafeImage::pop()
{
  cv::Mat image;
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    while (image_.empty())
    {
      condition_.wait(lock);
    }
    image = image_;
    image_.release();
  }
  return image;
}

class ImageNodelet : public nodelet::Nodelet
{
  image_transport::Subscriber sub_;

  boost::thread window_thread_;

  ThreadSafeImage queued_image_, shown_image_;
  
  std::string window_name_;
  bool autosize_;
  boost::format filename_format_;
  int count_;
  
  ros::WallTimer gui_timer_;

  virtual void onInit();
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg);

  static void mouseCb(int event, int x, int y, int flags, void* param);
  static void guiCb(const ros::WallTimerEvent&);

  void windowThread();  

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
  if (window_thread_.joinable())
  {
    window_thread_.interrupt();
    window_thread_.join();
  }
}

void ImageNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle local_nh = getPrivateNodeHandle();

  // Command line argument parsing
  const std::vector<std::string>& argv = getMyArgv();
  // First positional argument is the transport type
  std::string transport;
  local_nh.param("image_transport", transport, std::string("raw"));
  for (int i = 0; i < (int)argv.size(); ++i)
  {
    if (argv[i][0] != '-')
    {
      transport = argv[i];
      break;
    }
  }
  NODELET_INFO_STREAM("Using transport \"" << transport << "\"");
  // Internal option, should be used only by the image_view node
  bool shutdown_on_close = std::find(argv.begin(), argv.end(),
                                     "--shutdown-on-close") != argv.end();

  // Default window name is the resolved topic name
  std::string topic = nh.resolveName("image");
  local_nh.param("window_name", window_name_, topic);

  local_nh.param("autosize", autosize_, false);
  
  std::string format_string;
  local_nh.param("filename_format", format_string, std::string("frame%04i.jpg"));
  filename_format_.parse(format_string);

  // Since cv::startWindowThread() triggers a crash in cv::waitKey()
  // if OpenCV is compiled against GTK, we call cv::waitKey() from
  // the ROS event loop periodically, instead.
  /*cv::startWindowThread();*/
  gui_timer_ = local_nh.createWallTimer(ros::WallDuration(0.1), ImageNodelet::guiCb);

  window_thread_ = boost::thread(&ImageNodelet::windowThread, this);

  image_transport::ImageTransport it(nh);
  image_transport::TransportHints hints(transport, ros::TransportHints(), getPrivateNodeHandle());
  sub_ = it.subscribe(topic, 1, &ImageNodelet::imageCb, this, hints);
}

void ImageNodelet::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  // We want to scale floating point images so that they display nicely
  bool do_dynamic_scaling = (msg->encoding.find("F") != std::string::npos);

  // Convert to OpenCV native BGR color
  try {
    cv_bridge::CvtColorForDisplayOptions options;
    options.do_dynamic_scaling = do_dynamic_scaling;
    queued_image_.set(cvtColorForDisplay(cv_bridge::toCvShare(msg), "", options)->image);
  }
  catch (cv_bridge::Exception& e) {
    NODELET_ERROR_THROTTLE(30, "Unable to convert '%s' image for display: '%s'",
                             msg->encoding.c_str(), e.what());
  }
}

void ImageNodelet::guiCb(const ros::WallTimerEvent&)
{
  // Process pending GUI events and return immediately
  cv::waitKey(1);
}

void ImageNodelet::mouseCb(int event, int x, int y, int flags, void* param)
{
  ImageNodelet *this_ = reinterpret_cast<ImageNodelet*>(param);
  // Trick to use NODELET_* logging macros in static function
  boost::function<const std::string&()> getName =
    boost::bind(&ImageNodelet::getName, this_);

  if (event == cv::EVENT_LBUTTONDOWN)
  {
    NODELET_WARN_ONCE("Left-clicking no longer saves images. Right-click instead.");
    return;
  }
  if (event != cv::EVENT_RBUTTONDOWN)
    return;
  
  cv::Mat image(this_->shown_image_.get());
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

void ImageNodelet::windowThread()
{
  cv::namedWindow(window_name_, autosize_ ? cv::WND_PROP_AUTOSIZE : 0);
  cv::setMouseCallback(window_name_, &ImageNodelet::mouseCb, this);

  try
  {
    while (true)
    {
      cv::Mat image(queued_image_.pop());
      cv::imshow(window_name_, image);
      cv::waitKey(1);
      shown_image_.set(image);
    }
  }
  catch (const boost::thread_interrupted&)
  {
  }

  cv::destroyWindow(window_name_);
}

} // namespace image_view

// Register the nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( image_view::ImageNodelet, nodelet::Nodelet)
