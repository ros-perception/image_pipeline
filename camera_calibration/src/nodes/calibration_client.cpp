//license

#include "camera_calibration/calibrate.h"
#include "camera_calibration/CalibrationPattern.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>
#include <topic_synchronizer2/topic_synchronizer.h>

#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv_latest/CvBridge.h>

static const char wnd_name[] = "calibration";

class CalibrationClient {
public:
  ros::NodeHandle n;
  
  sensor_msgs::CvBridge bridge;
  TopicSynchronizer sync;
  ros::Subscriber pattern_sub;
  ros::Subscriber image_sub;

  boost::mutex sync_mutex;
  camera_calibration::CalibrationPatternConstPtr pattern;
  sensor_msgs::ImageConstPtr image;

  camera_calibration::Calibrater calibrater;

  cv::WImageBuffer3_b display_image;
  cv::WImageView3_b raw_image;
  cv::WImageView3_b rect_image;
  cv::WImageView3_b pattern_image;

  CalibrationClient(ros::NodeHandle handle)
    : n(handle),
      sync(&CalibrationClient::syncCB, this),
      pattern_sub( n.subscribe("~calibration", 1, sync.synchronize(&CalibrationClient::calibrationCB, this)) ),
      image_sub( n.subscribe("image", 1, sync.synchronize(&CalibrationClient::imageCB, this)) )
  {
    cvNamedWindow(wnd_name, 0);
  }

  ~CalibrationClient()
  {
    cvDestroyWindow(wnd_name);
  }

  void syncCB()
  {
    ROS_INFO("syncCB called");
    boost::lock_guard<boost::mutex> guard(sync_mutex);

    cvShowImage(wnd_name, display_image.Ipl());
  }

  void calibrationCB(const camera_calibration::CalibrationPatternConstPtr& msg)
  {
    boost::lock_guard<boost::mutex> guard(sync_mutex);
    pattern = msg;
  }

  void imageCB(const sensor_msgs::ImageConstPtr& msg)
  {
    boost::lock_guard<boost::mutex> guard(sync_mutex);
    image = msg;

    if (!bridge.fromImage(*image, "bgr8")) {
      ROS_WARN("Unable to convert from image to bgr");
      return;
    }

    // Set up display images
    cv::WImageView3_b raw( bridge.toIpl() );
    int w = raw.Width(), h = raw.Height();
    if (display_image.Width() != w || display_image.Height() != h) {
      display_image.Allocate(w*3, h);
      raw_image = display_image.View(0, 0, w, h);
      rect_image = display_image.View(w, 0, w, h);
      pattern_image = display_image.View(w*2, 0, w, h);
    }

    raw_image.CopyFrom(raw);
    rect_image.CopyFrom(raw);
    pattern_image.CopyFrom(raw);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration_server");
  ros::NodeHandle n;
  CalibrationClient calib_client(n);
  ros::ServiceClient request_client = n.serviceClient<std_srvs::Empty>("~request");

  while (n.ok()) {
    int k = cvWaitKey(10);
    if (k == 'q')
      break;
    ros::spinOnce();
  }

  return 0;
}
