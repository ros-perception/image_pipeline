#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <boost/format.hpp>

sensor_msgs::CvBridge g_bridge;
int g_count = 0;
boost::format g_format;

void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info)
{
  if (g_bridge.fromImage(*image, "bgr8")) {
    IplImage *image = g_bridge.toIpl();
    if (image) {
      std::string filename = (g_format % g_count % "jpg").str();
      cvSaveImage(filename.c_str(), image);
      ROS_INFO("Saved image %s", filename.c_str());
      filename = (g_format % g_count % "ini").str();
      camera_calibration_parsers::writeCalibration(filename, "camera", *info);
      
      g_count++;
    } else {
      ROS_WARN("Couldn't save image, no data!");
    }
  }
  else
    ROS_ERROR("Unable to convert %s image to bgr8", image->encoding.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_saver", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  g_format.parse("left%04i.%s");
  image_transport::ImageTransport it(nh);
  std::string topic = nh.resolveName("image");
  image_transport::CameraSubscriber sub = it.subscribeCamera(topic, 1, &callback);

  ros::spin();
}
