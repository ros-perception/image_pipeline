#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <boost/format.hpp>

int g_count = 0;
boost::format g_format;

void callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info)
{
  cv::Mat image;
  try
  {
    image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
  } catch(cv_bridge::Exception)
  {
    ROS_ERROR("Unable to convert %s image to bgr8", image_msg->encoding.c_str());
    return;
  }
  
    if (!image.empty()) {
      std::string filename = (g_format % g_count % "jpg").str();
      cv::imwrite(filename, image);
      ROS_INFO("Saved image %s", filename.c_str());
      filename = (g_format % g_count % "ini").str();
      camera_calibration_parsers::writeCalibration(filename, "camera", *info);
      
      g_count++;
    } else {
      ROS_WARN("Couldn't save image, no data!");
    }
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
