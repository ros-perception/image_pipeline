#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv_latest/CvBridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher left_pub = it.advertise("narrow_stereo/left/image", 1);
  image_transport::Publisher right_pub = it.advertise("narrow_stereo/right/image", 1);

  //cv::WImageBuffer3_b image( cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR) );
  cv::WImageBuffer1_b image( cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE) );
  sensor_msgs::Image msg;
  sensor_msgs::CvBridge::fromIpltoRosImage(image.Ipl(), msg);
  //msg.encoding = "bgr";
  msg.header.frame_id = "base_link";
  
  ros::Rate loop_rate(5);
  while (nh.ok()) {
    ros::Time time = ros::Time::now();
    msg.header.stamp = time;
    left_pub.publish(msg);
    right_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
