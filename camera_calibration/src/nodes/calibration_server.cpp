// license

#include "camera_calibration/calibrate.h"
#include "camera_calibration/CalibrationPattern.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <std_srvs/Empty.h>
#include <boost/thread.hpp>

class CalibrationServer {
public:
  ros::NodeHandle n;
  
  sensor_msgs::CvBridge bridge;
  camera_calibration::CheckerboardDetector detector;
  ros::Publisher pattern_pub;
  ros::Subscriber raw_sub;
  ros::ServiceServer request_srv;

  bool request_made;
  boost::mutex request_mutex;

  CalibrationServer(ros::NodeHandle handle)
    : n(handle),
      pattern_pub( n.advertise<camera_calibration::CalibrationPattern>("~calibration", 1) ),
      raw_sub( n.subscribe("image", 1, &CalibrationServer::image_cb, this) ),
      request_srv( n.advertiseService("~request", &CalibrationServer::request_view, this) ),
      request_made(false)
  {
    int board_width, board_height;
    double square_size;
    if (!n.getParam("~board_width", board_width) ||
        !n.getParam("~board_height", board_height) ||
        !n.getParam("~square_size", square_size))
    {
      ROS_FATAL("Board not fully specified");
      n.shutdown();
      return;
    }
    detector.setDimensions(board_width, board_height, square_size);

    int search_radius;
    if (n.getParam("~search_radius", search_radius))
      detector.setSearchRadius(search_radius);

    /** @todo: set detection flags */
  }

  bool request_view(std_srvs::Empty::Request& req,
                    std_srvs::Empty::Response& rsp)
  {
    ROS_INFO("View requested");
    request_mutex.lock();
    request_made = true;
    request_mutex.unlock();
    return true;
  }

  void image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    camera_calibration::CalibrationPattern pattern_msg;
    pattern_msg.header = msg->header;
    
    boost::lock_guard<boost::mutex> guard(request_mutex);
    if (!request_made) {
      pattern_pub.publish(pattern_msg);
      return;
    }

    if (!bridge.fromImage(*msg, "mono8")) {
      ROS_WARN("Unable to convert image");
      pattern_pub.publish(pattern_msg);
      return;
    }

    pattern_msg.image_points.resize(detector.corners());
    CvPoint2D32f* corners = (CvPoint2D32f*)&pattern_msg.image_points[0];
    int ncorners = 0;
    bool success = detector.findCorners(bridge.toIpl(), corners, &ncorners);
    if (success) {
      pattern_msg.object_points.resize(detector.corners());
      memcpy(&pattern_msg.object_points[0], detector.objectPoints(),
             detector.corners() * sizeof(CvPoint3D32f));
    } else {
      pattern_msg.image_points.resize(ncorners);
    }

    pattern_pub.publish(pattern_msg);
    request_made = false;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration_server");
  ros::NodeHandle n;
  CalibrationServer server(n);

  ros::spin();

  return 0;
}
