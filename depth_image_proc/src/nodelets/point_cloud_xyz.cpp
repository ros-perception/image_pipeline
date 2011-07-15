#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

namespace depth_image_proc {

namespace enc = sensor_msgs::image_encodings;

class PointCloudXyzNodelet : public nodelet::Nodelet
{
  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_depth_;
  bool subscribed_;

  // Publications
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  ros::Publisher pub_point_cloud_;

  image_geometry::PinholeCameraModel model_;

  virtual void onInit();

  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
};

void PointCloudXyzNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Monitor whether anyone is subscribed to the output
  subscribed_ = false;
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloudXyzNodelet::connectCb, this);
  pub_point_cloud_ = nh.advertise<PointCloud>("points", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void PointCloudXyzNodelet::connectCb()
{
  if (pub_point_cloud_.getNumSubscribers() == 0)
  {
    sub_depth_.shutdown();
    subscribed_ = false;
  }
  else if (!subscribed_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_depth_ = it_->subscribeCamera("image_rect_raw", 1, &PointCloudXyzNodelet::depthCb, this, hints);
    subscribed_ = true;
  }
}

void PointCloudXyzNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                   const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  /// @todo Support float (metric) depth images also?
  if (depth_msg->encoding != enc::TYPE_16UC1)
  {
    NODELET_ERROR("Expected data of type [%s], got [%s]", enc::TYPE_16UC1.c_str(),
                  depth_msg->encoding.c_str());
    return;
  }
  
  PointCloud::Ptr cloud_msg (new PointCloud);
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width  = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->points.resize (cloud_msg->height * cloud_msg->width);

  // Use correct principal point from calibration
  model_.fromCameraInfo(info_msg);
  float center_x = model_.cx();
  float center_y = model_.cy();

  // Pre-compute constants for focal length and m->mm conversion
  float constant_x = 0.001 / model_.fx();
  float constant_y = 0.001 / model_.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  
  int depth_idx = 0;
  pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud_msg->begin ();
  const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);

  for (int v = 0; v < (int)cloud_msg->height; ++v)
  {
    for (int u = 0; u < (int)cloud_msg->width; ++u, ++depth_idx, ++pt_iter)
    {
      pcl::PointXYZ& pt = *pt_iter;

      // Check for invalid measurements
      if (depth_data[depth_idx] == 0)
      {
        // not valid
        pt.x = pt.y = pt.z = bad_point;
        continue;
      }

      // Fill in XYZ
      pt.x = (u - center_x) * depth_data[depth_idx] * constant_x;
      pt.y = (v - center_y) * depth_data[depth_idx] * constant_y;
      pt.z = depth_data[depth_idx] * 0.001;
    }
  }

  pub_point_cloud_.publish (cloud_msg);
}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (depth_image_proc, point_cloud_xyz, depth_image_proc::PointCloudXyzNodelet, nodelet::Nodelet);
