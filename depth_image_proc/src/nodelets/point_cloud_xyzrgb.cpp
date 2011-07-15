#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

namespace depth_image_proc {

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

class PointCloudXyzrgbNodelet : public nodelet::Nodelet
{
  ros::NodeHandlePtr rgb_nh_;
  boost::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;
  
  // Subscriptions
  image_transport::SubscriberFilter sub_depth_, sub_rgb_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
  typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  boost::shared_ptr<Synchronizer> sync_;
  bool subscribed_;

  // Publications
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
  ros::Publisher pub_point_cloud_;

  image_geometry::PinholeCameraModel model_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::ImageConstPtr& rgb_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
};

void PointCloudXyzrgbNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  rgb_nh_.reset( new ros::NodeHandle(nh, "rgb") );
  ros::NodeHandle depth_nh(nh, "depth_registered");
  rgb_it_  .reset( new image_transport::ImageTransport(*rgb_nh_) );
  depth_it_.reset( new image_transport::ImageTransport(depth_nh) );

  // Read parameters
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);

  // Monitor whether anyone is subscribed to the output
  subscribed_ = false;
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloudXyzrgbNodelet::connectCb, this);
  pub_point_cloud_ = depth_nh.advertise<PointCloud>("points", 1, connect_cb, connect_cb);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_) );
  sync_->registerCallback(boost::bind(&PointCloudXyzrgbNodelet::imageCb, this, _1, _2, _3));
}

// Handles (un)subscribing when clients (un)subscribe
void PointCloudXyzrgbNodelet::connectCb()
{
  if (pub_point_cloud_.getNumSubscribers() == 0)
  {
    sub_depth_.unsubscribe();
    sub_rgb_  .unsubscribe();
    sub_info_ .unsubscribe();
    subscribed_ = false;
  }
  else if (!subscribed_)
  {
    sub_depth_.subscribe(*depth_it_, "image_rect_raw",   1);
    sub_rgb_  .subscribe(*rgb_it_,   "image_rect_color", 1);
    sub_info_ .subscribe(*rgb_nh_,   "camera_info",      1);
    subscribed_ = true;
  }
}

void PointCloudXyzrgbNodelet::imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                      const sensor_msgs::ImageConstPtr& rgb_msg,
                                      const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // Check for bad inputs
  if (depth_msg->encoding != enc::TYPE_16UC1)
  {
    NODELET_ERROR_THROTTLE(5, "Expected depth data of type [%s], got [%s]",
                           enc::TYPE_16UC1.c_str(), depth_msg->encoding.c_str());
    return;
  }

  if (depth_msg->header.frame_id != rgb_msg->header.frame_id)
  {
    NODELET_ERROR_THROTTLE(5, "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
                           depth_msg->header.frame_id.c_str(), rgb_msg->header.frame_id.c_str());
    return;
  }

  if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height)
  {
    NODELET_ERROR_THROTTLE(5, "Depth resolution (%ux%u) does not match RGB resolution (%ux%u)",
                           depth_msg->width, depth_msg->height, rgb_msg->width, rgb_msg->height);
    return;
  }

  // Supported color encodings: RGB8, BGR8, MONO8
  int red_offset, green_offset, blue_offset, color_step;
  if (rgb_msg->encoding == enc::RGB8)
  {
    red_offset   = 0;
    green_offset = 1;
    blue_offset  = 2;
    color_step   = 3;
  }
  else if (rgb_msg->encoding == enc::BGR8)
  {
    red_offset   = 2;
    green_offset = 1;
    blue_offset  = 0;
    color_step   = 3;
  }
  else if (rgb_msg->encoding == enc::MONO8)
  {
    red_offset   = 0;
    green_offset = 0;
    blue_offset  = 0;
    color_step   = 1;
  }
  else
  {
    NODELET_ERROR_THROTTLE(5, "Unsupported encoding [%s]", rgb_msg->encoding.c_str());
    return;
  }

  // Allocate new point cloud message
  PointCloud::Ptr cloud_msg (new PointCloud);
  cloud_msg->header = depth_msg->header; // Use depth image time stamp
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
  
  int depth_idx = 0, color_idx = 0;
  const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
  const uint8_t* rgb_buffer = &rgb_msg->data[0];
  PointCloud::iterator pt_iter = cloud_msg->begin ();

  for (int v = 0; v < (int)cloud_msg->height; ++v)
  {
    for (int u = 0; u < (int)cloud_msg->width; ++u, ++depth_idx, color_idx += color_step, ++pt_iter)
    {
      pcl::PointXYZRGB& pt = *pt_iter;

      // Check for invalid measurements
      if (depth_data[depth_idx] == 0)
      {
        pt.x = pt.y = pt.z = bad_point;
      }
      else
      {
        // Fill in XYZ
        pt.x = (u - center_x) * depth_data[depth_idx] * constant_x;
        pt.y = (v - center_y) * depth_data[depth_idx] * constant_y;
        pt.z = depth_data[depth_idx] * 0.001;
      }

      // Fill in color
      RGBValue color;
      color.Red   = rgb_buffer[color_idx + red_offset];
      color.Green = rgb_buffer[color_idx + green_offset];
      color.Blue  = rgb_buffer[color_idx + blue_offset];
      color.Alpha = 0;
      pt.rgb = color.float_value;
    }
  }

  pub_point_cloud_.publish (cloud_msg);
}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (depth_image_proc, point_cloud_xyzrgb, depth_image_proc::PointCloudXyzrgbNodelet, nodelet::Nodelet);
