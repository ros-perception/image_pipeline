#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <Eigen/Core>

namespace depth_image_proc {

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

class RegisterNodelet : public nodelet::Nodelet
{
  ros::NodeHandlePtr nh_depth_, nh_rgb_;
  boost::shared_ptr<image_transport::ImageTransport> it_depth_;
  
  // Subscriptions
  image_transport::SubscriberFilter sub_depth_image_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_depth_info_, sub_rgb_info_;
  boost::shared_ptr<tf::TransformListener> tf_;
  typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  boost::shared_ptr<Synchronizer> sync_;
  bool subscribed_;

  // Publications
  image_transport::CameraPublisher pub_registered_;

  image_geometry::PinholeCameraModel depth_model_, rgb_model_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& depth_image_msg,
               const sensor_msgs::CameraInfoConstPtr& depth_info_msg,
               const sensor_msgs::CameraInfoConstPtr& rgb_info_msg);
};

void RegisterNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  nh_depth_.reset( new ros::NodeHandle(nh, "depth") );
  nh_rgb_.reset( new ros::NodeHandle(nh, "rgb") );
  it_depth_.reset( new image_transport::ImageTransport(*nh_depth_) );
  tf_.reset( new tf::TransformListener );

  // Read parameters
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);

  // Monitor whether anyone is subscribed to the output
  subscribed_ = false;
  image_transport::ImageTransport it_depth_reg(ros::NodeHandle(nh, "depth_registered"));
  image_transport::SubscriberStatusCallback image_connect_cb = boost::bind(&RegisterNodelet::connectCb, this);
  ros::SubscriberStatusCallback info_connect_cb = boost::bind(&RegisterNodelet::connectCb, this);
  pub_registered_ = it_depth_reg.advertiseCamera("image_rect", 1,
                                                 image_connect_cb, image_connect_cb,
                                                 info_connect_cb, info_connect_cb);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_image_, sub_depth_info_, sub_rgb_info_) );
  sync_->registerCallback(boost::bind(&RegisterNodelet::imageCb, this, _1, _2, _3));
}

// Handles (un)subscribing when clients (un)subscribe
void RegisterNodelet::connectCb()
{
  if (pub_registered_.getNumSubscribers() == 0)
  {
    sub_depth_image_.unsubscribe();
    sub_depth_info_ .unsubscribe();
    sub_rgb_info_   .unsubscribe();
    subscribed_ = false;
  }
  else if (!subscribed_)
  {
    sub_depth_image_.subscribe(*it_depth_, "image_rect",  1);
    sub_depth_info_ .subscribe(*nh_depth_, "camera_info", 1);
    sub_rgb_info_   .subscribe(*nh_rgb_,   "camera_info", 1);
    subscribed_ = true;
  }
}

void RegisterNodelet::imageCb(const sensor_msgs::ImageConstPtr& depth_image_msg,
                              const sensor_msgs::CameraInfoConstPtr& depth_info_msg,
                              const sensor_msgs::CameraInfoConstPtr& rgb_info_msg)
{
  // Check for bad inputs
  if (depth_image_msg->encoding != enc::TYPE_16UC1)
  {
    NODELET_ERROR("Expected depth data of type [%s], got [%s]",
                  enc::TYPE_16UC1.c_str(), depth_image_msg->encoding.c_str());
    return;
  }

  // Update camera models - these take binning & ROI into account
  depth_model_.fromCameraInfo(depth_info_msg);
  rgb_model_  .fromCameraInfo(rgb_info_msg);

  // Query tf for transform from (X,Y,Z) in depth camera frame to RGB camera frame
  Eigen::Matrix4d depth_to_rgb;
  try
  {
    ros::Duration timeout(1.0 / 30); /// @todo Parameter for expected frame rate
    tf_->waitForTransform(rgb_info_msg->header.frame_id, depth_info_msg->header.frame_id,
                          depth_info_msg->header.stamp, timeout);
    tf::StampedTransform transform;
    tf_->lookupTransform (rgb_info_msg->header.frame_id, depth_info_msg->header.frame_id,
                          depth_info_msg->header.stamp, transform);

    btMatrix3x3& R = transform.getBasis();
    btVector3&   T = transform.getOrigin();
    depth_to_rgb << R[0][0], R[0][1], R[0][2], T[0],
                    R[1][0], R[1][1], R[1][2], T[1],
                    R[2][0], R[2][1], R[2][2], T[2],
                          0,       0,       0,    1;
  }
  catch (tf::TransformException& ex)
  {
    NODELET_WARN_THROTTLE(2, "TF exception:\n%s", ex.what());
    return;
    /// @todo Can take on order of a minute to register a disconnect callback when we
    /// don't call publish() in this cb. What's going on roscpp?
  }

  // Allocate registered depth image
  sensor_msgs::ImagePtr registered_msg( new sensor_msgs::Image );
  registered_msg->header.stamp    = depth_image_msg->header.stamp;
  registered_msg->header.frame_id = rgb_info_msg->header.frame_id;
  registered_msg->encoding        = depth_image_msg->encoding;
  
  cv::Size resolution = rgb_model_.reducedResolution();
  registered_msg->height = resolution.height;
  registered_msg->width  = resolution.width;
  registered_msg->step   = registered_msg->width * sizeof(uint16_t);
  registered_msg->data.resize( registered_msg->height * registered_msg->step ); // zero filled

  // Extract all the parameters we need
  double inv_depth_fx = 1.0 / depth_model_.fx();
  double inv_depth_fy = 1.0 / depth_model_.fy();
  double depth_cx = depth_model_.cx(), depth_cy = depth_model_.cy();
  double depth_Tx = depth_model_.Tx(), depth_Ty = depth_model_.Ty();
  double rgb_fx = rgb_model_.fx(), rgb_fy = rgb_model_.fy();
  double rgb_cx = rgb_model_.cx(), rgb_cy = rgb_model_.cy();
  double rgb_Tx = rgb_model_.Tx(), rgb_Ty = rgb_model_.Ty();
  
  // Transform the depth values into the RGB frame
  /// @todo When RGB is higher res, interpolate by rasterizing depth triangles onto the registered image  
  const uint16_t* raw_data = reinterpret_cast<const uint16_t*>(&depth_image_msg->data[0]);
  uint16_t* reg_data = reinterpret_cast<uint16_t*>(&registered_msg->data[0]);
  int raw_index = 0;
  for (unsigned v = 0; v < depth_image_msg->height; ++v)
  {
    for (unsigned u = 0; u < depth_image_msg->width; ++u, ++raw_index)
    {
      double depth = raw_data[raw_index] * 0.001; // convert mm->m

      // Reproject (u,v,Z) to (X,Y,Z,1) in depth camera frame
      Eigen::Vector4d xyz_depth;
      xyz_depth << ((u - depth_cx)*depth - depth_Tx) * inv_depth_fx,
                   ((v - depth_cy)*depth - depth_Ty) * inv_depth_fy,
                   depth,
                   1;

      // Transform to RGB camera frame
      Eigen::Vector4d xyz_rgb = depth_to_rgb * xyz_depth;

      // Project to (u,v) in RGB image
      double inv_Z = 1.0 / xyz_rgb.z();
      int u_rgb = (rgb_fx*xyz_rgb.x() + rgb_Tx)*inv_Z + rgb_cx + 0.5;
      int v_rgb = (rgb_fy*xyz_rgb.y() + rgb_Ty)*inv_Z + rgb_cy + 0.5;
      
      if (u_rgb < 0 || u_rgb >= (int)registered_msg->width ||
          v_rgb < 0 || v_rgb >= (int)registered_msg->height)
        continue;
      
      uint16_t& reg_depth = reg_data[v_rgb*registered_msg->width + u_rgb];
      uint16_t  new_depth = (xyz_rgb.z() * 1000.0) + 0.5; // Convert back to uint16 mm
      // Validity and Z-buffer checks
      if (reg_depth == 0 || reg_depth > new_depth)
        reg_depth = new_depth;
    }
  }

  // Registered camera info is the same as the RGB info, but uses the depth timestamp
  sensor_msgs::CameraInfoPtr registered_info_msg( new sensor_msgs::CameraInfo(*rgb_info_msg) );
  registered_info_msg->header.stamp = registered_msg->header.stamp;

  pub_registered_.publish(registered_msg, registered_info_msg);
}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (depth_image_proc, register, depth_image_proc::RegisterNodelet, nodelet::Nodelet);
