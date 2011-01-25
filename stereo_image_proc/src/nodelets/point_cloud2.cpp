#include "stereo_image_proc/nodelets/point_cloud2.h"
#include <image_proc/advertisement_checker.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(stereo_image_proc, point_cloud2,
                        stereo_image_proc::PointCloud2Nodelet, nodelet::Nodelet)

namespace stereo_image_proc {

using namespace sensor_msgs;
using namespace stereo_msgs;

struct PointCloud2Nodelet::Impl
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  // Subscriptions
  /// @todo Implement (optional) approx synch of left and right cameras
  image_transport::SubscriberFilter sub_l_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  message_filters::Subscriber<DisparityImage> sub_disparity_;
  message_filters::TimeSynchronizer<Image, CameraInfo, CameraInfo, DisparityImage> sync_;
  bool subscribed_;

  // Publications
  ros::Publisher pub_points2_;

  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;
  cv::Mat_<cv::Vec3f> points_mat_; // scratch buffer

  // Error reporting
  image_proc::AdvertisementChecker check_inputs_;

  Impl(const ros::NodeHandle& nh, const std::string& name)
    : nh_(nh),
      it_(nh),
      sync_(3), /// @todo Parameter for sync queue size
      subscribed_(false),
      check_inputs_(nh, name)
  {
  }
};

void PointCloud2Nodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  impl_ = boost::make_shared<Impl>(nh, getName());
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloud2Nodelet::connectCb, this);
  impl_->pub_points2_  = nh.advertise<PointCloud2>("points2",  1, connect_cb, connect_cb);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  impl_->sync_.connectInput(impl_->sub_l_image_, impl_->sub_l_info_,
                            impl_->sub_r_info_, impl_->sub_disparity_);
  impl_->sync_.registerCallback(boost::bind(&PointCloud2Nodelet::imageCb, this, _1, _2, _3, _4));

  // Print a warning every minute until the input topics are advertised
  ros::V_string topics;
  topics.push_back("left/image_rect_color");
  topics.push_back("left/camera_info");
  topics.push_back("right/camera_info");
  topics.push_back("disparity");
  impl_->check_inputs_.start(topics, 60.0);
}

// Handles (un)subscribing when clients (un)subscribe
void PointCloud2Nodelet::connectCb()
{
  if (impl_->pub_points2_.getNumSubscribers() == 0)
  {
    impl_->sub_l_image_  .unsubscribe();
    impl_->sub_l_info_   .unsubscribe();
    impl_->sub_r_info_   .unsubscribe();
    impl_->sub_disparity_.unsubscribe();
    impl_->subscribed_ = false;
  }
  else if (!impl_->subscribed_)
  {
    impl_->sub_l_image_  .subscribe(impl_->it_, "left/image_rect_color", 1);
    impl_->sub_l_info_   .subscribe(impl_->nh_, "left/camera_info", 1);
    impl_->sub_r_info_   .subscribe(impl_->nh_, "right/camera_info", 1);
    impl_->sub_disparity_.subscribe(impl_->nh_, "disparity", 1);
    impl_->subscribed_ = true;
  }
  /// @todo Parameter for queue size
}

inline bool isValidPoint(const cv::Vec3f& pt)
{
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
}

void PointCloud2Nodelet::imageCb(const ImageConstPtr& l_image_msg,
                                 const CameraInfoConstPtr& l_info_msg,
                                 const CameraInfoConstPtr& r_info_msg,
                                 const DisparityImageConstPtr& disp_msg)
{
  // Update the camera model
  impl_->model_.fromCameraInfo(l_info_msg, r_info_msg);

  // Calculate dense point cloud
  const Image& dimage = disp_msg->image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  impl_->model_.projectDisparityImageTo3d(dmat, impl_->points_mat_, true);
  cv::Mat_<cv::Vec3f> mat = impl_->points_mat_;

  // Fill in new PointCloud2 message
  PointCloud2Ptr points_msg = boost::make_shared<PointCloud2>();
  points_msg->height = mat.rows;
  points_msg->width  = mat.cols;
  points_msg->fields.resize (4);
  points_msg->fields[0].name = "x";
  points_msg->fields[0].offset = 0;
  points_msg->fields[0].count = 1;
  points_msg->fields[0].datatype = PointField::FLOAT32;
  points_msg->fields[1].name = "y";
  points_msg->fields[1].offset = 4;
  points_msg->fields[1].count = 1;
  points_msg->fields[1].datatype = PointField::FLOAT32;
  points_msg->fields[2].name = "z";
  points_msg->fields[2].offset = 8;
  points_msg->fields[2].count = 1;
  points_msg->fields[2].datatype = PointField::FLOAT32;
  points_msg->fields[3].name = "rgb";
  points_msg->fields[3].offset = 12;
  points_msg->fields[3].count = 1;
  points_msg->fields[3].datatype = PointField::FLOAT32;
  //points_msg->is_bigendian = false; ???
  static const int STEP = 16;
  points_msg->point_step = STEP;
  points_msg->row_step = points_msg->point_step * points_msg->width;
  points_msg->data.resize (points_msg->row_step * points_msg->height);
  points_msg->is_dense = false; // there may be invalid points
 
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  int offset = 0;
  for (int v = 0; v < mat.rows; ++v)
  {
    for (int u = 0; u < mat.cols; ++u, offset += STEP)
    {
      if (isValidPoint(mat(v,u)))
      {
        // x,y,z,rgba
        memcpy (&points_msg->data[offset + 0], &mat(v,u)[0], sizeof (float));
        memcpy (&points_msg->data[offset + 4], &mat(v,u)[1], sizeof (float));
        memcpy (&points_msg->data[offset + 8], &mat(v,u)[2], sizeof (float));
      }
      else
      {
        memcpy (&points_msg->data[offset + 0], &bad_point, sizeof (float));
        memcpy (&points_msg->data[offset + 4], &bad_point, sizeof (float));
        memcpy (&points_msg->data[offset + 8], &bad_point, sizeof (float));
      }
    }
  }

  // Fill in color
  namespace enc = sensor_msgs::image_encodings;
  const std::string& encoding = l_image_msg->encoding;
  offset = 0;
  if (encoding == enc::MONO8)
  {
    const cv::Mat_<uint8_t> color(l_image_msg->height, l_image_msg->width,
                                  (uint8_t*)&l_image_msg->data[0],
                                  l_image_msg->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, offset += STEP)
      {
        if (isValidPoint(mat(v,u)))
        {
          uint8_t g = color(v,u);
          int32_t rgb = (g << 16) | (g << 8) | g;
          memcpy (&points_msg->data[offset + 12], &rgb, sizeof (int32_t));
        }
        else
        {
          memcpy (&points_msg->data[offset + 12], &bad_point, sizeof (float));
        }
      }
    }
  }
  else if (encoding == enc::RGB8)
  {
    const cv::Mat_<cv::Vec3b> color(l_image_msg->height, l_image_msg->width,
                                    (cv::Vec3b*)&l_image_msg->data[0],
                                    l_image_msg->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, offset += STEP)
      {
        if (isValidPoint(mat(v,u)))
        {
          const cv::Vec3b& rgb = color(v,u);
          int32_t rgb_packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
          memcpy (&points_msg->data[offset + 12], &rgb_packed, sizeof (int32_t));
        }
        else
        {
          memcpy (&points_msg->data[offset + 12], &bad_point, sizeof (float));
        }
      }
    }
  }
  else if (encoding == enc::BGR8)
  {
    const cv::Mat_<cv::Vec3b> color(l_image_msg->height, l_image_msg->width,
                                    (cv::Vec3b*)&l_image_msg->data[0],
                                    l_image_msg->step);
    for (int v = 0; v < mat.rows; ++v)
    {
      for (int u = 0; u < mat.cols; ++u, offset += STEP)
      {
        if (isValidPoint(mat(v,u)))
        {
          const cv::Vec3b& bgr = color(v,u);
          int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
          memcpy (&points_msg->data[offset + 12], &rgb_packed, sizeof (int32_t));
        }
        else
        {
          memcpy (&points_msg->data[offset + 12], &bad_point, sizeof (float));
        }
      }
    }
  }
  else
  {
    ROS_WARN_THROTTLE_NAMED(30, getName(),
                            "Could not fill color channel of the point cloud, "
                            "unrecognized encoding '%s'", encoding.c_str());
  }
}

} // namespace stereo_image_proc
