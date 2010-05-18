#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <image_rotate/ImageRotateConfig.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <dynamic_reconfigure/server.h>
#include <math.h>

class ImageRotater
{
  tf::TransformListener tf_sub_;
  tf::TransformBroadcaster tf_pub_;

  image_rotate::ImageRotateConfig config_;
  dynamic_reconfigure::Server<image_rotate::ImageRotateConfig> srv;

  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;

  sensor_msgs::CvBridge bridge_;

  tf::Stamped<tf::Vector3> target_vector_;
  tf::Stamped<tf::Vector3> reference_vector_;

  double angle_;
  ros::Time prev_stamp_;

  void reconfigureCallback(image_rotate::ImageRotateConfig &new_config, uint32_t level)
  {
    config_ = new_config;
    target_vector_.setValue(config_.target_x, config_.target_y, config_.target_z);
    reference_vector_.setValue(config_.reference_x, config_.reference_y, config_.reference_z);
  }

  const std::string &frameWithDefault(const std::string &frame, const std::string &image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {

    try
    {
      // Transform the target vector into the image frame.
      target_vector_.stamp_ = msg->header.stamp;
      target_vector_.frame_id_ = frameWithDefault(config_.target_frame_id, msg->header.frame_id);
      tf::Stamped<tf::Vector3> target_vector_transformed;
      tf_sub_.waitForTransform(msg->header.frame_id, msg->header.stamp,
                               target_vector_.frame_id_, target_vector_.stamp_,
                               msg->header.frame_id, ros::Duration(0.2));
      tf_sub_.transformVector(msg->header.frame_id, msg->header.stamp, target_vector_,
                              msg->header.frame_id, target_vector_transformed);

      // Transform the reference vector into the image frame.
      reference_vector_.stamp_ = msg->header.stamp;
      reference_vector_.frame_id_ = frameWithDefault(config_.reference_frame_id, msg->header.frame_id);
      tf::Stamped<tf::Vector3> reference_vector_transformed;
      tf_sub_.waitForTransform(msg->header.frame_id, msg->header.stamp,
                               reference_vector_.frame_id_, reference_vector_.stamp_,
                               msg->header.frame_id, ros::Duration(0.01));
      tf_sub_.transformVector(msg->header.frame_id, msg->header.stamp, reference_vector_,
                              msg->header.frame_id, reference_vector_transformed);

      //ROS_INFO("target: %f %f %f", target_vector_.x(), target_vector_.y(), target_vector_.z());
      //ROS_INFO("target_transformed: %f %f %f", target_vector_transformed.x(), target_vector_transformed.y(), target_vector_transformed.z());
      //ROS_INFO("reference: %f %f %f", reference_vector_.x(), reference_vector_.y(), reference_vector_.z());
      //ROS_INFO("reference_transformed: %f %f %f", reference_vector_transformed.x(), reference_vector_transformed.y(), reference_vector_transformed.z());

      // Calculate the angle of the rotation.
      double angle = angle_;
      if ((target_vector_transformed.x()    != 0 || target_vector_transformed.y()    != 0) &&
          (reference_vector_transformed.x() != 0 || reference_vector_transformed.y() != 0))
      {
        angle = atan2(target_vector_transformed.y(), target_vector_transformed.x());
        angle -= atan2(reference_vector_transformed.y(), reference_vector_transformed.x());
      }

      // Rate limit the rotation.
      if (config_.max_angular_rate == 0)
        angle_ = angle;
      else
      {
        double delta = fmod(angle - angle_, 2.0 * M_PI);
        if (delta > M_PI)
          delta -= 2.0 * M_PI;
        else if (delta < - M_PI)
          delta += 2.0 * M_PI;

        double max_delta = config_.max_angular_rate * (msg->header.stamp - prev_stamp_).toSec();
        if (delta > max_delta)
          delta = max_delta;
        else if (delta < -max_delta)
          delta = - max_delta;

        angle_ += delta;
      }
      angle_ = fmod(angle_, 2.0 * M_PI);
    }
    catch (tf::TransformException &e)
    {
      ROS_ERROR("Transform error: %s", e.what());
    }

    //ROS_INFO("angle: %f", 180 * angle_ / M_PI);

    // Publish the transform.
    tf::StampedTransform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform.setRotation(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), angle_));
    transform.frame_id_ = msg->header.frame_id;
    transform.child_frame_id_ = frameWithDefault(config_.output_frame_id, msg->header.frame_id + "_rotated");
    transform.stamp_ = msg->header.stamp;
    tf_pub_.sendTransform(transform);

    // Transform the image.
    try
    {
      // Convert the image into something opencv can handle.
      IplImage *in_image_ipl = bridge_.imgMsgToCv(msg, "passthrough");
      cv::Mat in_image = in_image_ipl;

      // Compute the output image size.
      int max_dim = in_image.cols > in_image.rows ? in_image.cols : in_image.rows;
      int min_dim = in_image.cols < in_image.rows ? in_image.cols : in_image.rows;
      int noblack_dim = min_dim / sqrt(2);
      int diag_dim = sqrt(pow(in_image.cols, 2) + pow(in_image.rows, 2));
      int out_size;
      int candidates[] = { noblack_dim, min_dim, max_dim, diag_dim, diag_dim }; // diag_dim repeated to simplify limit case.
      int step = config_.output_image_size;
      out_size = candidates[step] + (candidates[step + 1] - candidates[step]) * (config_.output_image_size - step);
      //ROS_INFO("out_size: %d", out_size);

      // Compute the rotation matrix.
      cv::Mat rot_matrix = cv::getRotationMatrix2D(cv::Point2f(in_image.cols / 2.0, in_image.rows / 2.0), 180 * angle_ / M_PI, 1);
      cv::Mat translation = rot_matrix.col(2);
      rot_matrix.at<double>(0, 2) += (out_size - in_image.cols) / 2.0;
      rot_matrix.at<double>(1, 2) += (out_size - in_image.rows) / 2.0;

      // Do the rotation
      cv::Mat out_image;
      cv::warpAffine(in_image, out_image, rot_matrix, cv::Size(out_size, out_size));

      // Publish the image.
      IplImage out_msg = out_image;
      sensor_msgs::Image::Ptr out_img = bridge_.cvToImgMsg(&out_msg, "passthrough");
      img_pub_.publish(out_img);
    }
    catch (cv::Exception &e)
    {
      ROS_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

public:
  ImageRotater(ros::NodeHandle nh = ros::NodeHandle()) : angle_(0), prev_stamp_(0, 0)
  {
    image_transport::ImageTransport it(nh);
    img_sub_ = it.subscribe("image", 1, &ImageRotater::imageCallback, this);
    img_pub_ = it.advertise("image_rotated", 1);
    dynamic_reconfigure::Server<image_rotate::ImageRotateConfig>::CallbackType f =
      boost::bind(&ImageRotater::reconfigureCallback, this, _1, _2);
    srv.setCallback(f);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_rotate", ros::init_options::AnonymousName);

  ImageRotater ir;
  ros::spin();
}
