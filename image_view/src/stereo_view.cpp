/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include <boost/format.hpp>

#ifdef HAVE_GTK

#include <gtk/gtk.h>

#endif

#include <chrono>
#include <memory>
#include <mutex>

namespace image_view
{

namespace enc = sensor_msgs::image_encodings;

// colormap for disparities, RGB
static unsigned char colormap[768] = 
  { 150, 150, 150,
    107, 0, 12,
    106, 0, 18,
    105, 0, 24,
    103, 0, 30,
    102, 0, 36,
    101, 0, 42,
    99, 0, 48,
    98, 0, 54,
    97, 0, 60,
    96, 0, 66,
    94, 0, 72,
    93, 0, 78,
    92, 0, 84,
    91, 0, 90,
    89, 0, 96,
    88, 0, 102,
    87, 0, 108,
    85, 0, 114,
    84, 0, 120,
    83, 0, 126,
    82, 0, 131,
    80, 0, 137,
    79, 0, 143,
    78, 0, 149,
    77, 0, 155,
    75, 0, 161,
    74, 0, 167,
    73, 0, 173,
    71, 0, 179,
    70, 0, 185,
    69, 0, 191,
    68, 0, 197,
    66, 0, 203,
    65, 0, 209,
    64, 0, 215,
    62, 0, 221,
    61, 0, 227,
    60, 0, 233,
    59, 0, 239,
    57, 0, 245,
    56, 0, 251,
    55, 0, 255,
    54, 0, 255,
    52, 0, 255,
    51, 0, 255,
    50, 0, 255,
    48, 0, 255,
    47, 0, 255,
    46, 0, 255,
    45, 0, 255,
    43, 0, 255,
    42, 0, 255,
    41, 0, 255,
    40, 0, 255,
    38, 0, 255,
    37, 0, 255,
    36, 0, 255,
    34, 0, 255,
    33, 0, 255,
    32, 0, 255,
    31, 0, 255,
    29, 0, 255,
    28, 0, 255,
    27, 0, 255,
    26, 0, 255,
    24, 0, 255,
    23, 0, 255,
    22, 0, 255,
    20, 0, 255,
    19, 0, 255,
    18, 0, 255,
    17, 0, 255,
    15, 0, 255,
    14, 0, 255,
    13, 0, 255,
    11, 0, 255,
    10, 0, 255,
    9, 0, 255,
    8, 0, 255,
    6, 0, 255,
    5, 0, 255,
    4, 0, 255,
    3, 0, 255,
    1, 0, 255,
    0, 4, 255,
    0, 10, 255,
    0, 16, 255,
    0, 22, 255,
    0, 28, 255,
    0, 34, 255,
    0, 40, 255,
    0, 46, 255,
    0, 52, 255,
    0, 58, 255,
    0, 64, 255,
    0, 70, 255,
    0, 76, 255,
    0, 82, 255,
    0, 88, 255,
    0, 94, 255,
    0, 100, 255,
    0, 106, 255,
    0, 112, 255,
    0, 118, 255,
    0, 124, 255,
    0, 129, 255,
    0, 135, 255,
    0, 141, 255,
    0, 147, 255,
    0, 153, 255,
    0, 159, 255,
    0, 165, 255,
    0, 171, 255,
    0, 177, 255,
    0, 183, 255,
    0, 189, 255,
    0, 195, 255,
    0, 201, 255,
    0, 207, 255,
    0, 213, 255,
    0, 219, 255,
    0, 225, 255,
    0, 231, 255,
    0, 237, 255,
    0, 243, 255,
    0, 249, 255,
    0, 255, 255,
    0, 255, 249,
    0, 255, 243,
    0, 255, 237,
    0, 255, 231,
    0, 255, 225,
    0, 255, 219,
    0, 255, 213,
    0, 255, 207,
    0, 255, 201,
    0, 255, 195,
    0, 255, 189,
    0, 255, 183,
    0, 255, 177,
    0, 255, 171,
    0, 255, 165,
    0, 255, 159,
    0, 255, 153,
    0, 255, 147,
    0, 255, 141,
    0, 255, 135,
    0, 255, 129,
    0, 255, 124,
    0, 255, 118,
    0, 255, 112,
    0, 255, 106,
    0, 255, 100,
    0, 255, 94,
    0, 255, 88,
    0, 255, 82,
    0, 255, 76,
    0, 255, 70,
    0, 255, 64,
    0, 255, 58,
    0, 255, 52,
    0, 255, 46,
    0, 255, 40,
    0, 255, 34,
    0, 255, 28,
    0, 255, 22,
    0, 255, 16,
    0, 255, 10,
    0, 255, 4,
    2, 255, 0,
    8, 255, 0,
    14, 255, 0,
    20, 255, 0,
    26, 255, 0,
    32, 255, 0,
    38, 255, 0,
    44, 255, 0,
    50, 255, 0,
    56, 255, 0,
    62, 255, 0,
    68, 255, 0,
    74, 255, 0,
    80, 255, 0,
    86, 255, 0,
    92, 255, 0,
    98, 255, 0,
    104, 255, 0,
    110, 255, 0,
    116, 255, 0,
    122, 255, 0,
    128, 255, 0,
    133, 255, 0,
    139, 255, 0,
    145, 255, 0,
    151, 255, 0,
    157, 255, 0,
    163, 255, 0,
    169, 255, 0,
    175, 255, 0,
    181, 255, 0,
    187, 255, 0,
    193, 255, 0,
    199, 255, 0,
    205, 255, 0,
    211, 255, 0,
    217, 255, 0,
    223, 255, 0,
    229, 255, 0,
    235, 255, 0,
    241, 255, 0,
    247, 255, 0,
    253, 255, 0,
    255, 251, 0,
    255, 245, 0,
    255, 239, 0,
    255, 233, 0,
    255, 227, 0,
    255, 221, 0,
    255, 215, 0,
    255, 209, 0,
    255, 203, 0,
    255, 197, 0,
    255, 191, 0,
    255, 185, 0,
    255, 179, 0,
    255, 173, 0,
    255, 167, 0,
    255, 161, 0,
    255, 155, 0,
    255, 149, 0,
    255, 143, 0,
    255, 137, 0,
    255, 131, 0,
    255, 126, 0,
    255, 120, 0,
    255, 114, 0,
    255, 108, 0,
    255, 102, 0,
    255, 96, 0,
    255, 90, 0,
    255, 84, 0,
    255, 78, 0,
    255, 72, 0,
    255, 66, 0,
    255, 60, 0,
    255, 54, 0,
    255, 48, 0,
    255, 42, 0,
    255, 36, 0,
    255, 30, 0,
    255, 24, 0,
    255, 18, 0,
    255, 12, 0,
    255,  6, 0,
    255,  0, 0,
  };

inline void increment(int * value)
{
  ++(*value);
}

using sensor_msgs::msg::Image;
using stereo_msgs::msg::DisparityImage;
using message_filters::sync_policies::ExactTime;
using message_filters::sync_policies::ApproximateTime;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class StereoViewNode
  : public rclcpp::Node
{
private:
  image_transport::SubscriberFilter left_sub_, right_sub_;
  message_filters::Subscriber<DisparityImage> disparity_sub_;
  typedef ExactTime<Image, Image, DisparityImage> ExactPolicy;
  typedef ApproximateTime<Image, Image, DisparityImage> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  std::shared_ptr<ExactSync> exact_sync_;
  std::shared_ptr<ApproximateSync> approximate_sync_;
  int queue_size_;
  
  Image::ConstSharedPtr last_left_msg_, last_right_msg_;
  cv::Mat last_left_image_, last_right_image_;
  cv::Mat_<cv::Vec3b> disparity_color_;
  std::mutex image_mutex_;
  
  boost::format filename_format_;
  int save_count_;

  rclcpp::TimerBase::SharedPtr check_synced_timer_;
  int left_received_, right_received_, disp_received_, all_received_;

public:
  StereoViewNode(const rclcpp::NodeOptions & options, const std::string & transport)
  : rclcpp::Node("stereo_view_node", options),
    filename_format_(""), save_count_(0),
    left_received_(0), right_received_(0), disp_received_(0), all_received_(0)
  {
    // Read local parameters
    bool autosize = this->declare_parameter("autosize", true);
    
    std::string format_string;
    format_string = this->declare_parameter("filename_format", std::string("%s%04i.jpg"));
    filename_format_.parse(format_string);

    // Do GUI window setup
    int flags = autosize ? (cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED) : 0;
    cv::namedWindow("left", flags);
    cv::namedWindow("right", flags);
    cv::namedWindow("disparity", flags);
    cv::setMouseCallback("left", &StereoViewNode::mouseCb, this);
    cv::setMouseCallback("right", &StereoViewNode::mouseCb, this);
    cv::setMouseCallback("disparity", &StereoViewNode::mouseCb, this);

    // Resolve topic names
    std::string stereo_ns = rclcpp::expand_topic_or_service_name(
      "stereo", this->get_name(), this->get_namespace());
    std::string left_topic = rclcpp::expand_topic_or_service_name(stereo_ns + "/left/" +
      rclcpp::expand_topic_or_service_name("image", this->get_name(), this->get_namespace()),
      this->get_name(), this->get_namespace());
    std::string right_topic = rclcpp::expand_topic_or_service_name(stereo_ns + "/right/" +
      rclcpp::expand_topic_or_service_name("image", this->get_name(), this->get_namespace()),
      this->get_name(), this->get_namespace());
    std::string disparity_topic = rclcpp::expand_topic_or_service_name(stereo_ns + "/disparity",
      this->get_name(), this->get_namespace());
    RCLCPP_INFO(
      this->get_logger(),
      "Subscribing to:\n\t* %s\n\t* %s\n\t* %s",
      left_topic.c_str(), right_topic.c_str(),
      disparity_topic.c_str());

    // Subscribe to three input topics.
    left_sub_.subscribe(this, left_topic, transport);
    right_sub_.subscribe(this, right_topic, transport);
    disparity_sub_.subscribe(this, disparity_topic);

    // Complain every 30s if the topics appear unsynchronized
    left_sub_.registerCallback(std::bind(increment, &left_received_));
    right_sub_.registerCallback(std::bind(increment, &right_received_));
    disparity_sub_.registerCallback(std::bind(increment, &disp_received_));
    check_synced_timer_ = this->create_wall_timer(
      std::chrono::seconds(15),
      std::bind(&StereoViewNode::checkInputsSynchronized, this));

    // Synchronize input topics. Optionally do approximate synchronization.
    queue_size_ = this->declare_parameter("queue_size", 5);
    bool approx = this->declare_parameter("approximate_sync", false);
    if (approx)
    {
      approximate_sync_.reset(new ApproximateSync(
        ApproximatePolicy(queue_size_), left_sub_, right_sub_, disparity_sub_));
      approximate_sync_->registerCallback(std::bind(
        &StereoViewNode::imageCb, this, _1, _2, _3));
    }
    else
    {
      exact_sync_.reset( new ExactSync(ExactPolicy(queue_size_),
                                       left_sub_, right_sub_, disparity_sub_) );
      exact_sync_->registerCallback(std::bind(
        &StereoViewNode::imageCb, this, _1, _2, _3));
    }

    if (rclcpp::expand_topic_or_service_name("stereo", this->get_name(), this->get_namespace()) == "stereo") {
      RCLCPP_WARN(
        this->get_logger(), "'stereo' has not been remapped! Example command-line usage:\n"
        "\t$ rosrun image_view stereo_view stereo:=narrow_stereo image:=image_color");
    }
    if (rclcpp::expand_topic_or_service_name("image", this->get_name(), this->get_namespace()) == "/image_raw") {
      RCLCPP_WARN(
        this->get_logger(), "There is a delay between when the camera drivers publish the raw images and "
        "when stereo_image_proc publishes the computed point cloud. stereo_view "
        "may fail to synchronize these topics without a large queue_size.");
    }
  }

  ~StereoViewNode()
  {
    cv::destroyAllWindows();
  }

  void imageCb(
    const Image::ConstSharedPtr & left, const Image::ConstSharedPtr & right,
    const DisparityImage::ConstSharedPtr& disparity_msg)
  {
    ++all_received_; // For error checking
    
    image_mutex_.lock();

    // May want to view raw bayer data
    if (left->encoding.find("bayer") != std::string::npos)
      std::const_pointer_cast<Image>(left)->encoding = "mono8";
    if (right->encoding.find("bayer") != std::string::npos)
      std::const_pointer_cast<Image>(right)->encoding = "mono8";

    // Hang on to image data for sake of mouseCb
    last_left_msg_ = left;
    last_right_msg_ = right;
    try {
      last_left_image_ = cv_bridge::toCvShare(left, "bgr8")->image;
      last_right_image_ = cv_bridge::toCvShare(right, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(
        this->get_logger(), "Unable to convert one of '%s' or '%s' to 'bgr8'",
        left->encoding.c_str(), right->encoding.c_str());
    }

    // Colormap and display the disparity image
    float min_disparity = disparity_msg->min_disparity;
    float max_disparity = disparity_msg->max_disparity;
    float multiplier = 255.0f / (max_disparity - min_disparity);

    assert(disparity_msg->image.encoding == enc::TYPE_32FC1);
    const cv::Mat_<float> dmat(disparity_msg->image.height, disparity_msg->image.width,
                               (float*)&disparity_msg->image.data[0], disparity_msg->image.step);
    disparity_color_.create(disparity_msg->image.height, disparity_msg->image.width);
    
    for (int row = 0; row < disparity_color_.rows; ++row) {
      const float* d = dmat[row];

      for (int col = 0; col < disparity_color_.cols; ++col) {
        int index = (d[col] - min_disparity) * multiplier + 0.5;
        index = std::min(255, std::max(0, index));
        // Fill as BGR
        disparity_color_(row, col)[2] = colormap[3*index + 0];
        disparity_color_(row, col)[1] = colormap[3*index + 1];
        disparity_color_(row, col)[0] = colormap[3*index + 2];
      }
    }

    // Must release the mutex before calling cv::imshow, or can deadlock against
    // OpenCV's window mutex.
    image_mutex_.unlock();
    if (!last_left_image_.empty()) {
      cv::imshow("left", last_left_image_);
      cv::waitKey(1);
    }
    if (!last_right_image_.empty()) {
      cv::imshow("right", last_right_image_);
      cv::waitKey(1);
    }
    cv::imshow("disparity", disparity_color_);
    cv::waitKey(1);
  }

  void saveImage(const char * prefix, const cv::Mat & image)
  {
    if (!image.empty()) {
      std::string filename = (filename_format_ % prefix % save_count_).str();
      cv::imwrite(filename, image);
      RCLCPP_INFO(this->get_logger(), "Saved image %s", filename.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Couldn't save %s image, no data!", prefix);
    }
  }
  
  static void mouseCb(int event, int x, int y, int flags, void * param)
  {
    (void)x;
    (void)y;
    (void)flags;

    if (event == cv::EVENT_LBUTTONDOWN) {
      // RCLCPP_WARN_ONCE(this->get_logger(), "Left-clicking no longer saves images. Right-click instead.");
      return;
    }

    if (event != cv::EVENT_RBUTTONDOWN) {
      return;
    }
    
    StereoViewNode *sv = (StereoViewNode*)param;
    std::lock_guard<std::mutex> guard(sv->image_mutex_);

    sv->saveImage("left",  sv->last_left_image_);
    sv->saveImage("right", sv->last_right_image_);
    sv->saveImage("disp",  sv->disparity_color_);
    sv->save_count_++;
  }

  void checkInputsSynchronized()
  {
    int threshold = 3 * all_received_;
    if (left_received_ >= threshold || right_received_ >= threshold || disp_received_ >= threshold) {
      RCLCPP_WARN(
        this->get_logger(),
        "[stereo_view] Low number of synchronized left/right/disparity triplets received.\n"
        "Left images received:      %d (topic '%s')\n"
        "Right images received:     %d (topic '%s')\n"
        "Disparity images received: %d (topic '%s')\n"
        "Synchronized triplets: %d\n"
        "Possible issues:\n"
        "\t* stereo_image_proc is not running.\n"
        "\t  Does `rosnode info %s` show any connections?\n"
        "\t* The cameras are not synchronized.\n"
        "\t  Try restarting stereo_view with parameter _approximate_sync:=True\n"
        "\t* The network is too slow. One or more images are dropped from each triplet.\n"
        "\t  Try restarting stereo_view, increasing parameter 'queue_size' (currently %d)",
        left_received_, left_sub_.getTopic().c_str(),
        right_received_, right_sub_.getTopic().c_str(),
        disp_received_, disparity_sub_.getTopic().c_str(),
        all_received_, this->get_name(), queue_size_);
    }
  }
};

}

int main(int argc, char ** argv)
{
  using image_view::StereoViewNode;

  rclcpp::init(argc, argv);

  std::string transport = argc > 1 ? argv[1] : "raw";

  rclcpp::NodeOptions options;
  auto sv_node = std::make_shared<StereoViewNode>(options, transport);
  
  rclcpp::spin(sv_node);
  return 0;
}
